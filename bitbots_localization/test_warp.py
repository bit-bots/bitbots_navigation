import numpy as np
import math
from transforms3d.affines import compose
from transforms3d.quaternions import quat2mat
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import cv2
from sensor_msgs.msg import CameraInfo, Image
import threading
from geometry_msgs.msg import PoseStamped


class Warper(Node):
    def __init__(self):
        super().__init__('warper')
        self.camera_info_sub = self.create_subscription(        
            CameraInfo,
            'camera/camera_info',
            self.camera_info_callback,
            10)
        self.camera_info_sub  # prevent unused variable warning
        self.p = None
        self.map = cv2.imread('map.png')
        self.map_height, self.map_width, _ = self.map.shape
        self.map_corners = np.float32([[0,0],[self.map_width,0],[self.map_width,self.map_height],[0,self.map_height]])
        self.field_corners = np.float32([
            [-5.5, 4, 0, 1],  # lb
            [5.5, 4, 0, 1],  # lf
            [5.5, -4, 0, 1],  # rf
            [-5.5, -4, 0, 1],  # rb
        ])
        # create subscriber for vector3
        self.create_subscription(
            PoseStamped,
            '/pose',
            self.pose_callback,
            10)
        # create image publisher
        self.image_pub = self.create_publisher(Image, '/image', 10)

    def camera_info_callback(self, msg):
        self.p = np.array(msg.p).reshape(3,4)

    def pose_callback(self, msg):
        if self.p is None:
            return
        base_footprint_to_cof = np.array(
         [-0.000,  0.003,  1.000,  0.022,
         -1.000, -0.003,  0.000,  0.003,
          0.003, -1.000,  0.003,  0.767,
          0.000  ,0.000  ,0.000  ,1.000]
        )
        base_footprint_to_cof = base_footprint_to_cof.reshape(4,4)

        def transform(point, pos_transform):
            point = np.linalg.inv(pos_transform) @ point
            point_in_cof = np.linalg.inv(base_footprint_to_cof) @ point
            pxs = self.p @ point_in_cof
            return pxs[2] > 0, pxs[0]/pxs[2], pxs[1]/pxs[2]

        pos_x = msg.pose.position.x
        pos_y = msg.pose.position.y
        quat = [msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z]
        pos_transform = compose([pos_x, pos_y, 0], quat2mat(quat), np.ones(3))
        points = [transform(point, pos_transform) for point in self.field_corners]
        bad_points = [p for p in points if not p[0]]
        points = [p[1:] for p in points]
        goal_points = np.float32(points)
        warp = cv2.getPerspectiveTransform(self.map_corners, goal_points)
        dst = cv2.warpPerspective(self.map,warp,(800,600))
        lowest_bad_point = max(bad_points, key=lambda x: x[2])
        # rectangle in top of image
        cv2.rectangle(dst, (0,0), (800, int(lowest_bad_point[2])), (0,0,0), -1)
        # publish image
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = dst.shape[0]
        msg.width = dst.shape[1]
        msg.encoding = 'bgr8'
        msg.is_bigendian = False
        msg.step = dst.shape[1] * 3
        msg.data = dst.tobytes()
        self.image_pub.publish(msg)


if __name__ == '__main__':
    rclpy.init()
    ex = MultiThreadedExecutor()
    node = Warper()
    ex.add_node(node)
    ex.spin()
