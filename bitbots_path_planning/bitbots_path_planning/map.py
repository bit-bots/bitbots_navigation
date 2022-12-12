from typing import Tuple

import cv2
import numpy as np
import soccer_vision_3d_msgs.msg as sv3dm
import tf2_ros as tf2
from humanoid_league_msgs.msg import PoseWithCertaintyStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from ros2_numpy import msgify, numpify
from tf2_geometry_msgs import PointStamped

OBSTACLE_VALUE = 50

class Map:
    def __init__(self, node: Node, buffer: tf2.Buffer) -> None:
        self.node = node
        self.buffer = buffer
        self.resolution = self.node.declare_parameter('map.resolution', 20).value
        self.size = (
            self.node.declare_parameter('map.size.x', 11.0).value,
            self.node.declare_parameter('map.size.y', 8.0).value
        )
        self.map = np.ones((np.array(self.size)*self.resolution).astype(int), dtype=np.int8)
        self.frame = self.node.declare_parameter('map.planning_frame', 'map').value
        self.ball_buffer = []
        self.robot_buffer = []
        self.config_ball_diameter = self.node.declare_parameter('map.ball_diameter', 0.13).value
        self.config_inflation_dialation = self.node.declare_parameter('map.inflation.dialte', 3).value
        self.config_inflation_blur = self.node.declare_parameter('map.inflation.blur', 13).value

    def set_ball(self, ball: PoseWithCertaintyStamped):
        point = PointStamped()
        point.header.frame_id = ball.header.frame_id
        point.point = ball.pose.pose.pose.position
        self.ball_buffer = [self.buffer.transform(point, self.frame).point]

    def render_balls(self) -> None:
        ball: sv3dm.Ball
        for ball in self.ball_buffer:
            cv2.circle(
                self.map,
                self.to_map_space(ball.x, ball.y)[::-1],
                round(self.config_ball_diameter * self.resolution),
                OBSTACLE_VALUE,
                -1)

    def set_robots(self, robots: sv3dm.RobotArray):
        new_buffer = []
        robot: sv3dm.Robot
        for robot in robots.robots:
            point = PointStamped()
            point.header.frame_id = robots.header.frame_id
            point.point = robot.bb.center.position
            new_buffer.append((robot, self.buffer.transform(point, self.frame).point))
        self.robot_buffer = new_buffer

    def render_robots(self) -> None:
        robot: sv3dm.Robot
        for robot in self.robot_buffer:
            cv2.circle(
                self.map,
                self.to_map_space(robot[1].x, robot[1].y)[::-1],
                round(max(numpify(robot[0].bb.size)[:2]) * self.resolution),
                OBSTACLE_VALUE,
                -1)

    def to_map_space(self, x: float, y: float) -> Tuple[int, int]:
        return (
            max(0, min(round((x - self.get_origin()[0]) * self.resolution), self.map.shape[0] - 1)),
            max(0, min(round((y - self.get_origin()[1]) * self.resolution), self.map.shape[1] - 1)),
        )

    def from_map_space_np(self, points: np.ndarray) -> np.ndarray:
        return points / self.resolution + self.get_origin()

    def get_origin(self) -> np.ndarray:
        """
        Origin in Meters
        """
        return np.array([
            -self.map.shape[0] / self.resolution / 2,
            -self.map.shape[1] / self.resolution / 2
        ])

    def clear(self) -> None:
        self.map[...] = 1

    def inflate(self) -> None:
        idx = self.map == 1
        map = cv2.dilate(
            self.map.astype(np.uint8),
            np.ones((
                self.config_inflation_dialation,
                self.config_inflation_dialation), np.uint8),
            iterations=2)
        self.map[idx] = cv2.blur(
            map,
            (self.config_inflation_blur, self.config_inflation_blur)
        ).astype(np.int8)[idx]

    def update(self):
        self.clear()
        self.render_balls()
        self.render_robots()
        self.inflate()

    def get_map(self):
        return self.map

    def get_frame(self) -> str:
        return self.frame

    def to_msg(self) -> OccupancyGrid:
        msg: OccupancyGrid = msgify(OccupancyGrid, self.get_map().T)
        msg.header.frame_id = self.frame
        msg.info.width = self.map.shape[0]
        msg.info.height = self.map.shape[1]
        msg.info.resolution = 1 / self.resolution
        msg.info.origin.position.x = self.get_origin()[0]
        msg.info.origin.position.y = self.get_origin()[1]
        return msg