import rclpy
import soccer_vision_3d_msgs.msg as sv3dm
import tf2_ros as tf2
from bitbots_path_planning.controller import Controller
from bitbots_path_planning.map import Map
from bitbots_path_planning.planner import Planner
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Empty

from humanoid_league_msgs.msg import PoseWithCertaintyStamped


class PathPlanning(Node):
    """
    A minimal python path planning.
    """
    def __init__(self) -> None:
        super().__init__('bitbots_path_planning')

        # declare maximum speed, TODO: use ros params and find the right speed
        self.max_speed = 0.5
        self.max_angular_speed = 0.5

        # Declare params
        self.declare_parameter('base_footprint_frame', 'base_footprint')
        self.declare_parameter('rate', 20.0)

        # We need to create a tf buffer
        self.tf_buffer = tf2.Buffer(
            cache_time=Duration(seconds=self.declare_parameter('tf_buffer_duration', 5.0).value))
        self.tf_listener = tf2.TransformListener(self.tf_buffer, self)

        # Create submodules
        self.map = Map(node=self, buffer=self.tf_buffer)
        self.planner = Planner(node=self, buffer=self.tf_buffer, map=self.map)
        self.controller = Controller(node=self, buffer=self.tf_buffer)

        # Subscribe
        callback_group = ReentrantCallbackGroup()
        self.create_subscription(
            PoseWithCertaintyStamped,
            self.declare_parameter('map.ball_update_topic', 'ball_relative_filtered').value,
            self.map.set_ball,
            5,
            callback_group=callback_group)
        self.create_subscription(
            sv3dm.RobotArray,
            self.declare_parameter('map.robot_update_topic', 'robots_relative_filtered').value,
            self.map.set_robots,
            5,
            callback_group=callback_group)
        self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.planner.set_goal,
            5,
            callback_group=callback_group)
        self.create_subscription(
            Empty,
            'move_base/cancel',
            lambda _: self.planner.cancel(),
            5,
            callback_group=callback_group)

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.costmap_pub = self.create_publisher(OccupancyGrid, 'costmap', 1)
        self.path_pub = self.create_publisher(Path, 'path', 1)

        # Timer that updates the path and command velocity at a given rate
        self.create_timer(1 / self.get_parameter('rate').value, self.step, clock=self.get_clock())

    def step(self) -> None:
        """
        Performs a single step of the path planning
        """
        self.map.update()
        self.costmap_pub.publish(self.map.to_msg())

        if self.planner.active():
            path = self.planner.step()
            self.path_pub.publish(path)

            cmd_vel = self.controller.step(path)
            # if cmd_vel is below threshold, then publish, otherwise publish a maximum speed
            if not cmd_vel.linear.x + cmd_vel.linear.y < self.max_speed and cmd_vel.angular.z < self.max_angular_speed:
                x_rel = cmd_vel.linear.x / (cmd_vel.linear.x + cmd_vel.linear.y)
                y_rel = 1 - x_rel
                cmd_vel.linear.x = self.max_speed * x_rel
                cmd_vel.linear.y = self.max_speed * y_rel
                cmd_vel.angular.z = self.max_angular_speed
            self.cmd_vel_pub.publish(cmd_vel)


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanning()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    ex.spin()
    node.destroy_node()
    rclpy.shutdown()
