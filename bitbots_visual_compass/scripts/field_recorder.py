#!/usr/bin/env python3

import cv2
import math
import yaml
import os
import time
import rospy
import actionlib

from .videocv import Videocv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from actionlib_msgs.msg import GoalStatus
from bitbots_msgs.msgs import JointCommand
from humanoid_league_msgs import PlayAnimationAction, PlayAnimationGoal

class FieldRecorder():
    """
    Records images of specific orientation on a field matrix.
    """
    def __init__(self):
        # Init Node
        rospy.init_node('FieldRecorder')
        rospy.loginfo('Initializing FieldRecorder...')

        self.cv_bridge = CvBridge()

        self.transfer_image_msg_read_flag = False
        self.transfer_image_msg = None
        self.basler_subscriber = None
        self.basler_image = None
        self.head_motor_goals_publisher = None
        self.logitech_video_getter = None

        # Load config
        self.dirname = os.path.dirname(__file__)
        relative_path = "../config/recorder.yaml"
        config_path = os.path.join(self.dirname, relative_path)

        with open(config_path, 'r') as stream:
            self.config = yaml.load(stream)

        # Config
        self.rows = self.config['recorder']['rows']
        self.checkpoints = self.config['recorder']['checkpoints']
        self.yaw_steps = self.config['recorder']['yaw_steps']
        slef.joint_resolution = self.config['recorder']['joint_resolution']
        self.data_location = os.path.join(self.dirname, self.config['recorder']['output_path'])
        self.output_path = os.path.join(self.data_location, "record_{}/".format(str(int(time.time()))))
        self.file_index = []

        # Subscribe to basler camera
        if self.config['recorder']['basler']:
            self.basler_subscriber = rospy.Subscriber(
                self.config['recorder']['basler_topic'],
                Image,
                self.basler_callback,
                queue_size=1,
                buff_size=60000000)

        # Open logitech camera
        if self.config['recorder']['logitech']:
            logitech_source = self.config['recorder']['logitech_source']
            if isinstance(logitech_source, basestring):
                root_folder = os.curdir
                logitech_source = root_folder + logitech_source
            
            self.logitech_video_getter = Videocv(logitech_source)
            self.logitech_video_getter.run()

        if self.config['recorder']['motor_control']:
            # Play standing animation
            animation_succeded = self.play_animation(self.config['recorder']['animation'])
            if not animation_succeded:
                rospy.logerr("Animation can not be played...")
                os.exit(1)

            # Publisher for JointComand
            self.head_motor_goals_publisher = rospy.Publisher('/head_motor_goals', JointCommand, queue_size=1)

        # Get start row and checkpoint from user input
        input_str = input("Select start (row, checkpoint) and/or press enter.")
        if input_str == "":
            self.start_row = 0
            self.start_checkpoint = 0
        else:
            split_input = input_str.split(',')
            self.start_row = int(split_input[0])
            self.start_checkpoint = int(split_input[1])

        self.display_map(self.start_row self.start_checkpoint)

        rospy.spin()

    def basler_callback(self, image_msg):
        image_age = rospy.get_rostime() - image_msg.header.stamp
        if 1.0 < image_age.to_sec() < 1000.0:
            rospy.logerr("Image message to old")
            os.exit(1)

        # Check flag
        if self.transfer_image_msg_read_flag:
            return

        # Transfer the image to the main thread
        self.transfer_image_msg = image_msg

    def basler_handle_image(self):
        self.transfer_image_msg_read_flag = True
        self.basler_image = self.cv_bridge.imgmsg_to_cv2(image_msg, 'bgr8')
        self.transfer_image_msg_read_flag = False

        # Skip if image is None
        if self.basler_image is None:
            rospy.logerr("Basler image content is None")
            os.exit(1)

    def play_animation(self, animation):
        animation_client = actionlib.SimpleActionClient('animation', PlayAnimationAction)
        first_try = animation_client.wait_for_server(
            rospy.Duration(rospy.get_param("hcm/anim_server_wait_time", 10)))
        if not first_try:
            rospy.logerr("Animation Action Server not running! Motion can not work without animation action server. "
                "Will now wait until server is accessible!")
            animation_client.wait_for_server()
            rospy.logwarn("Animation server now running, hcm will go on.")
        goal = PlayAnimationGoal()
        goal.animation = animation
        goal.hcm = False
        state = anim_client.send_goal_and_wait(goal)
        if state == GoalStatus.PENDING:
            print('Pending')
        elif state == GoalStatus.ACTIVE:
            print('Active')
        elif state == GoalStatus.PREEMPTED:
            print('Preempted')
        elif state == GoalStatus.SUCCEEDED:
            print('Succeeded')
        elif state == GoalStatus.ABORTED:
            print('Aborted')
        elif state == GoalStatus.REJECTED:
            print('Rejected')
        elif state == GoalStatus.PREEMPTING:
            print('Preempting')
        elif state == GoalStatus.RECALLING:
            print('Recalling')
        elif state == GoalStatus.RECALLED:
            print('Recalled')
        elif state == GoalStatus.LOST:
            print('Lost')
        else:
            print('Unknown state', state)

        if state == GoalStatus.SUCCEEDED:
            return True
        else:
            False

    def publish_head_motor_goals(self, motor, value):
        if motor == 0:
            pos_msg.joint_names = ["HeadPan"]
        if motor == 1:
            pos_msg.joint_names = ["HeadTilt"]

        pos_msg = JointCommand()
        pos_msg.velocities = [2]
        pos_msg.accelerations = [-1]
        pos_msg.max_currents = [-1]
        pos_msg.header.stamp = rospy.Time.now()
        pos_msg.positions = [value]

        self.head_motor_goals_publisher.publish(pos_msg)

    def show_img(self, basler_image, logitech_image):
        cv2.imshow("Basler", basler_image)
        cv2.imshow("Logitech", logitech_image)

    def make_path(self, path):
        try:  
            os.makedirs(path)
        except OSError:  
            rospy.logerr(f"Creation of directory '{path}' failed!")
            os.exit(1)
        else:  
            rospy.loginfo(f"Successfully created directory '{path}'.")

    def deg_to_val(self, angle):
        resolution = self.joint_resolution
        return resolution - int((angle/(2*math.pi))*resolution)

    def drive(self, motor, value):
        if self.config['recorder']['motor_control']:
            self.publish_head_motor_goals(motor, value)
        else:
            rospy.loginfo("Sim mode!!! Enable 'motor_control' in config to turn off.")
        rospy.logdebug(f"Drive Motor {motor} | Value {value}")

    def save_index(self):
        path = os.path.join(self.output_path, "index.yaml")
        try:
            with open(path, 'w') as outfile:
                yaml.dump(self.file_index, outfile, default_flow_style=False)
        except:
            rospy.logerr("Save index file error")
            os.exit(1)

    def save(self, row, checkpoint, value, angle, path, basler_image, logitech_image):
        basler_image_name = "basler_image_{}.png".format(value)
        basler_local_path = os.path.join("{}/{}/".format(row, checkpoint), basler_image_name)
        basler_save_path = os.path.join(path, basler_image_name)

        logitech_image_name = "logitech_image_{}.png".format(value)
        logitech_local_path = os.path.join("{}/{}/".format(row, checkpoint), logitech_image_name)
        logitech_save_path = os.path.join(path, logitech_image_name)

        data = {'row': row,
                'checkpoint': checkpoint,
                'angle': angle,
                'basler_path': basler_local_path,
                'logitech_path': logitech_local_path,}
        self.file_index.append(data)

        cv2.imwrite(basler_save_path, basler_image)
        cv2.imwrite(logitech_save_path, logitech_image)

    def display_map(self, draw_row, draw_checkpoint):
        offset = max(0,(self.checkpoints - 4 + 1))
        print(" " * offset + " ------- ")
        print(" " * offset + "| GOAL1 |")
        print("-" * (self.checkpoints * 2 + 3))
        for row in range(self.rows):
            str_row = "|"
            for checkpoint in range(self.checkpoints):
                if row == draw_row and checkpoint == draw_checkpoint:
                    symbol = "|X"
                else:
                    symbol = "|-"
                str_row = str_row + symbol
            print(str_row + "||")
        print("-" * (self.checkpoints * 2 + 3))
        print(" " * offset + "| GOAL2 |")
        print(" " * offset + " ------- ")

    def record_pano(self, row, checkpoint, path):
        steps = self.yaw_steps
        delta = (2*math.pi)/steps
        for step in range(steps):
            angle = step*delta
            value = self.deg_to_val(angle)
            self.drive(0,value)
            # Sleep until motor is positioned
            if step == 0:
                time.sleep(self.config['recorder']['reset_time'])
            time.sleep(self.config['recorder']['step_time'])
            
            logitech_image = self.video_getter.frame
            self.show_img(self.basler, logitech_image)

            k = cv2.waitKey(1)  # TODO: Why is this here?
            self.save(row, checkpoint, value, angle, path, self.basler_image, logitech_image)

    def record(self):
        skipall = False
        for row in range(start_row, self.rows):
            if not skipall:
                print("------------- NEW ROW -------------")
            for checkpoint in range(start_checkpoint, self.checkpoints):
                if self.video_getter.ended or skipall:
                    break
                checkpoints_path = os.path.join(self.output_path, "{}/{}/".format(row, checkpoint))
                input_str = raw_input("Press enter for next checkpoint!")
                print("Current Position:\nRow: {}\nCheckpoint:{}".format(row, checkpoint))
                if input_str == "s":
                    print("Skiped ({}|{})".format(row, checkpoint))
                elif input_str == "e":
                    print("Exit")
                    skipall = True
                else:
                    self.make_path(checkpoints_path)
                    self.record_pano(row, checkpoint, checkpoints_path)
                self.display_map(row, checkpoint)
            start_checkpoint = 0
        self.save_index()
        self.video_getter.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    FieldRecorder()
