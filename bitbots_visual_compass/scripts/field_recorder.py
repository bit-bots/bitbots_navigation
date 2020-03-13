#!/usr/bin/env python3

import cv2
import math
import yaml
import os
import sys
import time
import rospy
import actionlib

from copy import deepcopy
from videocv import Videocv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from actionlib_msgs.msg import GoalStatus
from bitbots_msgs.msg import JointCommand
from bitbots_buttons.msg import Buttons
from humanoid_league_msgs.msg import PlayAnimationAction, PlayAnimationGoal

class FieldRecorder():
    """
    Records images of specific orientation on a field matrix.
    """
    def __init__(self):
        # Init Node
        rospy.init_node('FieldRecorder')
        rospy.loginfo('Initializing FieldRecorder...')

        self.cv_bridge = CvBridge()

        # Image transfer variable
        self._transfer_basler_image_msg = None
        self._transfer_basler_image_msg_read_flag = False

        self.button_subscriber = None
        self.basler_subscriber = None
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
        self.joint_resolution = self.config['recorder']['joint_resolution']
        self.data_location = os.path.join(self.dirname, self.config['recorder']['output_path'])
        self.output_path = os.path.join(self.data_location, "record_{}/".format(str(int(time.time()))))
        self.file_index = []

        # Get start row and checkpoint from user input
        input_str = input("Select Start Position (ROW, CHECKPOINT) default: (0,0) and press ENTER: ")
        try:
            split_input = input_str.split(',')
            self.start_row = int(split_input[0].strip())
            self.start_checkpoint = int(split_input[1].strip())
        except:
            self.start_row = 0
            self.start_checkpoint = 0

        rospy.loginfo(f"Start Positions: Row: {self.start_row} | Checkpoint: {self.start_checkpoint}")

        if self.config['recorder']['logitech']:
            # Open logitech camera
            logitech_source = self.config['recorder']['logitech_source']
            if isinstance(logitech_source, str):
                root_folder = os.curdir
                logitech_source = root_folder + logitech_source

            self.logitech_video_getter = Videocv(logitech_source)
            self.logitech_video_getter.run()

        if self.config['recorder']['basler']:
            # Subscribe to basler camera
            self.basler_subscriber = rospy.Subscriber(
                self.config['recorder']['basler_topic'],
                Image,
                self.basler_callback,
                queue_size=1,
                buff_size=60000000)

        if self.config['recorder']['use_buttons']:
            # Subscribe to button
            self.button_subscriber = rospy.Subscriber(
                "/buttons",
                Buttons,
                self.button_callback)
            self.button = False

        if self.config['recorder']['motor_control']:
            # Publisher for JointComand
            self.head_motor_goals_publisher = rospy.Publisher('/head_motor_goals', JointCommand, queue_size=1)

        self.record()

    def record(self):
        skipall = False
        for row in range(self.start_row, self.rows):
            rospy.loginfo("------------- NEW ROW -------------")
            for checkpoint in range(self.start_checkpoint, self.checkpoints):
                if rospy.is_shutdown() or ((self.logitech_video_getter is not None) and self.logitech_video_getter.ended) or skipall:
                    rospy.loginfo("Shutting down")
                    return
                else:
                    self.display_map(row, checkpoint)
                    if self.button_subscriber is not None:
                        input_str = "not pressed"
                        while input_str == "not pressed":
                            if self.button:
                                input_str = "\n"
                            time.sleep(0.1)
                    else:
                        input_str = input("Press ENTER for next checkpoint | 's' to skip | 'e' to exit: ")
                    if input_str.lower() == "s":
                        rospy.logdebug(f"Skipped Position: Row: {row} | Checkpoint: {checkpoint}")
                    elif input_str.lower() == "e":
                        rospy.loginfo("Exit...")
                        skipall = True
                    else:
                        checkpoints_path = os.path.join(self.output_path, "{}/{}/".format(row, checkpoint))
                        self.make_path(checkpoints_path)
                        self.record_pano(row, checkpoint, checkpoints_path)
                    self.save_index()
            start_checkpoint = 0
        if self.logitech_video_getter is not None:
            self.logitech_video_getter.stop()
        cv2.destroyAllWindows()

    def basler_callback(self, image_msg):
        image_age = rospy.get_rostime() - image_msg.header.stamp
        if 1.0 < image_age.to_sec() < 1000.0:
            rospy.logerr("Image message to old")

        # Check flag
        if self._transfer_basler_image_msg_read_flag:
            return

        # Transfer the image to the main thread
        self._transfer_image_msg = image_msg

    def button_callback(self, msg):
        """Callback for msg about pressed buttons."""
        print(msg.button1)
        self.button = msg.button1

    def basler_handle_image(self, image_msg):
        image = self.cv_bridge.imgmsg_to_cv2(image_msg, 'bgr8')

        # Skip if image is None
        if image is None:
            rospy.logerr("Basler image content is None")
            sys.exit(1)
        else:
            return image

    def publish_head_motor_goals(self, motor, value):
        pos_msg = JointCommand()
        if motor == 0:
            pos_msg.joint_names = ["HeadPan"]
        if motor == 1:
            pos_msg.joint_names = ["HeadTilt"]

        pos_msg.velocities = [2]
        pos_msg.accelerations = [-1]
        pos_msg.max_currents = [-1]
        pos_msg.header.stamp = rospy.Time.now()
        pos_msg.positions = [value]

        self.head_motor_goals_publisher.publish(pos_msg)

    def show_img(self, images):
        if self.config['recorder']['show_images']:
            # Data is a dict: images[camera_name] = image
            for camera_name, image in images.items():
                cv2.imshow(camera_name, image)

    def make_path(self, path):
        try:
            os.makedirs(path)
        except OSError:
            rospy.logerr(f"Creation of directory '{path}' failed!")
            sys.exit(1)
        else:  
            rospy.logdebug(f"Successfully created directory '{path}'.")

    def drive(self, motor, value):
        if self.config['recorder']['motor_control']:
            self.publish_head_motor_goals(motor, value)
        else:
            rospy.loginfo("Sim mode!!! Enable 'motor_control' in config to turn off.")
        rospy.logdebug(f"Drive Motor {motor} | Value {value}")

    def save_index(self):
        if self.file_index:
            path = os.path.join(self.output_path, "index.yaml")
            try:
                with open(path, 'w') as outfile:
                    yaml.dump(self.file_index, outfile, default_flow_style=False)
            except:
                rospy.logerr("Save index file error")
                sys.exit(1)

    def save(self, row, checkpoint, value, angle, path, images):
        meta = {'row': row,
                'checkpoint': checkpoint,
                'angle': angle,}
        
        # Data is a dict: images[camera_name] = image
        for camera_name, image in images.items():
            image_name = f"{camera_name}_{value}.png"
            local_image_path = os.path.join("{}/{}/".format(row, checkpoint), image_name)
            meta[camera_name] = local_image_path

            image_save_path = os.path.join(path, image_name)
            cv2.imwrite(image_save_path, image)

        self.file_index.append(meta)

    def display_map(self, current_row, current_checkpoint):
        offset = max(0,(self.checkpoints - 4))

        current_map = """"""
        current_map = current_map + " " * offset + "---------\n"
        current_map = current_map + " " * offset + "| GOAL1 |\n"
        current_map = current_map + "-" * (self.checkpoints * 2 + 3) + "\n"
        for row in range(self.rows):
            str_row = "|"
            for checkpoint in range(self.checkpoints):
                if row == current_row and checkpoint == current_checkpoint:
                    symbol = "|X"
                else:
                    symbol = "|-"
                str_row = str_row + symbol
            current_map = current_map + str_row + "||\n"
        current_map = current_map + "-" * (self.checkpoints * 2 + 3) + "\n"
        current_map = current_map + " " * offset + "| GOAL2 |\n"
        current_map = current_map + " " * offset + "---------\n"

        rospy.loginfo(f"Current Positions: Row: {row} | Checkpoint: {checkpoint}\n{current_map}")

    def record_pano(self, row, checkpoint, path):
        steps = self.yaw_steps
        delta = (2*math.pi)/steps
        for step in range(steps):
            angle = step*delta
            value = angle - math.pi
            self.drive(0,value)
            # Sleep until motor is positioned
            if step == 0:
                time.sleep(self.config['recorder']['reset_time'])
            time.sleep(self.config['recorder']['step_time'])

            images = {}

            if self.logitech_video_getter is not None:
                logitech_image = self.logitech_video_getter.frame
                images['logitech'] = logitech_image

            if self.basler_subscriber is not None:
                # Copy image from shared memory
                self._transfer_basler_image_msg_read_flag = True
                image_msg = deepcopy(self._transfer_image_msg)
                self._transfer_basler_image_msg_read_flag = False
                self._transfer_image_msg = None
                images['basler'] = self.basler_handle_image(image_msg)

            self.save(row, checkpoint, value, angle, path, images)
            self.show_img(images)
            k = cv2.waitKey(1)  # TODO: Why is this here?


if __name__ == "__main__":
    FieldRecorder()
