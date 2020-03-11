from __future__ import absolute_import

import math
import statistics
import cv2
import numpy as np
import rospy
from .feature_detector import FeatureDetector


class Compass(VisualCompassInterface):
    def __init__(self, config):
        self.config = config

        # ([keypoints], [descriptors])
        self.feature_map = ([], [])

        # (angle, confidence, matching_count)
        self.state = (None, None)
        self.feature_detector = FeatureDetector(self.config)

        # TODO set camera matrix from camera info message
        self.rotation_matrix = np.array([[1058.439188,   0.000000,       310.612349],
                                    [0.000000,      1092.738996,    229.659998],
                                    [0.000000,      0.000000,       1.000000]])

        # config values
        self.mean_feature_count = None
        self.mean_feature_count_list = []
        # Factor which describes the weight of the feature count
        self.mean_feature_count_seed = None

        self.set_config(config)

    def set_truth(self, angle, image):
        if 0 <= angle <= 2*math.pi:
            keypoints, descriptors = self.feature_detector.get_keypoints(image)

            self.feature_map[0].extend(self.tag_keypoints(angle, keypoints))
            self.feature_map[1].extend(descriptors)
            self.mean_feature_count_list.append(len(descriptors) / self.mean_feature_count_seed)
            self.mean_feature_count = statistics.mean(self.mean_feature_count_list)
            print("mean_feature_count " + str(self.mean_feature_count))
        else:
            print("Angle out of bounds")

    def get_feature_map(self):
        return self.feature_map

    def get_mean_feature_count(self):
        return self.mean_feature_count

    def set_feature_map(self, feature_map):
        self.feature_map = feature_map

    def set_mean_feature_count(self, mean_feature_count):
        self.mean_feature_count = mean_feature_count

    def _compute_state(self, matching_keypoints):
        # Extract angles from keypoints
        angles = list(map(lambda x: x.angle, matching_keypoints))

        # Catch too low matches
        length = len(angles)
        if length < 2:
            return .0, .0

        # Calculate the angle and confidence
        z = sum(map(lambda angle: np.exp(1j * angle), angles))
        median = np.angle(z) % (math.pi * 2)
        confidence = (float(np.abs(z)) / length)
        confidence *= 1 - math.exp(-(1. / self.mean_feature_count) * length)

        return median, confidence

    def process_image(self, image, resultCB=None):
        current_keypoints, current_descriptors = self.feature_detector.get_keypoints(image)

        matching_keypoints = self.feature_detector.match(self.feature_map[0], np.array(self.feature_map[1]), current_descriptors)

        self.state = self._compute_state(matching_keypoints)

        if resultCB is not None:
            resultCB(*self.state)

        return self.state[0], self.state[1]

    def set_config(self, config):
        self.config = config
        self.mean_feature_count_seed = float(config['compass_multiple_mean_feature_seed'])
        self.feature_detector.set_config(config)

    def tag_keypoints(self, offset, keypoints):
        return [self.tag_one_keypoint(offset, keypoints[i]) for i in range(len(keypoints))]

    def tag_one_keypoint(self, offset, keypoint):
        vector = np.array([keypoint.pt[0], keypoint.pt[1], 1.])
        x = np.linalg.inv(self.rotation_matrix).dot(vector)[0]
        angle = (math.atan(x) + offset) % (math.pi * 2)
        return cv2.KeyPoint(.0, .0, offset, angle)
