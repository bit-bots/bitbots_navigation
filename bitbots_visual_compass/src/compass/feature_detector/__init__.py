from __future__ import absolute_import

import cv2
# import rospy

from .orb_feature_detector import OrbFeatureDetector
from .sift_feature_detector import SiftFeatureDetector
from .akaze_feature_detector import AkazeFeatureDetector
from .interface import FeatureDetector as FeatureDetectorInterface

class FeatureDetector (FeatureDetectorInterface):
    def __init__(self, config):
        self.feature_detector = None
        self.feature_detectorType = None
        self.feature_detectorClasses = {
            "akaze": AkazeFeatureDetector,
            "orb": OrbFeatureDetector,
            "sift": SiftFeatureDetector
        }

        self.set_config(config)

    def match(self, keypoints, descriptor1, descriptor2):
        return self.feature_detector.match(keypoints, descriptor1, descriptor2)

    def get_keypoints(self, image):
        return self.feature_detector.get_keypoints(image)

    def debug_keypoints(self, image, debug_keypoints, color):
        return self.feature_detector.debug_keypoints(image, debug_keypoints, color)

    def set_config(self, config):
        feature_detector_type = config['compass_feature_detector']
        if feature_detector_type == self.feature_detectorType:
            self.feature_detector.set_config(config)
        else:
            self.feature_detectorType = feature_detector_type
            if feature_detector_type not in self.feature_detectorClasses:
                raise AssertionError(self.feature_detectorType + ": FeatureDetector not available!")
            feature_detector_class = self.feature_detectorClasses[self.feature_detectorType]
            self.feature_detector = feature_detector_class(config)
