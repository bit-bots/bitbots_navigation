#!/usr/bin/env python2
import numpy as np
import os
import cv2
import silx
import sys
import time
from silx.image import sift

class DemoSift():
    def __init__(self):
        # self.cap = cv2.VideoCapture(0)
        folder = os.curdir
        self.cap = cv2.VideoCapture(folder + "/test_data/imageset_373/frame%06d.png")
        self.side_key_points = [None,None]
        
        initframe = self.cap.read()[1]
        print (initframe)
        self.sift_ocl = sift.SiftPlan(template=initframe, devicetype="CPU")
        self.mp = sift.MatchPlan()
        # print("Device used for calculation: ", self.sift_ocl.ctx.devices[0].name)
        self.debug_image = None
        self.state = -1

    def compare(self, image):
        kp = self.sift_ocl(image)

        if not kp.shape[0]:
            return 0, 0

        matches = (self.mp(kp, self.side_key_points[0]), 
                   self.mp(kp, self.side_key_points[1]))

        # Debug
        image_with_matches = self.plot(image, self.convert_match(matches[0][:,0]), (255,0,0),6)
        image_with_matches = self.plot(image_with_matches, self.convert_match(matches[1][:,0]), (0,255,0),4)
        self.debug_image = self.plot(image_with_matches, kp, (0,0,255),2)

        # (Anzahl Keypoints Image1, Anzahl Keypoints Image2, Matched Keypoints)
        return matches[0].shape[0], matches[1].shape[0]


    def workloop(self):
        side = 0
        while True:
            time1 = time.time()
            ret, frame = self.cap.read()

            k = cv2.waitKey(1)

            if side < 2:
                self.debug_image = frame

                # Wurde SPACE gedrueckt
                if k%256 == 32:
                    self.initside(side, frame)
                    side += 1
            else:
                # Beginnt Vergleich
                common_keys = self.compare(frame)
                # print(common_keys)

                if common_keys[0] > common_keys[1]:
                    visible_side = 0
                else:
                    visible_side = 1

                if self.state != visible_side:
                    self.state = visible_side
                    # Debug
                print(common_keys)
                    # sys.stdout.write("  SEITE: %d   \r" % (self.state) )
                    # sys.stdout.flush()
                if abs(common_keys[0] - common_keys[1])/(float(sum(common_keys) + 1)) < 0.2:
                    self.state = "too common"
                
                if sum(common_keys) < 10:
                    self.state = "undefined"

                # Noch mehr Debug
                font = cv2.FONT_HERSHEY_SIMPLEX
                bottomLeftCornerOfText = (10,35)
                fontScale = 1
                fontColor = (255,255,255)
                lineType = 2

                cv2.putText(self.debug_image,"SIDE {}".format(self.state), 
                    bottomLeftCornerOfText, 
                    font, 
                    fontScale,
                    fontColor,
                    lineType)
                
            cv2.imshow("Video", self.debug_image)
            
            diff = float(1)/30 - (time.time() - time1)
            if diff > 0:
                time.sleep(diff)

            # Abbrechen mit ESC
            if k%256 == 27 or 0xFF == ord('q'):
                break
        self.cap.release() 
        cv2.destroyAllWindows()

    def initside(self, side, frame):
        print("Taken side {}!".format(side))
        self.side_key_points[side] = self.sift_ocl(frame)

    def convert_match(self, kps):
        d = np.dtype((np.record, [('x', '<f4'), ('y', '<f4'), ('scale', '<f4'), ('angle', '<f4'), ('desc', 'u1', (128,))]))
        return kps.astype(d)
    
    def plot(self, image, kp, color, size):
        for i in range(kp.shape[0]):
            cv2.circle(image, (kp[i].x, kp[i].y), size+int(kp[i].scale), color, thickness=2)
        return image

if __name__ == "__main__":
    demo = DemoSift()
    demo.workloop()
