#!/usr/bin/env python2
import cv2
import math
import yaml
import os
import time

from threading import Thread

class DavrosRecorder():
    """
    Records test data using the Davros vison robot.
    """
    def __init__(self):
        dirname = os.path.dirname(__file__)
        relative_path = "../config/recorder.yaml"
        config_path = os.path.join(dirname, relative_path)

        with open(config_path, 'r') as stream:
            self.config = yaml.load(stream)

        self.rows = self.config['recorder']['rows']
        self.checkpoints = self.config['recorder']['checkpoints']

        source = self.config['recorder']['input']

        data_location = os.path.join(dirname, self.config['recorder']['output_path'])

        self.output_path = os.path.join(data_location, "record_{}/".format(str(int(time.time()))))

        if isinstance(source, basestring):
            root_folder = os.curdir
            source = root_folder + source
        
        self.video_getter = VideoGet(source).start()

        self.file_index = []

        self.record()

    def show_img(self, image):
        cv2.imshow("Video", image)

    def make_path(self, path):
        try:  
            os.makedirs(path)
        except OSError:  
            print("Creation of the directory %s failed" % path)
        else:  
            print ("Successfully created the directory %s " % path)

    def deg_to_val(self, angle):
        resolution = 4095
        return int((angle/(2*math.pi))*resolution)

    def drive(self, motor, value):
        # TODO Drive Motor
        print("Drive Motor {} | Value {}".format(motor, value))

    def save(self, row, checkpoint, value, angle, path, image):
        name = "image_{}.png".format(value)
        local_path = os.path.join("{}/{}/".format(row, checkpoint), name)
        save_path = os.path.join(path, name)
        data = {'row': row,
                'checkpoint': checkpoint,
                'angle': angle,
                'path': local_path}
        self.file_index.append(data)
        cv2.imwrite(save_path, image)

    def save_index(self):
        path = os.path.join(self.output_path, "index.yaml")
        with open(path, 'w') as outfile:
            yaml.dump(self.file_index, outfile, default_flow_style=False)

    def record_pano(self, row, checkpoint, path):
        steps = self.config['recorder']['yaw_steps']
        delta = (2*math.pi)/steps
        for step in range(steps):
            angle = step*delta
            value = self.deg_to_val(angle)
            self.drive(1,value)
            # Sleep until camera is positioned
            # time.sleep(0.5)
            image = self.video_getter.frame
            self.show_img(image)
            k = cv2.waitKey(1)
            self.save(row, checkpoint, value, angle, path, image)
            # Abbrechen mit ESC
            if k%256 == 27 or 0xFF == ord('q') or self.video_getter.stopped:
                self.video_getter.stop()

    def record(self):
        for row in range(self.rows):
            print("------------- NEW ROW -------------")
            for checkpoint in range(self.checkpoints):
                if self.video_getter.stopped:
                    break
                checkpoints_path = os.path.join(self.output_path, "{}/{}/".format(row, checkpoint))
                self.make_path(checkpoints_path)
                raw_input("Press enter for next checkpoint!")
                self.record_pano(row, checkpoint, checkpoints_path)
                self.display_map(row, checkpoint)
        self.save_index()
        self.video_getter.stop()
        cv2.destroyAllWindows()

    def display_map(self, draw_row, draw_checkpoint):
        for row in range(self.rows):
            str_row = ""
            for checkpoint in range(self.checkpoints):
                if row == draw_row and checkpoint == draw_checkpoint:
                    symbol = "|X"
                else:
                    symbol = "|-"
                str_row = str_row + symbol
            print(str_row + "|")



class VideoGet:
    """
    Class that continuously gets frames from a VideoCapture object
    with a dedicated thread. # TODO Umschreiben
    """

    def __init__(self, src=0):
        self.FPS = float(30)
        self.stream = cv2.VideoCapture(src)
        (self.grabbed, self.frame) = self.stream.read()
        self.stopped = False
    
    def start(self):    
        Thread(target=self.get, args=()).start()
        return self

    def get(self):
        while not self.stopped:
            
            time.sleep(1/self.FPS)

            if not self.grabbed:
                self.stop()
            else:
                (self.grabbed, self.frame) = self.stream.read()

    def stop(self):
        self.stopped = True

if __name__ == "__main__":
    DavrosRecorder()