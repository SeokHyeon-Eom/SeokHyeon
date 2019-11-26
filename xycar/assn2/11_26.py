#!/usr/bin/env python

import rospy, time

from linedetector import LineDetector
from obstacledetector import ObstacleDetector
from motordriver import MotorDriver


class AutoDrive:

    def __init__(self):
        rospy.init_node('xycar_driver')
        self.line_detector = LineDetector('/usb_cam/image_raw')
        self.obstacle_detector = ObstacleDetector('/ultrasonic')
        self.driver = MotorDriver('/xycar_motor_msg')

    def trace(self):
        obs_l, obs_m, obs_r = self.obstacle_detector.get_distance()
        line_l, line_r = self.line_detector.detect_lines()
        self.line_detector.show_images(line_l, line_r)
        angle = self.steer(line_l, line_r)
        speed = self.accelerate(angle, obs_l, obs_m, obs_r)
        self.driver.drive(angle + 90, speed + 90)

    def steer(self, left, right):
        mid = (left + right) // 2
        angle = 0
        if 600 <= right < 650:
            if mid > 360:
                angle -= 20
            else:
                angle += 0
        elif 510 <= right < 600:
            angle -= 30
            if mid < 280:
                angle -= 10
        else:
            angle -= 40
        
        if -10 <= left < 65:
            if mid < 280:
                angle += 20
            else:
                angle += 0
        elif 65 <= left < 120:
            angle += 30
            if mid > 360:
                angle += 10
        else:
            angle += 40
        
        print("reft",left,"mid",mid,"right", right,)
        print("")
        
        return angle

    def accelerate(self, angle, left, mid, right):
        if min(left, mid, right) < 50:
            speed = 0
        if angle < -20 or angle > 20:
            speed = 20
        else:
            speed = 30
        return speed

    def exit(self):
        print('finished')


if __name__ == '__main__':
    car = AutoDrive()
    time.sleep(3)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        car.trace()
        rate.sleep()
    rospy.on_shutdown(car.exit)


#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class LineDetector:

    def __init__(self, topic):
        # Initialize various class-defined attributes, and then...
        self.left, self.right = -1, -1
        self.roi_vertical_pos = 290
        self.scan_height = 20
        self.image_width = 640
        self.scan_width = 200
        self.area_width = 20
        self.area_height = 10
        self.row_begin = (self.scan_height - self.area_height) // 2
        self.row_end = self.row_begin + self.area_height
        self.cam_img = np.zeros(shape=(480, 640, 3), dtype=np.uint8)
        self.mask = np.zeros(shape=(self.scan_height, self.image_width),
                             dtype=np.uint8)
        self.edge = np.zeros(shape=(self.scan_height, self.image_width),
                             dtype=np.uint8)
        self.bridge = CvBridge()
        rospy.Subscriber(topic, Image, self.conv_image)

    def conv_image(self, data):
        self.cam_img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        v = self.roi_vertical_pos
        roi = self.cam_img[v:v + self.scan_height, :]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        avg_value = np.average(hsv[:, :, 2])
        value_threshold = avg_value * 1.3
        lbound = np.array([0, 0, value_threshold], dtype=np.uint8)
        ubound = np.array([100, 255, 255], dtype=np.uint8)
        self.mask = cv2.inRange(hsv, lbound, ubound)

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edge = cv2.Canny(blur, 10, 100)
        self.edge = cv2.cvtColor(edge, cv2.COLOR_GRAY2BGR)

    def detect_lines(self):
        lmid = self.scan_width
        rmid = self.image_width - lmid

        pixel_cnt_threshold = 0.3 * self.area_width * self.area_height

        self.cam_img = cv2.rectangle(self.cam_img, (0, self.roi_vertical_pos),
                                     (self.image_width - 1, self.roi_vertical_pos + self.scan_height),
                                     (255, 0, 0), 3)

        hsv2 = cv2.cvtColor(self.edge, cv2.COLOR_BGR2HSV)

        avg_value = np.average(hsv2[:, :, 2])
        value_threshold = avg_value * 1.0

        lbound_1 = np.array([0, 0, value_threshold], dtype=np.uint8)
        ubound_1 = np.array([255, 128, 255], dtype=np.uint8)

        bin_1 = cv2.inRange(hsv2, lbound_1, ubound_1)

        left_start = -1
        for l in range(lmid, self.area_width, -1):
            area = bin_1[self.row_begin:self.row_end, l: l + self.area_width]
            if cv2.countNonZero(area) > 1:
                left_start = l - self.area_width
                break

        right_start = -1
        for r in range(rmid, self.image_width - self.area_width):
            area = bin_1[self.row_begin:self.row_end, r - self.area_width:r]
            if cv2.countNonZero(area) > 1:
                right_start = r + self.area_width
                break

        for l in range(left_start, lmid):
            area = self.mask[self.row_begin:self.row_end, l: l + self.area_width]
            if cv2.countNonZero(area) > pixel_cnt_threshold:
                self.left = l
                break

        for r in range(right_start, rmid, -1):
            area = self.mask[self.row_begin:self.row_end, r - self.area_width: r]
            if cv2.countNonZero(area) > pixel_cnt_threshold:
                self.right = r
                break

        print("l", left_start, "start", "r", right_start)

        # Return positions of left and right lines detected.
        return self.left, self.right

    def show_images(self, left, right):
        # Display images for debugging purposes;
        # do not forget to call cv2.waitKey().
        pass
