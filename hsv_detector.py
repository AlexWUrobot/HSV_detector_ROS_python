#!/usr/bin/env python
# -*- coding: utf-8 -*-


import cv2
import argparse
from operator import xor
from rgb_camera_robot_sensor import RobotRGBSensors
import rospy
import numpy as np


def callback(value):
    pass


def setup_trackbars(range_filter):
    cv2.namedWindow("Trackbars", 0)

    for i in ["MIN", "MAX"]:
        v = 0 if i == "MIN" else 255

        for j in range_filter:
            cv2.createTrackbar("%s_%s" % (j, i), "Trackbars", v, 255, callback)


def get_trackbar_values(range_filter):
    values = []

    for i in ["MIN", "MAX"]:
        for j in range_filter:
            v = cv2.getTrackbarPos("%s_%s" % (j, i), "Trackbars")
            values.append(v)

    return values


# construct the argument parser to parse input static image
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required=False, help="path to the input image")
args = vars(ap.parse_args())
print(args)
image_path = args['image']


def main():
    if not image_path:
        rospy.init_node("hsv_detector", log_level=rospy.DEBUG)
        rospy.logwarn("Starting....")

        # sensors_obj = RobotRGBSensors("/camera/rgb/image_raw")
        # sensors_obj = RobotRGBSensors("/catvehicle/camera_front/image_raw_front")
        # sensors_obj = RobotRGBSensors("/wamv/sensors/cameras/front_right_camera/image_raw") # for Task 6 shooting
        sensors_obj = RobotRGBSensors("/camera/rgb/image_raw") # for Task 1-5

        cv_image = sensors_obj.get_image()

    range_filter = "HSV"

    setup_trackbars(range_filter)

    # Load and convert static image once
    if image_path:
        image = cv2.imread(image_path)
        frame_to_thresh = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    pre_mask = None
    while not rospy.is_shutdown():
        if not image_path:
            image = sensors_obj.get_image()
            #image = cv2.fastNlMeansDenoisingColored(image, None, 10, 10, 7, 21)
            if image is None:
                continue
            else:
                frame_to_thresh = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        v1_min, v2_min, v3_min, v1_max, v2_max, v3_max = get_trackbar_values(range_filter)

        thresh = cv2.inRange(frame_to_thresh, (v1_min, v2_min, v3_min), (v1_max, v2_max, v3_max))
        blur_red =  cv2.GaussianBlur(thresh, (5, 5), 0)  # reduce noise
        kernel = np.ones((5, 5), np.uint8)
        blur_red = cv2.dilate(thresh, kernel, iterations=2)

        # cv2.imshow("thresh", thresh)

        # cv2.imshow("thresh", thresh)

        preview = cv2.bitwise_and(image, image, mask=blur_red)
        # preview = cv.cvSet(preview, (0, 255, 0))
        # preview[np.where(preview == [1])] = (0, 255, 0)
        #preview[np.where(preview == [0])] = [255]   # for detect black

        cv2.imshow("Preview", preview)

        cv2.imshow("thresh", blur_red)
        if cv2.waitKey(1) & 0xFF is ord('q'):
            break

    rospy.logwarn("Shutting down")

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
