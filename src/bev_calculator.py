#!usr/bin/env python2

import sys, os
import math
import numpy as np
import tf
import cv2
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

from skimage.measure import compare_ssim as ssim
from skimage.measure import compare_psnr as psnr

class BevCalculator:
    def __init__(self):
        rospy.init_node('bev_calculator', anonymous=True)

        # param
        self.HZ = rospy.get_param("~HZ", 10)

        self.true_img = None
        self.flow_img = None

        # subscriber
        true_img_sub = rospy.Subscriber('/bev_true/true_flow_image',Image ,self.true_img_callback)
        flow_img_sub = rospy.Subscriber('/bev/flow_image' ,Image ,self.flow_img_callback)

        # publisher
        self.measurement_val_pub = rospy.Publisher('/bev_evaluate/mesurement_val', Float32MultiArray, queue_size = 10)

    def true_img_callback(self, data):
        self.true_img = data

    def flow_img_callback(self, data):
        self.flow_img = data

    def ssim_measurement(self, true_img, flow_img):
        return measurement(ssim, true_img, flow_img)

    def psnr_measurement(self, true_img, flow_img):
        return measurement(psnr, true_img, flow_img)

    def process(self):
        r = rospy.Rate(self.HZ)
        while not rospy.is_shutdown():
            measurement_val = [0.0, 0.0]
            print("calculate")
            if self.true_img is not None and self.flow_img is not None:
                measurement_val[1] = ssim_measurement(self.true_img, self.flow_img)
                measurement_val[2] = psnr_measurement(self.true_img, self.flow_img)
            self.measurement_val_pub.publish(2, measurement_val)
            print("result")
            print(measurement_val)
            r.sleep()


if __name__ == '__main__':
    bc = BevCalculator()
    try:
        bc.process()
    except rospy.ROSInterruptException:
        pass
