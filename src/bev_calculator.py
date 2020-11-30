#!usr/bin/env python2

import sys, os
import math
import numpy as np
import tf
import cv2
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose2D

from skimage.measure import compare_ssim as ssim
from skimage.measure import compare_psnr as psnr

import openpyxl

class BevCalculator:
    def __init__(self):
        rospy.init_node('bev_calculator', anonymous=True)

        # param
        self.HZ = rospy.get_param("~HZ", 10)
        self.excel_file_name = rospy.get_param("EXCEL_FILE_NAME", "/records/records.xlsx")

        self.true_img = None
        self.flow_img = None
        self.current_yaw = None

        # subscriber
        true_img_sub = rospy.Subscriber('/bev_true/true_flow_image',Image ,self.true_img_callback)
        flow_img_sub = rospy.Subscriber('/bev/flow_image' ,Image ,self.flow_img_callback)
        current_pose2d_sub = rospy.Subscriber('/bev_true/current_yaw', Pose2D, self.current_pose2d_callback)

        # publisher
        self.measurement_val_pub = rospy.Publisher('/bev_evaluator/mesurement_val', Float32MultiArray, queue_size = 10)
        self.true_flow_img = rospy.Publisher('/bev_evaluator/true_flow_image', Image, queue_size = 10)

    def true_img_callback(self, data):
        self.true_img = data

    def flow_img_callback(self, data):
        self.flow_img = data

    def current_pose2d_callback(self, data):
        self.current_yaw = data.theta

    def ssim_measurement(self, true_img, flow_img):
        return measurement(ssim, true_img, flow_img)

    def psnr_measurement(self, true_img, flow_img):
        return measurement(psnr, true_img, flow_img)

    def process(self):
        r = rospy.Rate(self.HZ)
        # book = openpyxl.load_workbook(filename=self.excel_file_name)
        # i = 0
        while not rospy.is_shutdown():
            measurement_val = [0.0, 0.0]
            print("calculate")
            if self.true_img is not None and self.flow_img is not None:
                measurement_val[1] = ssim_measurement(self.true_img, self.flow_img)
                measurement_val[2] = psnr_measurement(self.true_img, self.flow_img)
                # sheet.cell(row=i, column=1).value = measurement_val[1]
                # sheet.cell(row=i, column=2).value = measurement_val[2]
                # i += 1
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
