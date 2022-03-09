#!/usr/bin/python3
# coding=utf8
import sys
import os

from cv2 import undistort

sys.path.append(os.path.dirname(os.getcwd()))

from MoveArm import MoveArm
import cv2
import time
import Camera
import threading
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from Perception import *
from CameraCalibration.CalibrationConfig import *
from RossROS.rossros import * 

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

#AK = ArmIK()

if __name__ == '__main__':

    done = False # arm state variable to end program
    aruco_perception = Perception() # 
    arm_IK = MoveArm()

    #Create RossROS busses
    #     
    raw_camera_frame_bus = Bus(name = "undistorted frame")
    post_detection_frame_bus = Bus(name = "undistorted frame")
    detection_status_bus = Bus(name = "undistorted frame")
    cup2base_distance_bus = Bus(name = "undistorted frame")

    termination_bus = Bus(name = "termination bus")

    #timer class to control runtime of the script
    timer = Timer(timer_busses = termination_bus, duration = 100, 
                  delay = 1, name = "termination timer") 

    #Creating perception RossROS services 

    camera_feed_service = Producer(aruco_perception.getRawCameraFrame, 
                                   output_busses = raw_camera_frame_bus,
                                   delay = 0.2,
                                   termination_busses = termination_bus,
                                   name = "raw camera images")


    perception_service = ConsumerProducer(aruco_perception.arucoDetect, 
                                          input_busses = raw_camera_frame_bus,
                                          output_busses = (post_detection_frame_bus,
                                                           detection_status_bus,
                                                           cup2base_distance_bus),
                                          delay = 0.1,
                                          termination_busses = termination_bus,
                                          name = "detect aruco markers")


    list_of_concurrent_services  = [camera_feed_service.__call__, 
                                    perception_service.__call__,
                                    ]

    
    runConcurrently(list_of_concurrent_services)
