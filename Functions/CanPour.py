#!/usr/bin/python3
# coding=utf8
import sys
import os

from cv2 import undistort

sys.path.append(os.path.dirname(os.getcwd()))

#from MoveArm import MoveArm
from Motion import Motion
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
    arm_ik = Motion()

    #Create RossROS busses
    #     
    perception_bus = Bus(name = "undistorted frame")

    termination_bus = Bus(name = "termination bus")

    #timer class to control runtime of the script
    timer = Timer(timer_busses = termination_bus, duration = 100, 
                  delay = 1, name = "termination timer") 

    #Creating perception RossROS services 
    perception_service = Producer(aruco_perception.aruco_detect, 
                                  output_busses = perception_bus,
                                  delay = 0.2,
                                  termination_busses = termination_bus,
                                  name = "detect aruco markers")


    roboarm_service = Consumer(arm_ik.step,
                               input_busses = perception_bus,
                               delay=0.1,
                               termination_busses = termination_bus,
                               name = 'control robo arm')
    list_of_concurrent_services  = [perception_service,
                                    roboarm_service]

    
    runConcurrently(list_of_concurrent_services)
