import sys,os
root = os.path.dirname(os.path.realpath(__file__))
root = os.path.abspath(os.path.join(root,os.pardir))
sys.path.append(root)
from tkinter import Y
import cv2
import time
#import Camera
import threading
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *


class MoveArm():

    def __init__(self) -> None:
        
        self.gripperAngle_closed = 265
        self.gripperAngle_open = 0     
        self.last_target_world_X = None
        self.last_target_world_Y = None
        self.detected_color = None 
        self.AK = ArmIK()
        #放置坐标
        self.coordinate = {
            'red':   (-15 + 0.5, 12 - 0.5, 1.5),
            'green': (-15 + 0.5, 6 - 0.5,  1.5),
            'blue':  (-15 + 0.5, 0 - 0.5,  1.5),
        }
    # 初始位置
    def initMove(self):
        """
        move end effector to home position

        """
        Board.setBusServoPulse(1, self.gripperAngle_closed - 50, 300)
        Board.setBusServoPulse(2, 500, 500)
        self.AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)

    def capture_block_location_and_color(self, X_Coord, Y_Coord, Color):

        self.last_target_world_X = X_Coord
        self.last_target_world_Y = Y_Coord
        self.detected_color = Color


    def set_Arm_RGB_Color(self, detected_color):

        if detected_color == "red":
            Board.RGB.setPixelColor(0, Board.PixelColor(255, 0, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(255, 0, 0))
            Board.RGB.show()
        elif detected_color == "green":
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 255, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 255, 0))
            Board.RGB.show()
        elif detected_color == "blue":
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 255))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 255))
            Board.RGB.show()
        else:
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 0))
            Board.RGB.show()
    
    def match_endEffector_angle_to_block(self,target_world_X, target_world_Y, target_orientation_angle):

        servo2_angle = getAngle(target_world_X, target_world_Y, target_orientation_angle)
        
        return servo2_angle

    def rotate_endEffector_by_angle(self, targetAngle):
        
        Board.setBusServoPulse(2, targetAngle, 500)

    def move_endEffector_to_target(self, target_world_X, target_world_Y):
        
        status = self.AK.setPitchRangeMoving((target_world_X, target_world_Y, 7), -90, -90, 0)
        
        if status != False:

            self.last_target_world_X = target_world_X
            self.last_target_world_Y = target_world_Y

        return status  

    def closeGripper(self):

        Board.setBusServoPulse(1, self.gripperAngle_open + self.gripperAngle_closed, 500)
 
    
    def openGripper(self):

        Board.setBusServoPulse(1, 0, 500)  # 爪子张开 #open gripper 

    
    def adjust_endEffector_Z_height(self, target_Z_height):
        
        if self.last_target_world_Y and self.last_target_world_X is not None: 

            self.AK.setPitchRangeMoving((self.last_target_world_X, self.last_target_world_Y, target_Z_height), 
            
                                    -90, -90, 0, 1000)
   
    def pickUpCan(self):
        #get ready to pick up can 
        targetX = 23
        targetY = 0
        Board.setBusServoPulse(2, 500, 2000)
        self.openGripper()
        self.AK.setPitchRangeMoving( (targetX, targetY, 12), 
        
                                10, 15, 0, 3000)
        targetX = 25
        time.sleep(3)
        self.AK.setPitchRangeMoving( (targetX, targetY, 12), 
        
                                5, 15, 0,1000)
       
        self.closeGripper()

        time.sleep(3)
        #move to pour position
        targetX = 0
        targetY = 26
        self.AK.setPitchRangeMoving( (targetX, targetY, 12), 
        
                                5, 15, 0, 3000) 
        time.sleep(3)
        #pour
        Board.setBusServoPulse(2, 800,1000)
        time.sleep(3)
        #extend and move down
        targetX = 0
        targetY = 34
        self.AK.setPitchRangeMoving( (targetX, targetY, 5), 
        
                                5, 15,0,1000)  
        time.sleep(3)

        #retract and move up
        targetX = 0
        targetY = 24
        self.AK.setPitchRangeMoving( (targetX, targetY, 15), 
        
                                5, 15, 0,1000)  
        time.sleep(3)
if __name__ == '__main__':

    arm_IK = MoveArm()
    arm_IK.initMove()
    while True:
        arm_IK.pickUpCan()
            


    