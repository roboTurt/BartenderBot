

from tkinter import Y
import cv2
import time
import Camera
import threading
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *


class MoveArm():

    def __init__(self) -> None:
        
        self.gripperAngle_closed = 500    
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

        Board.setBusServoPulse(1, self.gripperAngle_closed, 500)
 
    
    def openGripper(self):

        Board.setBusServoPulse(1, self.gripperAngle_closed - 280, 500)  # 爪子张开 #open gripper 

    
    def adjust_endEffector_Z_height(self, target_Z_height):
        
        if self.last_target_world_Y and self.last_target_world_X is not None: 

            self.AK.setPitchRangeMoving((self.last_target_world_X, self.last_target_world_Y, target_Z_height), 
            
                                    -90, -90, 0, 1000)
   
    def pickAndPlace(self, targetX, targetY, orientationAngle, color):
        
        #print(len(processedImage))

        # targetX = processedImage[2]
        # targetY = processedImage[3]
        # orientationAngle = processedImage[4]
        # color = processedImage[1]
        
        if color is "None":
            
            self.initMove()
        
        # print(orientationAngle)
        # print(color)

        else:

            previous_target_coords = [self.last_target_world_X, self.last_target_world_Y]
            current_target_coords = [targetX, targetY]

            # if None not in previous_target_coords:
            #     previous_target_coords = [round(num, 1) for num in previous_target_coords]
            if None not in current_target_coords:
                current_target_coords = [round(num, 1) for num in current_target_coords]

                print(targetX)
                print(" ")
                print(targetY)
                self.capture_block_location_and_color(targetX, targetY, color)
                self.openGripper()
                
                result = self.move_endEffector_to_target(targetX, targetY)
                #print(result)
                if result is not False:

                    target_orient_angle = self.match_endEffector_angle_to_block(targetX, targetY, orientationAngle)
                    time.sleep(0.2)
                    self.rotate_endEffector_by_angle(target_orient_angle)
                    time.sleep(0.2)
                    self.adjust_endEffector_Z_height(1.5) #lower claw to block 
                    time.sleep(0.2)
                    self.closeGripper()
                    time.sleep(0.2)
                    self.adjust_endEffector_Z_height(12) #raise claw 
                    time.sleep(0.2)
                    target_home_position_X = self.coordinate[color][0]
                    target_home_position_Y = self.coordinate[color][1]
                    self.move_endEffector_to_target(target_home_position_X, target_home_position_Y)
                    time.sleep(2.2)
                    self.adjust_endEffector_Z_height(1.5) #lower claw to block 
                    time.sleep(0.2)
                    self.openGripper()
                    time.sleep(0.2)
            
                else:
                    self.initMove()
                    time.sleep(5)

                    #time.sleep(2)

        

            


    