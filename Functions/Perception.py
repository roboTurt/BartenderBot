import os, sys 

# BASE_DIR = os.path.dirname(os.getcwd())

sys.path.append(os.path.dirname(os.getcwd()))

#print(os.path.join('../',BASE_DIR))

import cv2
import math
from CameraCalibration import *
from LABConfig import *
from ArmIK.Transform import *
import HiwonderSDK.Board as Board
import numpy as np
import time 
from collections import deque

from RossROS import rossros 

class Perception():

    def __init__(self, targetColor):
        #super().__init__()
        #initialize perception class with a target color to track
        self.targetColor = targetColor
        self.image = None 
        self.image_height = None
        self.image_width = None
        self.resized_image_dimension = (640,480)
        self.area_theshold = 2500
        self.roi_acquired = False 
        self.block_worldX_coord = 0
        self.block_worldY_coord = 0
        self.last_block_worldX_coord = 0
        self.last_block_worldY_coord = 0
        self.count = 0
        self.listOfBlockCenterCoords = []
        self.t1 = 0
        self.t1_Counter_Started = False
        self.range_rgb = {
            'red': (0, 0, 255),
            'blue': (255, 0, 0),
            'green': (0, 255, 0),
            'black': (0, 0, 0),
            'white': (255, 255, 255),
        }

        self.colorList = deque([], maxlen = 10)

        self.start_pick_up = False

        self.rotation_angle = 0

    def setTargetColor(self, targetColor):

        self.targetColor = targetColor 

    def set_board_RGB(self, targetColor):
    
        if targetColor == "red":
            Board.RGB.setPixelColor(0, Board.PixelColor(255, 0, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(255, 0, 0))
            Board.RGB.show()
        elif targetColor == "green":
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 255, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 255, 0))
            Board.RGB.show()
        elif targetColor == "blue":
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 255))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 255))
            Board.RGB.show()
        else:
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 0))
            Board.RGB.show()


    def getAreaMaxContour(self,contours):

        tempMaxContourArea = 0
        maxContourArea = 0
        maxContour = None 

        for c in contours:  # 历遍所有轮廓
            tempMaxContourArea = math.fabs(cv2.contourArea(c))  # 计算轮廓面积
            if tempMaxContourArea > maxContourArea:
                maxContourArea = tempMaxContourArea
                if tempMaxContourArea > 300:  # 只有在面积大于300时，最大面积的轮廓才是有效的，以过滤干扰
                    maxContour = c

        return maxContour, maxContourArea  # 返回最大的轮廓
    
    
    def resizeAndSmoothImage(self):

        frame_resize = cv2.resize(self.image, self.resized_image_dimension, interpolation=cv2.INTER_NEAREST) #resize
        frame_gaussianBlur = cv2.GaussianBlur(frame_resize, (11, 11), 11) #smooth image
        frame_lab = cv2.cvtColor(frame_gaussianBlur, cv2.COLOR_BGR2LAB)  # transform color space 将图像转换到LAB空间 
        #print(frame_lab)
        return frame_lab


    def readImageFrame(self, image):
        #print(image)
        self.image = image
        image_height, image_width = image.shape[:2]
        self.image_height = image_height
        self.image_width = image_width
        cv2.line(self.image, (0, int(image_height / 2)), (image_width, int(image_height / 2)), (0, 0, 200), 1)
        cv2.line(self.image, (int(image_width / 2), 0), (int(image_width / 2), image_height), (0, 0, 200), 1)


    def findObjectContours(self, colorRange, processedImageFrame):
        
        frame_mask = cv2.inRange(processedImageFrame, color_range[colorRange][0], color_range[colorRange][1])  # 对原图像和掩模进行位运算
        opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))  # 开运算
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))  # 闭运算
        contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓
        areaMaxContour, area_max = self.getAreaMaxContour(contours)  # 找出最大轮廓
        
        return areaMaxContour, area_max 

    def convertContourToRegionOfInterest(self, areaMaxContour):
        
        rect = cv2.minAreaRect(areaMaxContour) #use CV2 to draw bounding box around ID'd object
        box = np.int0(cv2.boxPoints(rect)) #get coordinates of bounding box

        roi = getROI(box)
        self.roi_acquired = True 
        
        return rect, box, roi 
    

    def convertCameraFrame2WorldFrame(self,rect,roi,imageDimension,squareLength):

        img_centerX, img_centerY = getCenter(rect, roi, imageDimension, squareLength) #gets center point of detected block

        worldX, worldY = convertCoordinate(img_centerX, img_centerY, imageDimension) #applies transform from camera frame to world frame
        
        return worldX, worldY 

    def drawBox_and_displayCoordinates(self, box, colorRange):
        #print("bish")
        cv2.drawContours(self.image, [box], -1, self.range_rgb[colorRange], 2)
        cv2.putText(self.image, '(' + str(self.block_worldX_coord) + ',' + str(self.block_worldY_coord) + ')', (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.range_rgb[colorRange], 1) #绘制中心点

    def returnDetectedColor(self, averageColor):

        if averageColor == 1:
            detected_color = 'red'
            rgbValue = self.range_rgb["red"]
        elif averageColor == 2:
            detected_color = 'green'
            rgbValue = self.range_rgb["green"]
        elif averageColor == 3:
            detected_color = 'blue'
            rgbValue = self.range_rgb["blue"]
        else:
            detected_color = 'None'
            rgbValue = self.range_rgb["black"]


        return detected_color, rgbValue


    def detectObject(self, processedImageFrame):

        """
        Input:

            The processed image frame

        Output:

            ID location of target object and draw bounding box 

        """

        previousMaxContourArea = 0
        colorOfMaxContour = None
        previousMaxContour = None 
        rgbValue = (0, 0, 0)
        detectedColor = "None"
        #if not self.start_pick_up:
        maxContour = None
        maxContourArea = 0
        #self.image = processedImageFrame
        if processedImageFrame is not 0:
            for i in color_range:

                if i in self.targetColor:
                    targetColor_Range = i
                    #print(processedImageFrame)
                    maxContour, maxContourArea = self.findObjectContours(targetColor_Range, processedImageFrame)
                    #print(maxContour, maxContourArea)
                    if maxContour is not None:

                        if maxContourArea > previousMaxContourArea:

                            previousMaxContourArea = maxContourArea
                            colorOfMaxContour = i 
                            previousMaxContour = maxContour 


            if maxContourArea > self.area_theshold:  # 有找到最大面积
                # rect = cv2.minAreaRect(areaMaxContour)
                # box = np.int0(cv2.boxPoints(rect))

                # roi = getROI(box) #获取roi区域
                # get_roi = True

                boundingRect, boundingRectCoords, regionOfInterest = self.convertContourToRegionOfInterest(maxContour)
                self.block_worldX_coord, self.block_worldY_coord = self.convertCameraFrame2WorldFrame(boundingRect, regionOfInterest, 
                                                                    self.resized_image_dimension, 
                                                                    square_length)


                self.drawBox_and_displayCoordinates(boundingRectCoords,targetColor_Range)

                # print(pow(self.block_worldX_coord - self.last_block_worldX_coord, 2))
                # print(pow(self.block_worldY_coord - self.last_block_worldY_coord, 2))
                distance = math.sqrt(pow(self.block_worldX_coord - self.last_block_worldX_coord, 2) + 
                                    pow(self.block_worldY_coord - self.last_block_worldY_coord,2))
                
                                    #pow(self.block_worldY_coord - self.last_block_worldY_coord, 2)) #对比上次坐标来判断是否移动
                #print(distance)
                
                self.last_block_worldX_coord, self.last_block_worldY_coord = self.block_worldX_coord, self.block_worldY_coord
                

                if colorOfMaxContour == 'red':
                    
                    self.colorList.append(1)
                
                elif colorOfMaxContour == 'green':
                    
                    self.colorList.append(2)
            
                elif colorOfMaxContour == 'blue':
                
                    self.colorList.append(3)

                else:
                
                    self.colorList.append(0)
            
                # if distance < 0.5:
                #     self.count += 1
                #     self.listOfBlockCenterCoords.extend((self.block_worldX_coord, self.block_worldY_coord))
                #     if self.t1_Counter_Started:
                #         self.t1_Counter_Started = False
                #         self.t1 = time.time()
                #     if time.time() - self.t1 > 0.5:
                #         self.rotation_angle = boundingRect[2]
                #         self.t1_Counter_Started = True
                #         self.block_worldX_coord, self.block_worldY_coord = np.mean(np.array(self.listOfBlockCenterCoords).reshape(self.count, 2), axis=0)
                #         self.listOfBlockCenterCoords.clear()
                #         self.count = 0
                #         self.start_pick_up = True

                # else: 
                #     self.t1 = time.time()
                #     self.t1_Counter_Started = True
                #     self.listOfBlockCenterCoords.clear()
                #     self.count = 0
                #track = True
                #print(count,distance)
                # 累计判断
                # if action_finish:
                #     if distance < 0.3:
                #         center_list.extend((world_x, world_y))
                #         count += 1
                #         if t1_Counter_Started:
                #             t1_Counter_Started = False
                #             t1 = time.time()
                #         if time.time() - t1 > 1.5:
                #             rotation_angle = rect[2]
                #             t1_Counter_Started = True
                #             world_X, world_Y = np.mean(np.array(center_list).reshape(count, 2), axis=0)
                #             count = 0
                #             center_list = []
                #             start_pick_up = True
                #     else:
                #         t1 = time.time()
                #         t1_Counter_Started = True
                #         count = 0
                #         center_list = []

                averageColorValue = int(round(np.mean(np.array(self.colorList))))
                
                detectedColor, rgbValue = self.returnDetectedColor(averageColorValue)
            
            else:

                #if not self.start_pick_up:
                rgbValue = (0, 0, 0)
                detectedColor = "None"

            # if move_square:
                
            #     cv2.putText(self.image, "Make sure no blocks in the stacking area", (15, int(self.image.shape[0]/2)), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)    
            
            cv2.putText(self.image, "Color: " + detectedColor, (10, self.image.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, rgbValue, 2)
            #print(self.image)
            # cv2.imshow('Frame', self.image)
            # cv2.waitkey()
            #print([self.image, detectedColor, self.block_worldX_coord, self.block_worldY_coord, self.rotation_angle])
            return [self.image, detectedColor, self.block_worldX_coord, self.block_worldY_coord, self.rotation_angle]
        
        else:

            return [self.image,None,None,None,None]
    

    def drawImageCV2(self,image):
        print(image[1])
        if image[0] is not None:
            cv2.imshow('Frame',image[0])
            cv2.waitKey(1)