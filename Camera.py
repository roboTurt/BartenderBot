#!/usr/bin/env python3
# encoding:utf-8
import sys
sys.path.append('/home/pi/ArmPi/')
import cv2
import time
import threading
import numpy as np
from CameraCalibration.CalibrationConfig import *

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

class Camera:

    def __init__(self, resolution=(640, 360)):
        self.cap = None
        self.opened = False
        self.width = 0
        self.height = 0
        self.camera_open()
        _, self.raw_frame = self.cap.read()
        #self.width, self.height = self.raw_frame.shape[:2]
        #self.height = resolution[1]
        self.frame = self.raw_frame
        #加载参数
        #self.camera_calibration = np.load(calibration_param_path + '.npz')
        
        #获取参数
        self.camera_intrinsic_matrix = np.load(calibration_param_path + 'intrinsic_camera_matrix.npy')
        self.distortion_coeffs = np.load(calibration_param_path + 'camera_distortion_coeffs.npy')
        
        self.newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.camera_intrinsic_matrix, self.distortion_coeffs, (self.width, self.height), 0, (self.width, self.height))
        
        self.mapx, self.mapy = cv2.initUndistortRectifyMap(self.camera_intrinsic_matrix, self.distortion_coeffs, None, self.newcameramtx, (self.width,self.height), cv2.CV_32FC1)
        
        #self.th = threading.Thread(target=self.camera_task, args=(), daemon=True)
        #self.th.start()

    def camera_open(self):
        try:
            self.cap = cv2.VideoCapture(0)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y', 'U', 'Y', 'V'))
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            self.cap.set(cv2.CAP_PROP_SATURATION, 40)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 600)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 300)
            self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            self.opened = True
        except Exception as e:
            print('打开摄像头失败:', e)

    def camera_close(self):
        try:
            self.opened = False
            time.sleep(0.2)
            if self.cap is not None:
                self.cap.release()
                time.sleep(0.05)
            self.cap = None
        except Exception as e:
            print('关闭摄像头失败:', e)

    def get_camera_frame(self):
        #print("frame")
        return self.raw_frame

    def get_undistorted_frame(self):

        try:
            if self.opened and self.cap.isOpened():
                ret, frame_tmp = self.cap.read()
                if ret:
                    frame_resize = cv2.resize(frame_tmp, (self.width, self.height), interpolation=cv2.INTER_NEAREST)
                    undistorted_frame = cv2.remap(frame_resize, self.mapx, self.mapy, cv2.INTER_LINEAR)
                    return undistorted_frame
                else:
                    
                    return None 
            else:
                return None 
        
        except Exception as e:
            print('error in reading camera frame', e)
            time.sleep(0.01)

    def camera_task(self):
        while True:
            try:
                if self.opened and self.cap.isOpened():
                    ret, frame_tmp = self.cap.read()
                    if ret:
                        frame_resize = cv2.resize(frame_tmp, (self.width, self.height), interpolation=cv2.INTER_NEAREST)
                        self.frame = cv2.remap(frame_resize, self.mapx, self.mapy, cv2.INTER_LINEAR)
                    else:
                        print(1)
                        self.frame = None
                        cap = cv2.VideoCapture(-1)
                        ret, _ = cap.read()
                        if ret:
                            self.cap = cap
                elif self.opened:
                    print(2)
                    cap = cv2.VideoCapture(-1)
                    ret, _ = cap.read()
                    if ret:
                        self.cap = cap              
                else:
                    time.sleep(0.01)
            except Exception as e:
                print('获取摄像头画面出错:', e)
                time.sleep(0.01)

if __name__ == '__main__':
    my_camera = Camera()
    my_camera.camera_open()
    while True:
        img = my_camera.frame
        if img is not None:
            cv2.imshow('img', img)
            key = cv2.waitKey(1)
            if key == 27:
                break
    my_camera.camera_close()
    cv2.destroyAllWindows()
