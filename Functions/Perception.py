import sys,os
root = os.path.dirname(os.path.realpath(__file__))
root = os.path.abspath(os.path.join(root,os.pardir))
sys.path.append(root)
import cv2
import time
import Camera
import threading
import numpy as np
from scipy.spatial.transform import Rotation as R
from CameraCalibration.CalibrationConfig import *
#from RossROS import rossros 


# import cv2
# import math
# from CameraCalibration import *
# from LABConfig import *
# from ArmIK.Transform import *
# import HiwonderSDK.Board as Board
# import numpy as np
# import time 
# from collections import deque

# from RossROS import rossros 

class Perception():

    def __init__(self):
    
        self.camera = Camera.Camera() 
        self.markers = {}
        self.marker_counts = 0

    def aruco_detect(self):
        frame = self.camera.get_undistorted_frame()

        #Load the dictionary that was used to generate the markers.
        dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

        # Initialize the detector parameters using default values
        parameters = cv2.aruco.DetectorParameters_create()

        # Detect the markers in the image
        corners, markerIDs, rejectedCandidates = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters, 
                                                                            cameraMatrix = self.camera.camera_intrinsic_matrix, 
                                                                            distCoeff = self.camera.distortion_coeffs)
        cv2.aruco.drawDetectedMarkers(frame, corners) 
        #markerIDs get sorted in decreasing order 
        #grab array indices 

        # Check that at least one ArUco marker was detected
        markers = {}

        conf = self.camera.camera_intrinsic_matrix, self.camera.distortion_coeffs
        if markerIDs is not None:
            # WARNING: Assumes no duplicate markers
            for i, marker_id in enumerate(markerIDs):
                rvec, tvec, points = cv2.aruco.estimatePoseSingleMarkers(corners[i], MARKER_WIDTH, *conf)
                R, _ = cv2.Rodrigues(rvec)
                R = np.matrix(R)
                tvec = np.array(tvec).flatten()

                # Draw Axis
                cv2.aruco.drawAxis(frame, *conf, rvec, tvec, 0.01)

                if marker_id == ROBOT_BASE_MARKER_ID:
                    key = 'base'
                elif marker_id == CUP_MARKER_ID:
                    key = 'cup'
                elif marker_id == CAN_MARKER_ID:
                    key = 'can'
                else:
                    continue

                #print(f"detected a {key} with pose {tvec}")
                markers[key] = R, tvec
                if 'base' in markers and 'can' in markers:
                    #print("offset between base and can:")
                    #print(markers['base'][1] - markers['can'][1])
                    pass

            # Run observations through naive filter to get rid of any noise
            alpha = 0.6
            for k in markers:
                if k not in self.markers:
                    self.markers[k] = markers[k]
                else:
                    self.markers[k] = [alpha * x + (1-alpha) * y for (x,y) in zip(self.markers[k], markers[k])]
        self.frame = frame
        return self.markers

if __name__ == '__main__':
    aruco_perception = Perception() # 

    while True:
        markers = aruco_perception.aruco_detect()
        cv2.imshow('Frame', aruco_perception.frame)
        key = cv2.waitKey(1)
        if key == 27:
            break
    
