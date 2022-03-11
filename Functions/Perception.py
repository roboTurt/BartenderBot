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
        detection_successful = False 
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

        """
        if (markerIDs is not None) and (ROBOT_BASE_MARKER_ID and CUP_MARKER_ID in markerIDs):
            
            try:
                base_marker_idx = int(np.where(markerIDs == ROBOT_BASE_MARKER_ID)[0][0])
                cup_marker_idx = int(np.where(markerIDs == CUP_MARKER_ID)[0][0])

                print(base_marker_idx, cup_marker_idx)

                base_rvec, base_tvec, base_markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[base_marker_idx], MARKER_WIDTH, self.camera.camera_intrinsic_matrix,
                                                                            self.camera.distortion_coeffs)
                cup_rvec, cup_tvec, cup_markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[cup_marker_idx], MARKER_WIDTH, self.camera.camera_intrinsic_matrix,
                                                                            self.camera.distortion_coeffs)
                
                print(f"base_tvec: {base_tvec}")
                print(" ")
                print(f"cup_tvec: {cup_tvec}")

                # Calculate rotation matrix of cup from base.
                # If we don't need it, comment it out.
                cup_R,_ = cv2.Rodrigues(cup_rvec)
                base_R, _  = cv2.Rodrigues(base_rvec)
                cup_R = np.matrix(cup_R)
                base_R = np.matrix(base_R)
                rel_R = R.from_matrix(np.linalg.inv(base_R)*cup_R)
                print(f"cup euler ZYZ from base: {rel_R.as_euler('zyz', degrees=True)}")

                cup2base_distance = cup_tvec - base_tvec
                detection_successful = True 
                print(f"cup distance from base: {cup2base_distance}")

            except IndexError:

                print("no cup detected")
                cup2base_distance = 0
        else:
            print("No markers detected")
            cup2base_distance = 0
                
        """
        self.frame = frame
        return markers

if __name__ == '__main__':
    aruco_perception = Perception() # 

    while True:
        markers = aruco_perception.aruco_detect()
        cv2.imshow('Frame', aruco_perception.frame)
        key = cv2.waitKey(1)
        if key == 27:
            break
    
