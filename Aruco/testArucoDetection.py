import sys,os
root = os.path.dirname(os.path.realpath(__file__))
root = os.path.abspath(os.path.join(root,os.pardir))
print(root)
sys.path.append(root)
import cv2
import time
import Camera
import threading
import numpy as np
from scipy.spatial.transform import Rotation as R
from CameraCalibration.CalibrationConfig import *

#load camera calibration parameters 

#camera_mtx_file_path = root+"/CameraCalibration/Camera_Calibration_Params/intrinsic_camera_matrix.npy"

#distortion_coeffs_file_path = root+"/CameraCalibration/Camera_Calibration_Params/camera_distortion_coeffs.npy"

# camera_intrinsic_matrix = np.load(camera_mtx_file_path)
# distortion_coeffs = np.load(distortion_coeffs_file_path)
try:
    camera_intrinsic_matrix = np.load(calibration_param_path + 'intrinsic_camera_matrix.npy')
    distortion_coeffs = np.load(calibration_param_path + 'camera_distortion_coeffs.npy')

except AttributeError:

    print("camera calibration parameters not found")



def arucoDetect(frame):

    #Load the dictionary that was used to generate the markers.
    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

    # Initialize the detector parameters using default values
    parameters =  cv2.aruco.DetectorParameters_create()

    # Detect the markers in the image
    corners, markerIDs, rejectedCandidates = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters, 
                                                                           cameraMatrix = camera_intrinsic_matrix, 
                                                                           distCoeff = distortion_coeffs)
    #markerIDs get sorted in decreasing order 
    #grab array indices 

    # Check that at least one ArUco marker was detected
    detection_successful = False 

    if (markerIDs is not None) and (ROBOT_BASE_MARKER_ID and CUP_MARKER_ID in markerIDs):
        
        try:
            base_marker_idx = int(np.where(markerIDs == ROBOT_BASE_MARKER_ID)[0][0])
            cup_marker_idx = int(np.where(markerIDs == CUP_MARKER_ID)[0][0])

            print(base_marker_idx, cup_marker_idx)

            base_rvec, base_tvec, base_markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[base_marker_idx], MARKER_WIDTH, camera_intrinsic_matrix,
                                                                        distortion_coeffs)
            cup_rvec, cup_tvec, cup_markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[cup_marker_idx], MARKER_WIDTH, camera_intrinsic_matrix,
                                                                        distortion_coeffs)
            
            print(f"base_tvec: {base_tvec}")
            print(" ")
            print(f"cup_tvec: {cup_tvec}")

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
            
    if len(corners) > 0:

        for i in range(0, len(markerIDs)):
            # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
            
            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], MARKER_WIDTH, camera_intrinsic_matrix,
                                                                       distortion_coeffs)
            
            #print(f"marker ID {markerIDs[i]} translation vector : {tvec}")
            # Draw a square around the markers
            cv2.aruco.drawDetectedMarkers(frame, corners) 

            # Draw Axis
            cv2.aruco.drawAxis(frame, camera_intrinsic_matrix, distortion_coeffs, rvec, tvec, 0.01)

    return frame, detection_successful, cup2base_distance


if __name__ == '__main__':

    my_camera = Camera.Camera()
    #my_camera.camera_open()
    
    while True:
        
        img = my_camera.get_undistorted_frame()
        if img is not None:
            undistorted_frame = my_camera.get_undistorted_frame()
            processed_frame,_,_ = arucoDetect(undistorted_frame)
            cv2.imshow('Frame', processed_frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
    
    my_camera.camera_close()
    cv2.destroyAllWindows()