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

#load camera calibration parameters 

camera_mtx_file_path = root+"/CameraCalibration/Camera_Calibration_Params/intrinsic_camera_matrix.npy"

distortion_coeffs_file_path = root+"/CameraCalibration/Camera_Calibration_Params/camera_distortion_coeffs.npy"


camera_intrinsic_matrix = np.load(camera_mtx_file_path)
distortion_coeffs = np.load(distortion_coeffs_file_path)

MARKER_WIDTH = 0.04275 #units in meters
ROBOT_BASE_MARKER_ID = 2 
CUP_MARKER_ID = 4

def undistortFrame(frame_image):

    h,  w = frame_image.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(camera_intrinsic_matrix, distortion_coeffs, (w,h), 1, (w,h))

    dst = cv2.undistort(frame_image, camera_intrinsic_matrix, distortion_coeffs, None, newcameramtx)
    # crop the image
    #x, y, w, h = roi
    #dst = dst[y:y+h, x:x+w]
    
    return dst
    #cv2.imwrite('calibresult.png', dst)


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
    # base_marker_idx = int(np.where(markerIDs == ROBOT_BASE_MARKER_ID)[0][0])
    # cup_marker_idx = int(np.where(markerIDs == CUP_MARKER_ID)[0][0])

    # print(base_marker_idx, cup_marker_idx)
    # Check that at least one ArUco marker was detected

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

            # Calculate rotation matrix of cup from base.
            # If we don't need it, comment it out.
            cup_R,_ = cv2.Rodrigues(cup_rvec)
            base_R, _  = cv2.Rodrigues(base_rvec)
            cup_R = np.matrix(cup_R)
            base_R = np.matrix(base_R)
            rel_R = R.from_matrix(np.linalg.inv(base_R)*cup_R)
            print(f"cup euler ZYZ from base: {rel_R.as_euler('zyz', degrees=True)}")

            cup2base_distance = cup_tvec - base_tvec
            print(f"cup distance from base: {cup2base_distance}")
            #return cup2base_distance[0][0]

        except IndexError:

            print("no cup detected")
            cup2base_distance = 0
    else:
        print("No arm detected")
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

    return frame, cup2base_distance


if __name__ == '__main__':

    my_camera = Camera.Camera()
    my_camera.camera_open()
    
    while True:
        
        img = my_camera.frame
        if img is not None:
            frame = img.copy()
            undistorted_frame = undistortFrame(frame)
            frame, _ = arucoDetect(undistorted_frame)
            cv2.imshow('Frame', frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
    
    my_camera.camera_close()
    cv2.destroyAllWindows()