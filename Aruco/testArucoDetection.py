import sys,os
root = os.path.dirname(os.getcwd())
sys.path.append(root)
import cv2
import time
import Camera
import threading
import numpy as np

#load camera calibration parameters 

camera_mtx_file_path = root+"/CameraCalibration/Fang_Camera_Calibration_Params/fang_intrinsic_camera_matrix.npy"

distortion_coeffs_file_path = root+"/CameraCalibration/Fang_Camera_Calibration_Params/fang_camera_distortion_coeffs.npy"


camera_intrinsic_matrix = np.load(camera_mtx_file_path)
distortion_coeffs = np.load(distortion_coeffs_file_path)

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
    corners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters, 
                                                                           cameraMatrix = camera_intrinsic_matrix, 
                                                                           distCoeff = distortion_coeffs)

    # Check that at least one ArUco marker was detected
    if len(corners) > 0:

        for i in range(0, len(markerIds)):
            # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, camera_intrinsic_matrix,
                                                                       distortion_coeffs)
            # Draw a square around the markers
            cv2.aruco.drawDetectedMarkers(frame, corners) 

            # Draw Axis
            cv2.aruco.drawAxis(frame, camera_intrinsic_matrix, distortion_coeffs, rvec, tvec, 0.01)

    return frame 
    #cv2.aruco.drawDetectedMarkers(img, markerCorners) 
   # print(markerCorners)

# ARUCO_DICT = {
#   "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
#   "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
#   "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
#   "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
#   "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
#   "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
#   "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
#   "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
#   "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
#   "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
#   "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
#   "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
#   "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
#   "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
#   "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
#   "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
#   "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
# }




if __name__ == '__main__':

    my_camera = Camera.Camera()
    my_camera.camera_open()
    
    while True:
        
        img = my_camera.frame
        if img is not None:
            frame = img.copy()
            undistorted_frame = undistortFrame(frame)
            frame = arucoDetect(undistorted_frame)
            cv2.imshow('Frame', frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
    
    my_camera.camera_close()
    cv2.destroyAllWindows()