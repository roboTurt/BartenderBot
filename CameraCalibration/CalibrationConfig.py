
import os 
#相邻两个角点间的实际距离，单位cm
corners_length = 2.1

#木块边长3cm
square_length = 3

#标定棋盘大小, 列， 行, 指内角点个数，非棋盘格
calibration_size = (7, 7)

root = os.path.dirname(os.path.realpath(__file__))
root_path = os.path.abspath(os.path.join(root,os.pardir))

calibration_param_path = root_path + '/CameraCalibration/Camera_Calibration_Params/'

# #采集标定图像存储路径
# save_path = calibration_path + '/calibration_images'

# #标定参数存储路径
# calibration_param_path = calibration_path + '/calibration_param'

# #映射参数存储路径
# map_param_path = calibration_path + '/map_param'


MARKER_WIDTH = 0.04275 #units in meters
ROBOT_BASE_MARKER_ID = 2 
CUP_MARKER_ID = 4