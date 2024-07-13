import pygcm
import cv2
import os

pic_root_path = "/data1/jiagang.chen/DroidGS/slam/submodules/general_camera_model/apps/test_data/"

# Create a camera model
cam0 = pygcm.CameraModel()
cam0.load_config_file(os.path.join(pic_root_path, "penn_c1_cam0.json"))
print(cam0.info())

cam1 = pygcm.CameraModel()
cam1.initialize_params(2, cam0.width() / 2.0, cam0.width(), cam0.height())

mapx, mapy = pygcm.init_undistort_rectify_map(cam0, cam1)

img = cv2.imread(os.path.join(pic_root_path, "2049.png"))
img_undistorted = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)
cv2.imwrite(os.path.join(pic_root_path, "2049_undistorted_py.png"), img_undistorted)