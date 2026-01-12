import cv2
from realsense_depth import *

dc = DepthCamera()

cv2.namedWindow("")


ret, depth_frame, color_frame = dc.get_frame()

cv2.imshow("Color frame", color_frame)
cv2.waitKey(0)