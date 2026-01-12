# python3.13 camera_installed.py
import pyrealsense2 as rs
print("pipeline available?", "pipeline" in dir(rs))
p = rs.pipeline()
print("Pipeline created:", p)