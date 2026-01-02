# planner.py
import threading
import time
import numpy as np
from kinematics import camera_point_to_base, ik_position_only
from control import robot_state, state_lock
from hardware import stop_event

PLANNER_HZ = 2  # update rate (Hz)

def planner_loop(get_camera_point):
    """
    Runs in separate thread. get_camera_point() → 3-element np.array in camera frame
    """
    while not stop_event.is_set():
        with state_lock:
            q_current = robot_state["joint_angles"].copy()

        P_camera = get_camera_point()
        if P_camera is not None:
            target_pos = camera_point_to_base(P_camera, q_current)
            q_sol, success = ik_position_only(target_pos, q_current)
            if success:
                with state_lock:
                    robot_state["target_q"] = q_sol

        time.sleep(1.0/PLANNER_HZ)
