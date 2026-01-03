import numpy as np
import threading
import time

from scipy.spatial.transform import Rotation as R
from config import DH_PARAMS, FK_HZ
from hardware import stop_event

# FORWARD KINEMATICS
class FKThread(threading.Thread):
    def __init__(self, robot_state, rate_hz=FK_HZ):
        super().__init__(daemon=True)
        self.robot_state = robot_state
        self.dt = 1.0 / rate_hz
    
    def run(self):
        while not stop_event.is_set():
            # Get actuator feedbacks in order
            actuators = self.robot_state.get("actuators", {})
            if not actuators:
                time.sleep(self.dt)
                continue

            # Ensure order by index
            q_deg = [actuators[i]["feedback_deg"] for i in sorted(actuators.keys())]

            q_rad = np.deg2rad(q_deg)
            T = forward_kinematics(q_rad)

            fk_state = self.robot_state["kinematics"]["fk"]
            fk_state["T_ee"] = T
            fk_state["position"] = T[:3, 3]
            fk_state["rotation"] = T[:3, :3]

            time.sleep(self.dt)

def dh_transform(a, alpha, d, theta):
    """
    Compute the DH transformation matrix for one joint.
    """
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0,              np.sin(alpha),               np.cos(alpha),               d],
        [0,              0,                           0,                           1]
    ])

def forward_kinematics(joint_angles):
    """
    Compute the end-effector pose from joint angles.
    """
    # Start from the base (no wall mount)
    T = np.eye(4)

    # Multiply transformations for each joint
    for i, (a, alpha, d, _) in enumerate(DH_PARAMS):
        T = T @ dh_transform(a, alpha, d, joint_angles[i])

    return T

# INVERSE KINEMATICS - POSITION ONLY
def fk_position(joint_angles):
    """ Compute the end-effector position from joint angles. """
    T = forward_kinematics(joint_angles)
    return T[:3, 3]

def numerical_jacobian_position(q, eps=1e-6):
    """ Compute the numerical Jacobian of the end-effector position. """
    q = np.array(q, dtype=float)
    J = np.zeros((3, len(q)))

    p0 = fk_position(q)

    for i in range(len(q)):
        dq = q.copy()
        dq[i] += eps
        pi = fk_position(dq)
        J[:, i] = (pi - p0) / eps

    return J

def ik_position_only(
    target_pos,
    q_init,
    max_iters=1000,
    tol=1e-4,
    damping=1e-2
):
    """ Inverse kinematics to reach a target position using damped least squares. """
    q = np.array(q_init, dtype=float)

    for _ in range(max_iters):
        p = fk_position(q)
        error = target_pos - p

        if np.linalg.norm(error) < tol:
            return q, True

        J = numerical_jacobian_position(q)

        # Damped least squares
        JT = J.T
        dq = JT @ np.linalg.inv(J @ JT + damping**2 * np.eye(3)) @ error

        q += dq

    return q, False

# INVERSE KINEMATICS - FULL POSE
def rotation_error(R_d, R):
    """ Compute the rotation error vector between desired and current rotation matrices. """
    R_err = R_d @ R.T
    return 0.5 * np.array([
        R_err[2,1] - R_err[1,2],
        R_err[0,2] - R_err[2,0],
        R_err[1,0] - R_err[0,1]
    ])

def fk_pose(q):
    """ Compute the end-effector pose (position + orientation) from joint angles. """
    T = forward_kinematics(q)
    return T[:3, 3], T[:3, :3]

def numerical_jacobian_full(q, eps=1e-6):
    """ Compute the numerical Jacobian of the end-effector pose (position + orientation). """
    q = np.array(q, dtype=float)
    J = np.zeros((6, len(q)))

    p0, R0 = fk_pose(q)

    for i in range(len(q)):
        dq = q.copy()
        dq[i] += eps
        pi, Ri = fk_pose(dq)

        # Position part
        J[:3, i] = (pi - p0) / eps

        # Orientation part
        dR = Ri @ R0.T
        J[3:, i] = 0.5 * np.array([
            dR[2,1] - dR[1,2],
            dR[0,2] - dR[2,0],
            dR[1,0] - dR[0,1]
        ]) / eps

    return J

def ik_full_pose(
    target_T,
    q_init,
    max_iters=1000,
    tol=1e-4,
    damping=1e-2
):
    """ Inverse kinematics to reach a target pose using damped least squares. """
    q = np.array(q_init, dtype=float)
    p_d = target_T[:3, 3]
    R_d = target_T[:3, :3]

    for _ in range(max_iters):
        p, R = fk_pose(q)

        pos_err = p_d - p
        rot_err = rotation_error(R_d, R)

        error = np.hstack((pos_err, rot_err))

        if np.linalg.norm(error) < tol:
            return q, True

        J = numerical_jacobian_full(q)

        JT = J.T
        dq = JT @ np.linalg.inv(J @ JT + damping**2 * np.eye(6)) @ error

        q += dq

    return q, False


if __name__ == "__main__":
    # Target position
    target_pos = np.array([0.3, 0.2, 0.5])  # x, y, z in meters

    # Target orientation in roll-pitch-yaw (in degrees or radians)
    roll, pitch, yaw = np.deg2rad([45, 30, 90])  # convert degrees to radians

    # Convert RPY to rotation matrix
    rot_matrix = R.from_euler('xyz', [roll, pitch, yaw]).as_matrix()

    target_T = np.eye(4)
    target_T[:3, 3] = target_pos      # position
    target_T[:3, :3] = rot_matrix    # orientation

    q_init = [0, 0, 0, 0, 0, 0]  # initial guess

    q_sol, success = ik_full_pose(target_T, q_init)

    if success:
        print("Joint angles for full-pose IK:", q_sol)
    else:
        print("IK solver failed to find a solution")

    q_sol, success = ik_full_pose(target_T, q_init)
