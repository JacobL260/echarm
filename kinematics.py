# kinematics.py
import numpy as np
from scipy.spatial.transform import Rotation as R
from config import DH_PARAMS

# ============================================================
# FORWARD KINEMATICS
# ============================================================

def dh_transform(a, alpha, d, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0,              np.sin(alpha),               np.cos(alpha),               d],
        [0,              0,                           0,                           1]
    ])

def forward_kinematics(q):
    T = np.eye(4)
    for i, (a, alpha, d, _) in enumerate(DH_PARAMS):
        T = T @ dh_transform(a, alpha, d, q[i])
    return T

def fk_position(q):
    return forward_kinematics(q)[:3,3]

def fk_pose(q):
    T = forward_kinematics(q)
    return T[:3,3], T[:3,:3]

# ============================================================
# CAMERA (eye-in-hand) TRANSFORM
# ============================================================

R_ee_camera = R.from_euler('xyz',[0,0,0]).as_matrix()
t_ee_camera = np.array([0.05, 0.0, 0.10])
T_ee_camera = np.eye(4)
T_ee_camera[:3,:3] = R_ee_camera
T_ee_camera[:3,3] = t_ee_camera

def base_to_camera(q):
    T_base_ee = forward_kinematics(q)
    return T_base_ee @ T_ee_camera

def camera_point_to_base(P_camera, q):
    P_h = np.append(P_camera, 1.0)
    return (base_to_camera(q) @ P_h)[:3]

# ============================================================
# NUMERICAL JACOBIANS
# ============================================================

def numerical_jacobian_position(q, eps=1e-6):
    q = np.array(q, dtype=float)
    J = np.zeros((3,len(q)))
    p0 = fk_position(q)
    for i in range(len(q)):
        dq = q.copy()
        dq[i] += eps
        J[:,i] = (fk_position(dq)-p0)/eps
    return J

def rotation_error(R_d,R_curr):
    R_err = R_d @ R_curr.T
    return 0.5*np.array([R_err[2,1]-R_err[1,2],
                         R_err[0,2]-R_err[2,0],
                         R_err[1,0]-R_err[0,1]])

def numerical_jacobian_full(q, eps=1e-6):
    q = np.array(q, dtype=float)
    J = np.zeros((6,len(q)))
    p0,R0 = fk_pose(q)
    for i in range(len(q)):
        dq = q.copy()
        dq[i] += eps
        pi,Ri = fk_pose(dq)
        J[:3,i] = (pi-p0)/eps
        dR = Ri @ R0.T
        J[3:,i] = 0.5*np.array([dR[2,1]-dR[1,2],
                                dR[0,2]-dR[2,0],
                                dR[1,0]-dR[0,1]])/eps
    return J

# ============================================================
# INVERSE KINEMATICS
# ============================================================

def ik_position_only(target_pos, q_init, max_iters=1000, tol=1e-4, damping=1e-2):
    q = np.array(q_init, dtype=float)
    for _ in range(max_iters):
        p = fk_position(q)
        error = target_pos - p
        if np.linalg.norm(error)<tol:
            return q, True
        J = numerical_jacobian_position(q)
        JT = J.T
        dq = JT @ np.linalg.inv(J @ JT + damping**2*np.eye(3)) @ error
        q += dq
    return q, False

def ik_full_pose(target_T, q_init, max_iters=1000, tol=1e-4, damping=1e-2):
    q = np.array(q_init, dtype=float)
    p_d = target_T[:3,3]
    R_d = target_T[:3,:3]

    for _ in range(max_iters):
        p,R_curr = fk_pose(q)
        pos_err = p_d - p
        rot_err = rotation_error(R_d,R_curr)
        error = np.hstack((pos_err, rot_err))
        if np.linalg.norm(error)<tol:
            return q, True
        J = numerical_jacobian_full(q)
        JT = J.T
        dq = JT @ np.linalg.inv(J @ JT + damping**2*np.eye(6)) @ error
        q += dq
    return q, False
