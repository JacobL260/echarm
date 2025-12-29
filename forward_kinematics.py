import numpy as np
import math

# DH parameters: [a, alpha, d, theta]
# theta will be replaced by joint variables
# a is link length (x direction)
# alpha is link twist (x-axis rotation)
# d is link offset (z direction)
# theta is joint angle (rotation about z)
DH_PARAMS = [
    [0, math.radians(-90), 0, 100],  # Joint 1
    [40, 0, 0, 0],    # Joint 2
    [100, 0, 0, 0],
]

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

# Example joint angles
q = [0, math.radians(90), math.radians(90)]
T_ee = np.round(forward_kinematics(q), 3)  # rounded for readability

print("End-effector pose:\n", T_ee)
