import math

# Self explaniroty
NUM_AXES = 6

# Pot configurations
VREF = 3.3
POT_MAX_DEG = 270

# Freqency of different threads and processes
ADC_HZ = 200
CTRL_HZ = 50
FK_HZ = 10
MAIN_STATUS_HZ = 5
BUTTON_HZ = 40

# Need to work on, essentially correction in calibration of the mechanical systems
POT_ZERO_OFFSETS = [0, 0, 0, 0, 0, 0]

# Indiviudal actuators in the robot settings
ACT_SOFT_LIMITS = [
    {"min": -60, "max": 60},
] * NUM_AXES
ACT_TO_POT_RATIO = [-1, -1, -1, -39/27, -1, -39/27]
ACT_TO_MOTOR_RATIO = [1, -39, -39, -23, -39, -23]
PID_PARAMS = [
    {"kp": 0.6, "ki": 0.05, "kd": 0.0},
] * NUM_AXES

# DH parameters: [a, alpha, d, theta]
# theta will be replaced by joint variables
# a is link length (x direction)
# alpha is link twist (x-axis rotation)
# d is link offset (z direction)
# theta is joint angle (rotation about z)
DH_PARAMS = [
    [0, math.radians(90), 15, 0],
    [10, 0, 0, 0],
    [0, math.radians(90), 0, 0],
    [0, math.radians(-90), 12, 0],
    [0, math.radians(90), 0, 0],
    [0, 0, 10, 0]
]

# ADS1115 I2C addresses
ADS_ADDRESSES = [0x48, 0x49]

# Channel mapping (ADS index, channel enum name)
ADC_CHANNEL_MAP = [
    (0, "P0"),
    (0, "P1"),
    (0, "P2"),
    (1, "P0"),
    (1, "P1"),
    (1, "P2"),
]

# TB6600 STEP / DIR pin mapping per axis
# Format: (STEP_PIN, DIR_PIN)
STEPPER_PINS = [
    (1, 2),
    (3, 4),
    (5, 6),
    (7, 8),
    (9, 10),
    (11, 12),
]

BUTTON_PINS = [
    13,
    14
]