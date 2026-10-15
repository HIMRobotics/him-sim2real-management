"""Shared hardware and SDK constants for the 29-joint Unitree G1."""

import math

# GR00T nominal base-height command and provisional standing safety limits.
G1_STANDING_PELVIS_HEIGHT_M = 0.74
STANDING_HEIGHT_MIN_FRACTION = 0.66
STANDING_HEIGHT_MIN_M = G1_STANDING_PELVIS_HEIGHT_M * STANDING_HEIGHT_MIN_FRACTION
STANDING_UPRIGHT_MIN = math.cos(math.radians(30.0))

# SDK motor indices, including waist slots even when mechanically locked.
G1_29_JOINT_NAMES = (
    "left_hip_pitch",
    "left_hip_roll",
    "left_hip_yaw",
    "left_knee",
    "left_ankle_pitch",
    "left_ankle_roll",
    "right_hip_pitch",
    "right_hip_roll",
    "right_hip_yaw",
    "right_knee",
    "right_ankle_pitch",
    "right_ankle_roll",
    "waist_yaw",
    "waist_roll",
    "waist_pitch",
    "left_shoulder_pitch",
    "left_shoulder_roll",
    "left_shoulder_yaw",
    "left_elbow",
    "left_wrist_roll",
    "left_wrist_pitch",
    "left_wrist_yaw",
    "right_shoulder_pitch",
    "right_shoulder_roll",
    "right_shoulder_yaw",
    "right_elbow",
    "right_wrist_roll",
    "right_wrist_pitch",
    "right_wrist_yaw",
)

NUM_JOINTS = len(G1_29_JOINT_NAMES)
LOWER_BODY_INDICES = tuple(range(15))  # Legs and waist.
ARM_INDICES = tuple(range(15, NUM_JOINTS))
UPPER_BODY_INDICES = tuple(range(12, NUM_JOINTS))  # Waist and arms.
LOWER_JOINT_NAMES = tuple(G1_29_JOINT_NAMES[i] for i in LOWER_BODY_INDICES)
ARM_WEIGHT_INDEX = 29  # SDK ownership slot, not a physical joint.

# Named SDK indices used by the arm trajectories.
LEFT_SHOULDER_ROLL_INDEX = G1_29_JOINT_NAMES.index("left_shoulder_roll")
RIGHT_SHOULDER_ROLL_INDEX = G1_29_JOINT_NAMES.index("right_shoulder_roll")
LEFT_ELBOW_INDEX = G1_29_JOINT_NAMES.index("left_elbow")
RIGHT_ELBOW_INDEX = G1_29_JOINT_NAMES.index("right_elbow")
G1_MACHINE_TYPE = 5

# Wireless remote packet layout. Bits match unitree_sdk2py.utils.joystick.
START_BUTTON_BYTE = 2
START_BUTTON_MASK = 1 << 2
F3_BUTTON_MASK = 1 << 7
DPAD_BUTTON_BYTE = 3
LEFT_BUTTON_MASK = 1 << 7
B_BUTTON_MASK = 1 << 1
WIRELESS_REMOTE_SIZE = 40

# Unitree SDK topics and protocol identifiers.
LOW_STATE_TOPIC = "rt/lowstate"
LOW_COMMAND_TOPIC = "rt/lowcmd"
ARM_SDK_TOPIC = "rt/arm_sdk"
DAMPING_FSM_ID = 1
SDK_RPC_TIMEOUT_CODE = 3104  # Unitree RPC_ERR_CLIENT_API_TIMEOUT.
SDK_RPC_SEND_CODE = 3102  # Unitree RPC_ERR_CLIENT_SEND.
MOTION_SWITCHER_SET_SILENT = 1004
ENGLISH_SPEAKER_ID = 1
