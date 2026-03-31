"""DIR demo program"""
__author__ = "Matej Hodnik"
__version__ = "28.03.26"
__email__ = "mh2078@student.uni-lj.si"


########################################################################################################################
# Import statements

from yrc_high_speed_ethernet import ClientOfYRC, UDPRequest, UDPAnswer
from time import sleep
import cv2
import numpy as np
import sys
import os
import pprint 

# Add Camera directory to path for apriltag import
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../Camera'))
import apriltag

########################################################################################################################
# Initialisation

# TODO: User Configuration Parameters
local = False

# Open UDP connection
if local:
    yrc = ClientOfYRC('localhost')
    print('Connection with virtual controller established')
else:
    yrc = ClientOfYRC('172.16.0.1')
    print('Connection with YRC1000 established')

#####################################CONSTANTS##########################################################################
ROT_PREE = 10000 #0.0001 degres -> 1deg
TRAN_PREE = 1000 # 0.001 mm -> 1mm

#################USER FUNCTIONS##########################################################################################
def wait_move_finnish():
    status = yrc.status_information_reading()
    while(status['Running'] != 0):
        print("Waiting for movement to finish...", end="\r")
        sleep(0.5)
        status = yrc.status_information_reading()

#
#   DIR26CILOPEN
#   DIR26CILCLOSE
#

def gripper_grab():
    ok_select = yrc.job_select("DIR26CILCLOSE", 0)
    print("Select DIR26CILCLOSE:", ok_select)

    if ok_select:
        ok_start = yrc.job_start()
        print("Start DIR26CILCLOSE:", ok_start)
    else:
        print("Failed to select DIR26CILCLOSE job")

def gripper_release():
    ok_select = yrc.job_select("DIR26CILOPEN", 0)
    print("Select DIR26CILOPEN:", ok_select)

    if ok_select:
        ok_start = yrc.job_start()
        print("Start DIR26CILOPEN:", ok_start)
    else:
        print("Failed to select DIR26CILOPEN job")

def capture_photo_with_transform(camera_id=0, tag_size=0.02375):
    """
    Capture a photo from camera and return homogeneous transform matrix.
    
    Args:
        camera_id (int): Camera device ID (default: 0)
        tag_size (float): Physical AprilTag size in meters (default: 0.02375)
    
    Returns:
        dict: Contains:
            - 'pose_matrix': 4x4 homogeneous transformation matrix
            - 'translation': [x, y, z] in camera frame
            - 'rotation': 3x3 rotation matrix
            - 'rpy_deg': [roll, pitch, yaw] in degrees
            - 'tag_id': Detected AprilTag ID
            - 'frame': Captured frame image
        Returns None if no tag detected
    """
    
    # Latest calibration parameters (from line 5 of camtest.py)
    camera_params = (1439.4915254739828, 1441.9589818431248, 
                     950.2746826769213, 550.7749059489563)
    fx, fy, cx, cy = camera_params
    
    # Camera matrix
    K = np.array([[fx, 0.0, cx],
                  [0.0, fy, cy],
                  [0.0, 0.0, 1.0]], dtype=np.float64)
    
    # Open camera
    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        print(f"Error: Cannot open camera {camera_id}")
        return None
    
    # Set detector options
    detector_options = apriltag.DetectorOptions(
        families='tag36h11',
        nthreads=1,
        quad_decimate=1.0,
        quad_blur=0.0,
        refine_edges=True,
    )
    
    try:
        detector = apriltag.Detector(detector_options,
                                     searchpath=apriltag._get_demo_searchpath())
    except RuntimeError as exc:
        print(f"Error: Could not create detector - {exc}")
        cap.release()
        return None
    
    # Capture single frame
    success, frame = cap.read()
    cap.release()
    
    if not success:
        print("Error: Failed to capture frame")
        return None
    
    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Detect AprilTags
    detections, dimg = detector.detect(gray, return_image=True)
    
    if len(detections) == 0:
        print("No AprilTag detected in frame")
        return None
    
    # Get first detection
    detection = detections[0]
    
    # Estimate pose
    pose, e0, e1 = detector.detection_pose(detection, camera_params, tag_size)
    
    # Extract components from pose matrix
    translation = pose[:3, 3]  # [x, y, z]
    rotation = pose[:3, :3]    # 3x3 rotation matrix
    
    # Convert rotation matrix to Euler angles (roll, pitch, yaw)
    sy = np.sqrt(rotation[0, 0] * rotation[0, 0] + rotation[1, 0] * rotation[1, 0])
    singular = sy < 1e-6
    
    if not singular:
        roll = np.arctan2(rotation[2, 1], rotation[2, 2])
        pitch = np.arctan2(-rotation[2, 0], sy)
        yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
    else:
        roll = np.arctan2(-rotation[1, 2], rotation[1, 1])
        pitch = np.arctan2(-rotation[2, 0], sy)
        yaw = 0.0
    
    rpy_deg = np.degrees(np.array([roll, pitch, yaw]))
    
    # Get tag ID
    tag_id = detection.tag_id
    
    return {
        'pose_matrix':  pose.tolist(),
        'translation':  translation.tolist(),
        'rotation':     rotation.tolist(),
        'rpy_deg':      rpy_deg.tolist(),
        'tag_id':       tag_id,
        'camera_params': camera_params
    }


########################################################################################################################
# Test

#position over table
X = 500*TRAN_PREE #in mm
Y = -150*TRAN_PREE
Z = 530*TRAN_PREE
TX = 180*ROT_PREE #in deg
TY = 1*ROT_PREE
TZ = 60*ROT_PREE


#position up high
X0 = 360*TRAN_PREE
Y0 = -124*TRAN_PREE
Z0 = 693*TRAN_PREE
TX0 = 180*ROT_PREE
TY0 = 0*ROT_PREE
TZ0 = 140*ROT_PREE

# far side motor position
# id=1 t=[-0.477683,-0.243644,0.795450] rpy_deg=[6.030,-1.639,-176.304]


STEP_10CM = 100000   # 100 mm
SPEED = 2000          # 1 = 1.0 mm/s

# print("Initial status:")
# print(yrc.status_information_reading())
# 
# print("\nServo ON:")
# print(yrc.servo_on())
# print("Servos ON")

# sleep(1)

# print("\nMove ")
# print(yrc.move_link(X, Y, Z, TX, TY, TZ, SPEED))
# wait_move_finnish()

# print("\nMove back to start:")
# print(yrc.move_straight(X0, Y0, Z0, TX0, TY0, TZ0, SPEED))
# wait_move_finnish()

# gripper_grab()

# sleep(5)

import json
pose = capture_photo_with_transform()
print(json.dumps(pose)) 

# gripper_release()

# sleep(1)

# print("\nServo OFF:")
# print(yrc.servo_off())
# print("Servos OFF")
