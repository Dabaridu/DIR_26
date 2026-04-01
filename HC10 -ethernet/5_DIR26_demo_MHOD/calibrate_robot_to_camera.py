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
import time
import json
import struct


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
M_TO_MM = 1000.0

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


def _create_detector_with_fallback():
    options = apriltag.DetectorOptions(
        families='tag36h11',
        nthreads=1,
        quad_decimate=1.0,
        quad_blur=0.0,
        refine_edges=True,
    )

    try:
        detector = apriltag.Detector(options, searchpath=apriltag._get_demo_searchpath())
        return detector, 'apriltag_ctypes'
    except RuntimeError as exc:
        if 'could not find apriltag shared library' not in str(exc):
            raise

        try:
            from pupil_apriltags import Detector as PupilDetector
        except ImportError:
            return None, 'missing_backend'

        detector = PupilDetector(
            families='tag36h11',
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=True,
        )
        return detector, 'pupil_apriltags'


def _draw_pose_axes(overlay, camera_params, tag_size, pose):
    fx, fy, cx, cy = camera_params
    K = np.array([[fx, 0.0, cx],
                  [0.0, fy, cy],
                  [0.0, 0.0, 1.0]], dtype=np.float64)

    axis_len = 0.5 * float(tag_size)
    obj = np.array([
        [0.0, 0.0, 0.0],
        [axis_len, 0.0, 0.0],
        [0.0, axis_len, 0.0],
        [0.0, 0.0, -axis_len],
    ], dtype=np.float64)

    rvec, _ = cv2.Rodrigues(np.asarray(pose[:3, :3], dtype=np.float64))
    tvec = np.asarray(pose[:3, 3], dtype=np.float64).reshape(3, 1)

    img_pts, _ = cv2.projectPoints(obj, rvec, tvec, K, None)
    pts = img_pts.reshape(-1, 2).astype(int)

    origin = tuple(pts[0])
    cv2.line(overlay, origin, tuple(pts[1]), (0, 0, 255), 2, cv2.LINE_AA)
    cv2.line(overlay, origin, tuple(pts[2]), (0, 255, 0), 2, cv2.LINE_AA)
    cv2.line(overlay, origin, tuple(pts[3]), (255, 0, 0), 2, cv2.LINE_AA)

    cv2.putText(overlay, 'X', tuple(pts[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
    cv2.putText(overlay, 'Y', tuple(pts[2]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
    cv2.putText(overlay, 'Z', tuple(pts[3]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1, cv2.LINE_AA)

    return overlay


def _rotation_matrix_to_euler_xyz_deg(rotation):
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

    return np.degrees(np.array([roll, pitch, yaw]))


def _build_pose_result(pose, tag_id, camera_params):
    translation = pose[:3, 3] * M_TO_MM
    rotation = pose[:3, :3]
    rpy_deg = _rotation_matrix_to_euler_xyz_deg(rotation)
    return {
        'pose_matrix': pose.tolist(),
        'translation': translation.tolist(),
        'rotation': rotation.tolist(),
        'rpy_deg': rpy_deg.tolist(),
        'tag_id': int(tag_id),
        'status': 'detected',
        'camera_params': camera_params,
    }


def _unpack_int32(data_bytes):
    return struct.unpack("=l", data_bytes)[0]


def read_current_robot_position():
    data = yrc.robot_position_data_read()
    if data is None or isinstance(data, tuple):
        return data

    x_raw = _unpack_int32(bytes(data["first_axis_data"]))
    y_raw = _unpack_int32(bytes(data["second_axis_data"]))
    z_raw = _unpack_int32(bytes(data["third_axis_data"]))
    tx_raw = _unpack_int32(bytes(data["fourth_axis_data"]))
    ty_raw = _unpack_int32(bytes(data["fifth_axis_data"]))
    tz_raw = _unpack_int32(bytes(data["sixth_axis_data"]))

    return {
        'x_mm': x_raw / 1000.0,
        'y_mm': y_raw / 1000.0,
        'z_mm': z_raw / 1000.0,
        'tx_deg': tx_raw / 10000.0,
        'ty_deg': ty_raw / 10000.0,
        'tz_deg': tz_raw / 10000.0,
        'raw': {
            'x': x_raw,
            'y': y_raw,
            'z': z_raw,
            'tx': tx_raw,
            'ty': ty_raw,
            'tz': tz_raw,
        }
    }

def transform_robot_to_tool(robot_pose, tool_offset):
    """
    Transform robot pose by a tool offset.

        Args:
                robot_pose:
                        - tuple/list -> (translation[3], rotation[3]) in mm / deg
                            Example: ([x, y, z], [rx, ry, rz])
                        - dict from read_current_robot_position() with keys
                            x_mm, y_mm, z_mm, tx_deg, ty_deg, tz_deg
        tool_offset: dict with keys: x, y, z, tx, ty, tz (mm / deg).

    Returns:
        dict with transformed translation, rotation and homogeneous matrix.
    """
    if isinstance(robot_pose, dict):
        pose_required = ("x_mm", "y_mm", "z_mm", "tx_deg", "ty_deg", "tz_deg")
        pose_missing = [k for k in pose_required if k not in robot_pose]
        if pose_missing:
            raise ValueError(f"robot_pose dict missing keys: {pose_missing}")
        translation = np.asarray(
            [robot_pose["x_mm"], robot_pose["y_mm"], robot_pose["z_mm"]],
            dtype=np.float64,
        ).reshape(3)
        rotation = np.asarray(
            [robot_pose["tx_deg"], robot_pose["ty_deg"], robot_pose["tz_deg"]],
            dtype=np.float64,
        ).reshape(3)
    elif isinstance(robot_pose, (tuple, list)) and len(robot_pose) == 2:
        translation = np.asarray(robot_pose[0], dtype=np.float64).reshape(3)
        rotation = np.asarray(robot_pose[1], dtype=np.float64).reshape(3)
    else:
        raise ValueError(
            "robot_pose must be (translation[3], rotation[3]) or "
            "dict with x_mm/y_mm/z_mm/tx_deg/ty_deg/tz_deg"
        )

    required = ("x", "y", "z", "tx", "ty", "tz")
    missing = [k for k in required if k not in tool_offset]
    if missing:
        raise ValueError(f"tool_offset missing keys: {missing}")

    h_robot = _pose_to_homogeneous(
        float(translation[0]),
        float(translation[1]),
        float(translation[2]),
        float(rotation[0]),
        float(rotation[1]),
        float(rotation[2]),
    )

    h_tool = _pose_to_homogeneous(
        float(tool_offset["x"]),
        float(tool_offset["y"]),
        float(tool_offset["z"]),
        float(tool_offset["tx"]),
        float(tool_offset["ty"]),
        float(tool_offset["tz"]),
    )

    h_out = h_robot @ h_tool
    out_translation = h_out[:3, 3]
    out_rotation = _rotation_matrix_to_euler_xyz_deg(h_out[:3, :3])

    translation_list = out_translation.tolist()
    rotation_list = out_rotation.tolist()

    return {
        "matrix": h_out.tolist(),
        "translation": translation_list,  # [x, y, z]
        "rotation": rotation_list,        # [tx, ty, tz]
        # Backward-compatible named keys used by existing call sites.
        "x_mm": translation_list[0],
        "y_mm": translation_list[1],
        "z_mm": translation_list[2],
        "tx_deg": rotation_list[0],
        "ty_deg": rotation_list[1],
        "tz_deg": rotation_list[2],
    }


_CAMERA_STATE = {
    'cap': None,
    'detector': None,
    'backend': None,
    'window_name': 'AprilTag Live View',
    'camera_id': None,
    'tag_size': 0.02375,
    'camera_params': (1439.4915254739828, 1441.9589818431248,
                      950.2746826769213, 550.7749059489563),
}


def camera_start(camera_id=1, tag_size=0.02375, window_name='AprilTag Live View'):
    if _CAMERA_STATE['cap'] is not None:
        return True

    detector, backend = _create_detector_with_fallback()
    if detector is None and backend == 'missing_backend':
        print("Error: apriltag DLL not found and pupil_apriltags is not installed.")
        print("Install fallback backend with: pip install pupil-apriltags")
        return False

    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        print(f"Error: Cannot open camera {camera_id}")
        return False

    # Request a practical preview resolution; camera may clamp to nearest supported mode.
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 1280, 720)

    _CAMERA_STATE['cap'] = cap
    _CAMERA_STATE['detector'] = detector
    _CAMERA_STATE['backend'] = backend
    _CAMERA_STATE['window_name'] = window_name
    _CAMERA_STATE['camera_id'] = camera_id
    _CAMERA_STATE['tag_size'] = float(tag_size)
    return True


def camera_wait_for_aruco_detect(camera_id=1, size=0.02375, timeout=10.0):
    # Keeps requested function name, but detection is AprilTag-based.
    if _CAMERA_STATE['cap'] is None:
        if not camera_start(camera_id=camera_id, tag_size=size):
            return None

    cap = _CAMERA_STATE['cap']
    detector = _CAMERA_STATE['detector']
    backend = _CAMERA_STATE['backend']
    window_name = _CAMERA_STATE['window_name']
    camera_params = _CAMERA_STATE['camera_params']
    tag_size = float(size)

    start_t = time.monotonic()
    pose = None
    frame = None
    detection = None
    tag_id = None

    while (time.monotonic() - start_t) < float(timeout):
        success, frame = cap.read()
        if not success:
            cv2.waitKey(1)
            continue

        # Keep the live view responsive regardless of detection outcome.
        cv2.imshow(window_name, frame)
        if cv2.waitKey(1) & 0xFF == 27:
            return None

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        try:
            if backend == 'apriltag_ctypes':
                detections, _ = detector.detect(gray, return_image=True)
            else:
                detections = detector.detect(
                    gray,
                    estimate_tag_pose=True,
                    camera_params=camera_params,
                    tag_size=tag_size,
                )
        except Exception:
            # Skip unstable frame and continue updating preview.
            continue

        if len(detections) == 0:
            continue

        detection = detections[0]
        if backend == 'apriltag_ctypes':
            pose, _e0, _e1 = detector.detection_pose(detection, camera_params, tag_size)
            tag_id = detection.tag_id
        else:
            if hasattr(detection, 'pose_R') and hasattr(detection, 'pose_t'):
                pose = np.eye(4)
                pose[:3, :3] = np.asarray(detection.pose_R)
                pose[:3, 3] = np.asarray(detection.pose_t).reshape(3)
                tag_id = int(detection.tag_id)

        if pose is not None:
            break
    # End of while timeout loop

    if pose is None or frame is None or detection is None:
        print("timeout reached")
        return None

    annotated = frame.copy()
    if hasattr(detection, 'corners'):
        corners = np.asarray(detection.corners).astype(int)
        cv2.polylines(annotated, [corners], True, (0, 255, 0), 2)
    _draw_pose_axes(annotated, camera_params, tag_size, pose)
    cv2.imshow(window_name, annotated)
    cv2.waitKey(1)
    return _build_pose_result(pose, tag_id, camera_params)


def camera_close():
    if _CAMERA_STATE['cap'] is not None:
        _CAMERA_STATE['cap'].release()
        _CAMERA_STATE['cap'] = None

    window_name = _CAMERA_STATE['window_name']
    try:
        cv2.destroyWindow(window_name)
    except cv2.error:
        pass

    _CAMERA_STATE['detector'] = None
    _CAMERA_STATE['backend'] = None
    _CAMERA_STATE['camera_id'] = None

def _pose_to_homogeneous(x, y, z, rx_deg, ry_deg, rz_deg):
    rx, ry, rz = np.radians([rx_deg, ry_deg, rz_deg])

    cx, sx = np.cos(rx), np.sin(rx)
    cy, sy = np.cos(ry), np.sin(ry)
    cz, sz = np.cos(rz), np.sin(rz)

    rx_m = np.array([[1.0, 0.0, 0.0],
                     [0.0, cx, -sx],
                     [0.0, sx, cx]], dtype=np.float64)
    ry_m = np.array([[cy, 0.0, sy],
                     [0.0, 1.0, 0.0],
                     [-sy, 0.0, cy]], dtype=np.float64)
    rz_m = np.array([[cz, -sz, 0.0],
                     [sz, cz, 0.0],
                     [0.0, 0.0, 1.0]], dtype=np.float64)

    r_m = rz_m @ ry_m @ rx_m
    # r_m = rx_m @ ry_m @ rz_m
    h = np.eye(4, dtype=np.float64)
    h[:3, :3] = r_m
    h[:3, 3] = [x, y, z]
    return h


def pose_translation_rpy_to_h(translation, rpy_deg):
    """Build 4x4 homogeneous matrix from [x,y,z] and [rx,ry,rz] in degrees."""
    return _pose_to_homogeneous(
        translation[0], translation[1], translation[2],
        rpy_deg[0], rpy_deg[1], rpy_deg[2]
    )


def wait_for_space_key():
    """Block until user presses the space key."""
    print("\nPress SPACE to move to H position (tool 26)...")
    try:
        import msvcrt
        while True:
            key = msvcrt.getwch()
            if key == ' ':
                break
    except Exception:
        while True:
            user_in = input("Type a single space and press Enter: ")
            if user_in == " ":
                break

def create_h_yaskawa(translation_mm, rotation_deg):
    """
    Create 4x4 homogeneous transform for Yaskawa Motoman Cartesian pose.

    Parameters
    ----------
    translation_mm : array-like of shape (3,)
        [X, Y, Z] in millimeters
    rotation_deg : array-like of shape (3,)
        [Rx, Ry, Rz] in degrees from Yaskawa controller

    Returns
    -------
    H : np.ndarray shape (4,4)
        Homogeneous transform
    """
    t = np.asarray(translation_mm, dtype=float).reshape(3)
    rx, ry, rz = np.asarray(rotation_deg, dtype=float).reshape(3)

    rx_rad, ry_rad, rz_rad = np.radians([rx, ry, rz])
    cx, sx = np.cos(rx_rad), np.sin(rx_rad)
    cy, sy = np.cos(ry_rad), np.sin(ry_rad)
    cz, sz = np.cos(rz_rad), np.sin(rz_rad)

    rx_m = np.array([[1.0, 0.0, 0.0],
                     [0.0, cx, -sx],
                     [0.0, sx, cx]], dtype=np.float64)
    ry_m = np.array([[cy, 0.0, sy],
                     [0.0, 1.0, 0.0],
                     [-sy, 0.0, cy]], dtype=np.float64)
    rz_m = np.array([[cz, -sz, 0.0],
                     [sz, cz, 0.0],
                     [0.0, 0.0, 1.0]], dtype=np.float64)

    # Yaskawa practical convention: R = Rz(rz) * Ry(ry) * Rx(rx)
    Rm = rz_m @ ry_m @ rx_m

    H = np.eye(4)
    H[:3, :3] = Rm
    H[:3, 3] = t
    return H


def h_to_move(H):
    h = np.asarray(H, dtype=np.float64)
    translation = h[:3, 3]
    rotation_matrix = h[:3, :3]
    rpy_deg = _rotation_matrix_to_euler_xyz_deg(rotation_matrix)
    return {
        "translation": translation,
        "rotation_matrix": rotation_matrix,
        "rpy_deg": rpy_deg,
    }


def compute_tool_to_camera_from_calibration():
    """Build fixed Tool->Camera transform from calibration poses (MATLAB-equivalent)."""
    tc_calib = create_h_yaskawa(
        [-0.37679815895757196 * M_TO_MM, -0.19150762252059986 * M_TO_MM, 0.8692484914166009 * M_TO_MM],
        [-48.44676638338047, 8.942934926705307, 129.4669873830014],
    )

    h1_calib = create_h_yaskawa(
        [521.006, 188.275, 584.78],
        [177.5772, -84.3365, 96.4181],
    )

    h2_calib = create_h_yaskawa(
        [704.412, 325.233, 372.982],
        [138.5319, 3.9771, 92.0742],
    )

    h_camera_global = h2_calib @ np.linalg.inv(tc_calib)
    return np.linalg.inv(h1_calib) @ h_camera_global

########################################################################################################################
# Tests
tool26 = {
    "no": 26,
    "x": -240.528,
    "y": -98.186,
    "z": 326.650,
    "tx": -2.5120,
    "ty": -3.0228,
    "tz": -133.1550,
}


#Home
X0 = 499*TRAN_PREE #in mm
Y0 = -167*TRAN_PREE
Z0 = 531*TRAN_PREE
TX0 = 179*ROT_PREE #in deg
TY0 = 0*ROT_PREE
TZ0 = -36*ROT_PREE

#Pos1
X1 = 499*TRAN_PREE #in mm
Y1 = -167*TRAN_PREE+250*TRAN_PREE
Z1 = 531*TRAN_PREE
TX1 = 179*ROT_PREE #in deg
TY1 = 0*ROT_PREE
TZ1 = -36*ROT_PREE

#point to camera transform
T_point_to_camera = np.array([
    [-0.1225, 0.7845, -0.6079, 765.0266],
    [-0.1315, -0.6200, -0.7735, 303.1642],
    [-0.9837, -0.0148, 0.1791, -330.9248],
    [0.0, 0.0, 0.0, 1.0000]
])

T_TOOL_TO_CAMERA = compute_tool_to_camera_from_calibration()




STEP_10CM = 100000   # 100 mm
SPEED = 2000          # 1 = 1.0 mm/s

print("Initial status:")
print(yrc.status_information_reading())

camera_start()

print("\nServo ON:")
print(yrc.servo_on())
print("Servos ON")

sleep(1)

#begin robot movement program

print("\nMove back to start:")
print(yrc.move_straight(X0, Y0, Z0, TX0, TY0, TZ0, SPEED, 0))
wait_move_finnish()


print("\nMove somewhere else:")
# print(yrc.move_link(521006, 188275, 584780, 1775772, -843365, 964181, SPEED, 26))
print(yrc.move_link(X1, Y1, Z1, TX1, TY1, TZ1, SPEED, 0))
wait_move_finnish()

print("\nGripper grab:")
gripper_grab()

sleep(2)

print("\nCapturing photo and detecting AprilTag...")

pose = camera_wait_for_aruco_detect(camera_id=1, size=0.02375, timeout=10.0)
T_camera_tag = None

if pose is None:
     print("Skipping move because AprilTag pose was not detected.")
else:
    if(pose['tag_id'] == 1):
        print("zaznan motor")
    if(pose['tag_id'] == 2):
        print("zaznan voziček")
    print("camera data")
    print(pose['translation'], pose['rpy_deg'], pose['tag_id'])

    T_camera_tag = pose_translation_rpy_to_h(pose['translation'], pose['rpy_deg'])
    print("T_camera_tag:")
    print(T_camera_tag)
    
    # Current tool pose in global frame (mm/deg)
    H_now_pos = read_current_robot_position()
    H_now = create_h_yaskawa(
        [H_now_pos['x_mm'], H_now_pos['y_mm'], H_now_pos['z_mm']],
        [H_now_pos['tx_deg'], H_now_pos['ty_deg'], H_now_pos['tz_deg']],
    )

    # MATLAB-equivalent runtime chain:
    # H_camera_now = H_now * T_tool_to_camera
    # H_target_tag = H_camera_now * T_camera_tag
    H_camera_now = H_now @ T_TOOL_TO_CAMERA
    H_target_tag = H_camera_now @ T_camera_tag
    print("H_target_tag:")
    print(H_target_tag)

    target_pose = h_to_move(H_target_tag)
    translation_mm = [
        float(target_pose["translation"][0]),
        float(target_pose["translation"][1]),
        float(target_pose["translation"][2]),
    ]
    rotation_deg = [
        float(target_pose["rpy_deg"][0]),
        float(target_pose["rpy_deg"][1]),
        float(target_pose["rpy_deg"][2]),
    ]

    print("wait for user confirm new move coordinates")
    print(translation_mm)
    print(rotation_deg)
    
    #correct rotation data to keep orientation
    H_now_tool26 = transform_robot_to_tool(H_now_pos, tool26)
    rotation_deg[0] = float(H_now_tool26['tx_deg'])
    rotation_deg[1] = float(H_now_tool26['ty_deg'])
    rotation_deg[2] = float(H_now_tool26['tz_deg'])

    cmd_x = int(round(translation_mm[0] * TRAN_PREE))
    cmd_y = int(round(translation_mm[1] * TRAN_PREE))
    cmd_z = int(round(translation_mm[2] * TRAN_PREE))
    cmd_tx = int(round(rotation_deg[0] * ROT_PREE))
    cmd_ty = int(round(rotation_deg[1] * ROT_PREE))
    cmd_tz = int(round(rotation_deg[2] * ROT_PREE))

    print("corrected rotation: to keep orientation")
    print(cmd_x, cmd_y, cmd_z, cmd_tx, cmd_ty, cmd_tz)

    wait_for_space_key()

    SPEED = 200
    print(yrc.move_straight(cmd_x, cmd_y, cmd_z + STEP_10CM, cmd_tx, cmd_ty, cmd_tz, SPEED, 26))
    # print(yrc.move_straight(translation[0], translation[1], translation[2]+STEP_10CM, 178, 3, -36, SPEED, 26))
    wait_move_finnish()


# print("\nRobot current position:")
# robot_pos = read_current_robot_position()
# print(robot_pos)

print("\nGripper release:")
gripper_release()

sleep(2)

#stop robot

print("\nServo OFF:")
print(yrc.servo_off())
print("Servos OFF")

camera_close()
