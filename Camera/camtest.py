#
#RUN WITH
# python3 .\camtest.py 1 -k 1032.4,1030.9,640.2,360.7 -s 0.02375
# python3 .\camtest.py 1 -k 1436.9664067347683,1434.9385963643442,945.7587609196078,542.7543358086843 -s 0.02375
# python3 .\camtest.py 1 -k 1439.4915254739828,1441.9589818431248,950.2746826769213,550.7749059489563 -s 0.02375


#!/usr/bin/env python

'''Demonstrate Python wrapper of C apriltag library by running on camera frames.'''
from __future__ import division
from __future__ import print_function

from argparse import ArgumentParser
import cv2
import numpy as np
import apriltag

# for some reason pylint complains about members being undefined :(
# pylint: disable=E1101


def _create_detector_with_fallback(options):
    """Create detector, falling back to pupil_apriltags if native DLL is missing."""

    try:
        detector = apriltag.Detector(options,
                                     searchpath=apriltag._get_demo_searchpath())
        return detector, 'apriltag_ctypes'
    except RuntimeError as exc:
        if 'could not find apriltag shared library' not in str(exc):
            raise

        try:
            from pupil_apriltags import Detector as PupilDetector
        except ImportError as import_exc:
            raise RuntimeError(
                'apriltag DLL was not found and pupil_apriltags is not installed. '
                'Install fallback backend with: pip install pupil-apriltags'
            ) from import_exc

        detector = PupilDetector(
            families=getattr(options, 'families', 'tag36h11'),
            nthreads=int(getattr(options, 'nthreads', 1)),
            quad_decimate=float(getattr(options, 'quad_decimate', 1.0)),
            quad_sigma=float(getattr(options, 'quad_sigma', 0.0)),
            refine_edges=bool(getattr(options, 'refine_edges', True)),
        )

        print('Native apriltag DLL not found, using pupil_apriltags fallback backend.')
        return detector, 'pupil_apriltags'


def _draw_pupil_overlay(frame, detections):
    """Draw detections from pupil_apriltags onto a copy of frame."""

    overlay = frame.copy()

    for det in detections:
        corners = det.corners.astype(int)
        cv2.polylines(overlay, [corners], True, (0, 255, 0), 2)

        center = tuple(det.center.astype(int))
        cv2.circle(overlay, center, 4, (0, 255, 255), -1)

        family = det.tag_family
        if isinstance(family, bytes):
            family = family.decode('utf-8', errors='replace')
        label = '{}:{}'.format(family, det.tag_id)
        cv2.putText(overlay, label, center,
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)

    return overlay


def _rotation_matrix_to_euler_xyz_deg(R):
    """Convert 3x3 rotation matrix to roll, pitch, yaw in degrees."""

    sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6

    if not singular:
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = 0.0

    return np.degrees([roll, pitch, yaw])


def _pose_values_from_matrix(pose):
    """Return translation and orientation from a 4x4 pose matrix."""

    t = pose[:3, 3]
    rpy = _rotation_matrix_to_euler_xyz_deg(pose[:3, :3])
    return t, rpy


def _quality_ok(detection, min_margin, max_hamming):
    """Filter weak detections that cause unstable pose estimates."""

    margin = float(getattr(detection, 'decision_margin', 0.0))
    hamming = int(getattr(detection, 'hamming', 0))
    return (margin >= min_margin) and (hamming <= max_hamming)


def _median_pose(history):
    """Compute robust median pose from recent samples."""

    t_stack = np.stack([p[0] for p in history], axis=0)
    rpy_stack = np.stack([p[1] for p in history], axis=0)
    return np.median(t_stack, axis=0), np.median(rpy_stack, axis=0)


def _draw_pose_axes(overlay, camera_params, tag_size, pose):
    """Draw tag coordinate axes on the overlay image using pose."""

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
    cv2.line(overlay, origin, tuple(pts[1]), (0, 0, 255), 2, cv2.LINE_AA)   # X red
    cv2.line(overlay, origin, tuple(pts[2]), (0, 255, 0), 2, cv2.LINE_AA)   # Y green
    cv2.line(overlay, origin, tuple(pts[3]), (255, 0, 0), 2, cv2.LINE_AA)   # Z blue

    cv2.putText(overlay, 'X', tuple(pts[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
    cv2.putText(overlay, 'Y', tuple(pts[2]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
    cv2.putText(overlay, 'Z', tuple(pts[3]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1, cv2.LINE_AA)

def main():

    '''Main function.'''

    parser = ArgumentParser(
        description='test apriltag Python bindings')

    parser.add_argument('device_or_movie', metavar='INPUT', nargs='?', default=0,
                        help='Movie to load or integer ID of camera device')

    parser.add_argument('-k', '--camera-params', type=apriltag._camera_params,
                        default=None,
                        help='intrinsic parameters fx,fy,cx,cy for pose estimation')

    parser.add_argument('-s', '--tag-size', type=float,
                        default=1.0,
                        help='physical tag size in user-selected units (for pose scale)')

    parser.add_argument('--min-margin', type=float, default=30.0,
                        help='minimum decision margin to accept detection for pose')

    parser.add_argument('--max-hamming', type=int, default=0,
                        help='maximum hamming distance to accept detection for pose')

    parser.add_argument('--smooth-window', type=int, default=7,
                        help='rolling window size for median pose smoothing')

    parser.add_argument('--print-every', type=int, default=3,
                        help='print one pose line every N frames')

    apriltag.add_arguments(parser)

    options = parser.parse_args()

    try:
        cap = cv2.VideoCapture(int(options.device_or_movie))
    except ValueError:
        cap = cv2.VideoCapture(options.device_or_movie)

    window = 'Camera'
    cv2.namedWindow(window)

    # set up a reasonable search path for the apriltag DLL inside the
    # github repo this file lives in;
    #
    # for "real" deployments, either install the DLL in the appropriate
    # system-wide library directory, or specify your own search paths
    # as needed.
    
    detector, backend = _create_detector_with_fallback(options)
    pose_history = {}
    frame_idx = 0

    while True:

        success, frame = cap.read()
        if not success:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        if backend == 'apriltag_ctypes':
            detections, dimg = detector.detect(gray, return_image=True)
            overlay = frame // 2 + dimg[:, :, None] // 2
        else:
            estimate_pose = (options.camera_params is not None)
            detections = detector.detect(gray,
                                        estimate_tag_pose=estimate_pose,
                                        camera_params=options.camera_params,
                                        tag_size=options.tag_size)
            overlay = _draw_pupil_overlay(frame, detections)

        frame_idx += 1

        for detection in detections:
            if not _quality_ok(detection, options.min_margin, options.max_hamming):
                continue

            if backend == 'apriltag_ctypes':
                if options.camera_params is not None:
                    pose, _e0, _e1 = detector.detection_pose(detection,
                                                             options.camera_params,
                                                             options.tag_size)
                    t, rpy = _pose_values_from_matrix(pose)
                    det_id = getattr(detection, 'tag_id', getattr(detection, 'id', -1))
                    pose_history.setdefault(det_id, []).append((t, rpy))
                    if len(pose_history[det_id]) > options.smooth_window:
                        pose_history[det_id].pop(0)
                    t_s, rpy_s = _median_pose(pose_history[det_id])
                    _draw_pose_axes(overlay, options.camera_params, options.tag_size, pose)
                    if (frame_idx % max(1, options.print_every)) == 0:
                        print('id={} t=[{:.6f},{:.6f},{:.6f}] rpy_deg=[{:.3f},{:.3f},{:.3f}]'.format(
                            det_id, t_s[0], t_s[1], t_s[2], rpy_s[0], rpy_s[1], rpy_s[2]))
            else:
                if options.camera_params is not None and hasattr(detection, 'pose_R') and hasattr(detection, 'pose_t'):
                    pose = np.eye(4)
                    pose[:3, :3] = np.asarray(detection.pose_R)
                    pose[:3, 3] = np.asarray(detection.pose_t).reshape(3)
                    t, rpy = _pose_values_from_matrix(pose)
                    det_id = int(detection.tag_id)
                    pose_history.setdefault(det_id, []).append((t, rpy))
                    if len(pose_history[det_id]) > options.smooth_window:
                        pose_history[det_id].pop(0)
                    t_s, rpy_s = _median_pose(pose_history[det_id])
                    _draw_pose_axes(overlay, options.camera_params, options.tag_size, pose)
                    if (frame_idx % max(1, options.print_every)) == 0:
                        print('id={} t=[{:.6f},{:.6f},{:.6f}] rpy_deg=[{:.3f},{:.3f},{:.3f}]'.format(
                            det_id, t_s[0], t_s[1], t_s[2], rpy_s[0], rpy_s[1], rpy_s[2]))

        cv2.imshow(window, overlay)
        k = cv2.waitKey(1)

        if k == 27:
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
