"""ArUco Vision GPS - detects ceiling markers and sends position to flight controller."""

import time
import signal
import sys
import json
import logging
import argparse
import threading
from collections import namedtuple
from pathlib import Path
from http.server import HTTPServer, BaseHTTPRequestHandler

import cv2
import numpy as np
import yaml

log = logging.getLogger(__name__)

Detection = namedtuple("Detection", "marker_id rvec tvec distance timestamp centroid")
DroneState = namedtuple("DroneState", "n e d yaw marker_ids confidence timestamp marker_weights")

# --- Config loaders ---

def load_camera_params(path, target_width=None, target_height=None):
    with open(path) as f:
        data = yaml.safe_load(f)
    cm = data["camera_matrix"]
    matrix = np.array(cm["data"] if isinstance(cm, dict) and "data" in cm else cm).reshape(3, 3)
    dc = data["distortion_coefficients"]
    coeffs = np.array(dc["data"] if isinstance(dc, dict) and "data" in dc else dc)
    # Scale camera matrix if running at different resolution than calibration
    cal_w = data.get("image_width", target_width)
    cal_h = data.get("image_height", target_height)
    if target_width and target_height and cal_w and cal_h:
        sx = target_width / cal_w
        sy = target_height / cal_h
        if abs(sx - 1.0) > 0.01 or abs(sy - 1.0) > 0.01:
            matrix[0, 0] *= sx  # fx
            matrix[0, 2] *= sx  # cx
            matrix[1, 1] *= sy  # fy
            matrix[1, 2] *= sy  # cy
            log.info(f"Scaled camera matrix from {cal_w}x{cal_h} to {target_width}x{target_height}")
    # Load level calibration tvec if present
    level_tvec = data.get("level_tvec")
    return matrix, coeffs, level_tvec


def load_marker_map(path):
    with open(path) as f:
        data = yaml.safe_load(f)
    markers = {}
    for m in data.get("markers", []):
        markers[str(m["id"])] = {
            "position": np.array(m["position"]),
            "orientation": m.get("orientation", 0),
            "size": m.get("size", None),  # per-marker size in meters (optional)
        }
    log.info(f"Loaded {len(markers)} markers from {path}")
    return markers

# --- Camera ---

class Camera:
    def __init__(self, device_id=0, width=1280, height=720, fps=30):
        self.device_id = device_id
        self.width = width
        self.height = height
        self.fps = fps
        self._cap = None
        self._frame = None
        self._frame_time = 0.0  # timestamp of last captured frame
        self._lock = threading.Lock()
        self._running = False

    def start(self):
        self._cap = cv2.VideoCapture(self.device_id, cv2.CAP_V4L2)
        if not self._cap.isOpened():
            # Fallback to auto backend
            self._cap = cv2.VideoCapture(self.device_id)
        if not self._cap.isOpened():
            log.error(f"Failed to open camera {self.device_id}")
            return False
        self._cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self._cap.set(cv2.CAP_PROP_FPS, self.fps)
        self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        # Lock focus at infinity — marker is always >1m, small sensor DOF covers it
        self._cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        self._cap.set(cv2.CAP_PROP_FOCUS, 0)
        log.info("Focus locked at infinity")
        actual_buf = self._cap.get(cv2.CAP_PROP_BUFFERSIZE)
        log.info(f"V4L2 buffer size: {actual_buf}")
        self._running = True
        threading.Thread(target=self._capture_loop, daemon=True).start()
        log.info(f"Camera started at {self.width}x{self.height}")
        return True

    def stop(self):
        self._running = False
        if self._cap:
            self._cap.release()

    def get_frame(self):
        with self._lock:
            if self._frame is not None:
                return self._frame.copy(), self._frame_time
            return None, 0.0

    # Camera controls exposed for live tuning
    _CONTROLS = {
        "brightness":  cv2.CAP_PROP_BRIGHTNESS,
        "contrast":    cv2.CAP_PROP_CONTRAST,
        "saturation":  cv2.CAP_PROP_SATURATION,
        "gain":        cv2.CAP_PROP_GAIN,
        "exposure":    cv2.CAP_PROP_EXPOSURE,
        "auto_exposure": cv2.CAP_PROP_AUTO_EXPOSURE,
        "focus":       cv2.CAP_PROP_FOCUS,
        "autofocus":   cv2.CAP_PROP_AUTOFOCUS,
        "white_balance": cv2.CAP_PROP_WB_TEMPERATURE,
        "auto_wb":     cv2.CAP_PROP_AUTO_WB,
        "sharpness":   cv2.CAP_PROP_SHARPNESS,
    }

    def get_config(self):
        if not self._cap:
            return {}
        cfg = {}
        for name, prop in self._CONTROLS.items():
            val = self._cap.get(prop)
            if val != -1 and val != 0.0 or name in ("brightness", "focus", "autofocus", "auto_wb"):
                cfg[name] = val
        return cfg

    def set_config(self, settings):
        if not self._cap:
            return {}
        applied = {}
        for name, val in settings.items():
            if name in self._CONTROLS:
                self._cap.set(self._CONTROLS[name], float(val))
                applied[name] = self._cap.get(self._CONTROLS[name])
                log.info(f"Camera {name} = {applied[name]}")
        return applied

    def _capture_loop(self):
        while self._running and self._cap:
            # Drain stale V4L2 buffers — grab discards without decoding
            self._cap.grab()
            ret, frame = self._cap.read()
            if ret:
                with self._lock:
                    self._frame = frame
                    self._frame_time = time.perf_counter()
            else:
                time.sleep(0.001)

# --- Detection ---

def create_detector(dictionary="DICT_4X4_50"):
    aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary))
    # Support both old (4.x) and new (4.7+) ArUco API
    if hasattr(cv2.aruco, 'DetectorParameters_create'):
        params = cv2.aruco.DetectorParameters_create()
    else:
        params = cv2.aruco.DetectorParameters()
    params.adaptiveThreshConstant = 7
    params.adaptiveThreshWinSizeMin = 3
    params.adaptiveThreshWinSizeMax = 23
    params.adaptiveThreshWinSizeStep = 14        # was 10 → fewer threshold passes (2 instead of 3)
    params.minMarkerPerimeterRate = 0.02          # was 0.01 → skip tiny false positives
    params.maxMarkerPerimeterRate = 4.0
    params.polygonalApproxAccuracyRate = 0.05
    params.minCornerDistanceRate = 0.01
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_CONTOUR
    params.minOtsuStdDev = 5.0
    params.perspectiveRemovePixelPerCell = 4
    params.perspectiveRemoveIgnoredMarginPerCell = 0.13
    if hasattr(cv2.aruco, 'ArucoDetector'):
        return cv2.aruco.ArucoDetector(aruco_dict, params)
    # Old API: return (dict, params) tuple, detect() handles the difference
    return (aruco_dict, params)


# Persistent PnP state for temporal consistency (per marker ID)
_last_pnp = {}  # marker_id -> (rvec, tvec)


_last_had_detection = False  # track whether CLAHE can be skipped

def _run_detect(image, detector):
    """Run ArUco detection on a grayscale or BGR image."""
    if isinstance(detector, tuple):
        return cv2.aruco.detectMarkers(image, detector[0], parameters=detector[1])
    else:
        return detector.detectMarkers(image)

def detect(frame, clahe, detector, cam_matrix, dist_coeffs, marker_size, marker_sizes=None):
    """marker_size: default size. marker_sizes: dict {str(id): size_m} for per-marker override."""
    global _last_had_detection
    t0 = time.perf_counter()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    t1 = time.perf_counter()
    # Conditional CLAHE: skip when previous frame had good detection
    if _last_had_detection:
        enhanced = gray
    else:
        enhanced = clahe.apply(gray)
    t2 = time.perf_counter()

    try:
        corners, ids, _ = _run_detect(enhanced, detector)
    except cv2.error as e:
        log.warning(f"Detection error: {e}")
        return [], {}

    # If skipping CLAHE lost detection, retry with CLAHE
    if ids is None and _last_had_detection:
        enhanced = clahe.apply(gray)
        try:
            corners, ids, _ = _run_detect(enhanced, detector)
        except cv2.error:
            pass

    _last_had_detection = ids is not None and len(ids) > 0

    t4 = time.perf_counter()
    timing = {
        "gray": f"{(t1-t0)*1000:.0f}",
        "clahe": f"{(t2-t1)*1000:.0f}",
        "detect": f"{(t4-t2)*1000:.0f}",
        "total": f"{(t4-t0)*1000:.0f}",
    }

    if ids is None:
        return [], timing

    ts = time.time()
    detections = []
    for i, mid in enumerate(ids.flatten()):
        corners_2d = corners[i].reshape(-1, 2)
        mid_int = int(mid)
        sz = marker_sizes.get(str(mid_int), marker_size) if marker_sizes else marker_size
        half = sz / 2
        obj_pts = np.array([[-half, half, 0], [half, half, 0],
                             [half, -half, 0], [-half, -half, 0]], dtype=np.float32)

        # Get both IPPE solutions and pick the physically valid one
        try:
            n_solutions, rvecs, tvecs, reproj_errors = cv2.solvePnPGeneric(
                obj_pts, corners_2d, cam_matrix, dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE)
        except Exception:
            # Fallback for older OpenCV without solvePnPGeneric
            ok, rvec, tvec = cv2.solvePnP(
                obj_pts, corners_2d, cam_matrix, dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok:
                continue
            rvecs, tvecs = [rvec], [tvec]
            n_solutions = 1

        # Filter: marker must be above camera (tvec Z > 0 in camera frame)
        valid_idx = [j for j in range(n_solutions) if tvecs[j].flatten()[2] > 0]
        if not valid_idx:
            continue

        if len(valid_idx) == 1:
            rvec, tvec = rvecs[valid_idx[0]], tvecs[valid_idx[0]]
        else:
            # Two valid solutions — disambiguate by marker Z direction.
            # Ceiling marker Z axis should point straight down toward camera.
            # Correct solution: marker Z in camera frame ≈ [0, 0, -1].
            # Pick the solution where marker Z has smallest XY deviation (most vertical).
            def marker_z_tilt(idx):
                R = cv2.Rodrigues(rvecs[idx])[0]
                mz = R[:, 2]  # marker Z in camera frame
                return mz[0]**2 + mz[1]**2  # XY magnitude (0 = perfectly vertical)
            best_idx = min(valid_idx, key=marker_z_tilt)
            rvec, tvec = rvecs[best_idx], tvecs[best_idx]

        tvec_flat = tvec.flatten()
        dist = float(np.linalg.norm(tvec_flat))
        # Sanity check: discard if distance > 10m or rvec explodes
        if dist > 10.0 or np.any(np.abs(rvec) > 2 * np.pi * 10):
            _last_pnp.pop(mid_int, None)
            continue
        _last_pnp[mid_int] = (rvec.copy(), tvec.copy())
        centroid = corners_2d.mean(axis=0)
        detections.append(Detection(
            mid_int, rvec.flatten(), tvec_flat,
            dist, ts, centroid))

    return detections, timing

# --- Position estimation ---

# Nominal camera-to-body rotation (physical mounting, upward-facing camera)
# cam_-X → body forward, cam_Y → body right, cam_Z → body up
R_CB_NOMINAL = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]], dtype=float)  # det=+1, proper rotation (180° around Y)

# R_CB will be set at startup: R_CB_NOMINAL @ R_tilt (from level calibration)
R_CB = R_CB_NOMINAL.copy()
_level_tvec = None  # Set at startup from camera_params.yaml


def compute_R_CB(level_tvec):
    """Compute R_CB including camera tilt correction from level calibration.

    level_tvec: [x, y, z] tvec recorded when drone is directly under marker, level.
    R_tilt is the rotation that maps this tvec to [0, 0, |tvec|] (straight up).
    """
    if level_tvec is None:
        log.info("No level calibration — using nominal R_CB")
        return R_CB_NOMINAL.copy()

    tvec = np.array(level_tvec)
    v1 = tvec / np.linalg.norm(tvec)
    v2 = np.array([0.0, 0.0, 1.0])  # ideal: marker straight up in camera frame

    axis = np.cross(v1, v2)
    axis_norm = np.linalg.norm(axis)
    if axis_norm < 1e-10:
        log.info("Level calibration: camera already vertical, no tilt correction")
        return R_CB_NOMINAL.copy()

    axis = axis / axis_norm
    angle = np.arccos(np.clip(np.dot(v1, v2), -1, 1))

    # Rodrigues formula: rotation matrix from axis-angle
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    R_tilt = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)

    R = R_CB_NOMINAL @ R_tilt
    log.info(f"Level calibration: tilt={np.degrees(angle):.1f}deg "
             f"axis=({axis[0]:.3f}, {axis[1]:.3f}, {axis[2]:.3f})")
    return R


def marker_to_world_rotation(orientation_deg):
    """Fixed rotation from marker frame to world (NED) for ceiling-mounted marker.
    orientation_deg: physical direction of marker top edge (NED compass, as seen from below).
    Subtract 180° because face-down mounting flips the detected marker frame vs physical."""
    theta = np.radians(orientation_deg)
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def estimate_single(detection, marker_cfg, imu_attitude=None):
    # tvec (reliable) + IMU pitch/roll (reliable) + vision yaw (reliable).
    # R_cm is NOT used for position — only for yaw extraction (Z-rotation is well-conditioned).
    tvec = detection.tvec
    marker_pos = np.array(marker_cfg["position"])
    R_mw = marker_to_world_rotation(marker_cfg["orientation"])
    R_cm, _ = cv2.Rodrigues(detection.rvec)

    # Yaw: from vision (in-plane rotation is well-conditioned even at perpendicular viewing)
    R_bw = R_mw @ R_cm.T @ R_CB_NOMINAL.T
    yaw = np.arctan2(R_bw[1, 0], R_bw[0, 0])
    # DEBUG: log yaw chain
    _v = R_cm.T @ R_CB_NOMINAL.T[:, 0]
    log.debug(f"YAW_DEBUG: R_cm[0,:]={R_cm[0,:]}, v={_v}, R_bw_col0={R_bw[:,0]}, yaw_rad={yaw:.3f} yaw_deg={np.degrees(yaw):.1f}")

    # Position: angle-based (avoids full R_CB rotation chain for better stability).
    cam_ax = np.arctan2(tvec[0], tvec[2])  # marker angle along camera X
    cam_ay = np.arctan2(tvec[1], tvec[2])  # marker angle along camera Y
    height = float(np.linalg.norm(tvec))

    if imu_attitude is not None:
        roll_imu, pitch_imu = imu_attitude[0], imu_attitude[1]
    else:
        roll_imu, pitch_imu = 0.0, 0.0

    # Static tilt offset from level calibration (so level+centered = 0)
    if _level_tvec is not None:
        level_ax = np.arctan2(_level_tvec[0], _level_tvec[2])
        level_ay = np.arctan2(_level_tvec[1], _level_tvec[2])
    else:
        level_ax, level_ay = 0.0, 0.0

    # World-frame angles: IMU corrects for drone tilt, level cal corrects for camera mount
    world_ax = pitch_imu + (cam_ax - level_ax)
    world_ay = roll_imu + (cam_ay - level_ay)

    # Body-frame offset (cam_X = -body_fwd, cam_Y = body_right)
    body_fwd = -height * np.tan(world_ax)
    body_right = height * np.tan(world_ay)
    body_down = -height  # marker is above drone → negative down
    estimate_single._debug_body = (body_fwd, body_right, body_down)

    # Rotate body offset to world and subtract from marker pos to get drone pos
    # (body_fwd/right point from drone toward marker; drone = marker - offset)
    cy, sy = np.cos(yaw), np.sin(yaw)
    pos = marker_pos - np.array([
        cy * body_fwd - sy * body_right,  # North
        sy * body_fwd + cy * body_right,  # East
        body_down,                         # Down
    ])

    # In NED, atan2(R_bw[1,0], R_bw[0,0]) = atan2(East, North) = compass heading directly
    yaw_compass = np.degrees(yaw)
    return pos, yaw_compass


def estimate_position(detections, marker_map, last_state, cam_matrix, imu_attitude=None):
    valid = [(d, marker_map[str(d.marker_id)]) for d in detections if str(d.marker_id) in marker_map]
    if not valid:
        return last_state

    if len(valid) == 1:
        d, cfg = valid[0]
        pos, yaw = estimate_single(d, cfg, imu_attitude)
        ids = [str(d.marker_id)]
        marker_weights = {str(d.marker_id): 1.0}
        conf = 1.0
    else:
        positions, yaws, weights, ids = [], [], [], []
        for d, cfg in valid:
            pos, yaw = estimate_single(d, cfg, imu_attitude)
            horiz_dist = np.sqrt(d.tvec[0]**2 + d.tvec[1]**2)
            w = 1.0 / (horiz_dist + 0.1)
            positions.append(pos * w)
            yaws.append((yaw, w))
            weights.append(w)
            ids.append(str(d.marker_id))
        tw = sum(weights)
        marker_weights = {mid: round(w / tw, 3) for mid, w in zip(ids, weights)}
        pos = sum(positions) / tw
        sin_sum = sum(np.sin(np.radians(y)) * w for y, w in yaws)
        cos_sum = sum(np.cos(np.radians(y)) * w for y, w in yaws)
        yaw = np.degrees(np.arctan2(sin_sum, cos_sum))
        stds = np.std([p / w for p, w in zip(positions, weights)], axis=0).mean()
        conf = min(1.0, len(valid) / 3.0) * np.exp(-stds)

    return DroneState(float(pos[0]), float(pos[1]), float(pos[2]),
                      float(yaw), ids, float(conf), time.time(), marker_weights)

# --- HTTP Position Server ---

class PositionServer:
    def __init__(self, port=8001):
        self.port = port
        self.state = None
        self.frame_jpeg = None
        self.marker_map = {}
        self.camera = None
        self._origin_lat = None
        self._origin_lon = None
        self.stats = {"frames": 0, "detections": 0, "start": time.time(), "timing": {}}
        self._lock = threading.Lock()
        self._server = None

    def update(self, state, frames, detections, debug_frame=None, timing=None,
               imu_attitude=None, cam_angles=None, fc_position=None, body_tvec=None, body_offset=None):
        with self._lock:
            self.state = state
            self.stats["frames"] = frames
            self.stats["detections"] = detections
            if timing:
                self.stats["timing"] = timing
            self.stats["imu"] = imu_attitude
            self.stats["cam_angles"] = cam_angles
            self.stats["fc_position"] = fc_position
            self.stats["body_tvec"] = body_tvec
            self.stats["body_offset"] = body_offset
            if debug_frame is not None:
                _, jpg = cv2.imencode(".jpg", debug_frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                self.frame_jpeg = jpg.tobytes()

    def _json(self):
        with self._lock:
            s, st = self.state, self.stats
            fc, dc = st["frames"], st["detections"]
            base = {"frame_count": fc, "detection_count": dc,
                    "detection_rate": round(dc / max(1, fc), 2),
                    "uptime": round(time.time() - st["start"], 1), "timing": st["timing"]}
            if s:
                base.update({"n": round(s.n, 3), "e": round(s.e, 3), "d": round(s.d, 3),
                             "yaw": round(s.yaw, 1), "marker_ids": s.marker_ids,
                             "confidence": round(s.confidence, 2),
                             "marker_weights": s.marker_weights})
            imu = st.get("imu")
            if imu:
                base["imu_roll"] = round(np.degrees(imu[0]), 2)
                base["imu_pitch"] = round(np.degrees(imu[1]), 2)
            cam = st.get("cam_angles")
            if cam:
                base["cam_pitch"] = round(cam[0], 2)
                base["cam_roll"] = round(cam[1], 2)
            bt = st.get("body_tvec")
            if bt is not None:
                base["body_tvec"] = {"fwd": round(float(bt[0]), 3),
                                     "right": round(float(bt[1]), 3),
                                     "down": round(float(bt[2]), 3)}
            bo = st.get("body_offset")
            if bo is not None:
                base["body_offset"] = {"fwd": round(float(bo[0]), 3),
                                       "right": round(float(bo[1]), 3),
                                       "down": round(float(bo[2]), 3)}
            fc = st.get("fc_position")
            if fc and self._origin_lat is not None:
                fc_lat, fc_lon, fc_alt, fc_yaw = fc
                fc_n = (fc_lat - self._origin_lat) * 111111.0
                fc_e = (fc_lon - self._origin_lon) * 111111.0 * np.cos(np.radians(self._origin_lat))
                base["fc"] = {"n": round(fc_n, 3), "e": round(fc_e, 3),
                              "d": round(-fc_alt, 3), "yaw": round(fc_yaw, 1)}
            return json.dumps(base)

    def start(self):
        ref = self
        class H(BaseHTTPRequestHandler):
            def log_message(self, *a): pass
            def do_GET(self):
                if self.path in ("/", "/position"):
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Access-Control-Allow-Origin", "*")
                    self.end_headers()
                    self.wfile.write(ref._json().encode())
                elif self.path == "/markers":
                    data = [{"id": int(k), "n": float(v["position"][0]), "e": float(v["position"][1]),
                             "d": float(v["position"][2])} for k, v in ref.marker_map.items()]
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Access-Control-Allow-Origin", "*")
                    self.end_headers()
                    self.wfile.write(json.dumps(data).encode())
                elif self.path == "/debug-frame":
                    with ref._lock:
                        jpg = ref.frame_jpeg
                    if jpg:
                        self.send_response(200)
                        self.send_header("Content-Type", "image/jpeg")
                        self.send_header("Cache-Control", "no-cache")
                        self.send_header("Access-Control-Allow-Origin", "*")
                        self.end_headers()
                        self.wfile.write(jpg)
                    else:
                        self.send_response(503)
                        self.send_header("Access-Control-Allow-Origin", "*")
                        self.end_headers()
                        self.wfile.write(b"No frame")
                elif self.path == "/camera-config":
                    cfg = ref.camera.get_config() if ref.camera else {}
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Access-Control-Allow-Origin", "*")
                    self.end_headers()
                    self.wfile.write(json.dumps(cfg).encode())
                else:
                    self.send_error(404)
            def do_POST(self):
                if self.path == "/camera-config":
                    length = int(self.headers.get("Content-Length", 0))
                    body = self.rfile.read(length) if length else b"{}"
                    try:
                        settings = json.loads(body)
                    except Exception:
                        self.send_error(400); return
                    applied = ref.camera.set_config(settings) if ref.camera else {}
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Access-Control-Allow-Origin", "*")
                    self.end_headers()
                    self.wfile.write(json.dumps(applied).encode())
                else:
                    self.send_error(404)
            def do_OPTIONS(self):
                self.send_response(204)
                self.send_header("Access-Control-Allow-Origin", "*")
                self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
                self.send_header("Access-Control-Allow-Headers", "Content-Type")
                self.end_headers()
        class ReusableHTTPServer(HTTPServer):
            allow_reuse_address = True
        self._server = ReusableHTTPServer(("0.0.0.0", self.port), H)
        threading.Thread(target=self._server.serve_forever, daemon=True).start()
        log.info(f"Position server on port {self.port}")

    def stop(self):
        if self._server:
            self._server.shutdown()

# --- Main ---

def main():
    parser = argparse.ArgumentParser(description="ArUco Vision GPS")
    parser.add_argument("--config", default="config/system_config.yaml")
    parser.add_argument("--mode", choices=["run", "test", "stream"], default="test")
    parser.add_argument("--port", type=int, default=8001)
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")

    config_path = Path(args.config)
    if not config_path.is_absolute():
        config_path = Path(__file__).parent.parent / config_path
    with open(config_path) as f:
        cfg = yaml.safe_load(f)
    config_dir = config_path.parent

    cam_w = cfg.get("camera", {}).get("width", 1280)
    cam_h = cfg.get("camera", {}).get("height", 720)
    cam_matrix, dist_coeffs, level_tvec = load_camera_params(
        config_dir / "camera_params.yaml", target_width=cam_w, target_height=cam_h)

    # Level calibration for position estimation
    global R_CB, _level_tvec
    R_CB = compute_R_CB(level_tvec)
    _level_tvec = np.array(level_tvec) if level_tvec is not None else None

    marker_map = load_marker_map(config_dir / "marker_map.yaml")
    aruco_cfg = cfg.get("aruco", {})
    marker_size = aruco_cfg.get("marker_size_m", 0.18)
    marker_sizes = {mid: m["size"] for mid, m in marker_map.items() if m.get("size")}
    if marker_sizes:
        log.info(f"Per-marker sizes: {marker_sizes}")

    clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8))
    detector = create_detector(aruco_cfg.get("dictionary", "DICT_4X4_50"))

    camera = Camera(
        device_id=cfg.get("camera", {}).get("device_id", 0),
        width=cfg.get("camera", {}).get("width", 1280),
        height=cfg.get("camera", {}).get("height", 720),
        fps=cfg.get("camera", {}).get("fps", 30),
    )
    if not camera.start():
        sys.exit(1)
    # loop_period removed — run as fast as possible, detection is the bottleneck

    origin = cfg.get("ekf_origin", {})
    origin_lat = origin.get("lat", 52.2297)
    origin_lon = origin.get("lon", 21.0122)
    origin_alt = origin.get("alt", 100.0)

    mavlink = None
    if args.mode == "run":
        from .mavlink_bridge import MAVLinkBridge
        serial_cfg = cfg.get("serial", {})
        mavlink = MAVLinkBridge(serial_cfg.get("port", "/dev/serial0"), serial_cfg.get("baud", 921600))
        if not mavlink.connect(timeout=30.0):
            log.error("Failed to connect to flight controller")
            sys.exit(1)
        log.info("Using GPS emulation mode")

    server = PositionServer(port=args.port)
    server.marker_map = marker_map
    server.camera = camera
    server._origin_lat = origin_lat
    server._origin_lon = origin_lon
    server.start()

    shutdown = [False]
    signal.signal(signal.SIGINT, lambda *_: shutdown.__setitem__(0, True))
    signal.signal(signal.SIGTERM, lambda *_: shutdown.__setitem__(0, True))

    # CSV log for debugging position estimation
    csv_path = Path(__file__).parent.parent / "debug_log.csv"
    csv_file = open(csv_path, "w")
    csv_file.write("frame,time,marker_id,cx,cy,tvec_x,tvec_y,tvec_z,"
                   "rvec_x,rvec_y,rvec_z,pos_n,pos_e,pos_d,yaw\n")
    log.info(f"Debug CSV: {csv_path}")

    # Optical flow state for inter-frame interpolation
    prev_gray = None
    prev_features = None
    flow_redetect_interval = 10  # re-pick features every N frames
    fx = cam_matrix[0, 0]  # focal length in pixels (X)
    fy = cam_matrix[1, 1]  # focal length in pixels (Y)
    flow_count = 0

    log.info(f"Running Vision GPS ({args.mode} mode)")
    frames, detections_count, last_state = 0, 0, None

    try:
        while not shutdown[0]:
            t_start = time.time()
            frame, frame_time = camera.get_frame()
            if frame is None:
                time.sleep(0.01)
                continue
            # Skip if frame is the same one we already processed
            frame_age = time.perf_counter() - frame_time if frame_time else 0
            if frame_age > 0.5:
                continue  # frame older than 500ms, wait for fresh one

            dets, timing = detect(frame, clahe, detector, cam_matrix, dist_coeffs, marker_size, marker_sizes)
            timing["age"] = f"{frame_age*1000:.0f}"
            frames += 1
            state = None

            # Current grayscale for optical flow (detect() already computed it, but
            # we need it here — cheap enough to redo: ~4ms)
            cur_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            if dets:
                detections_count += 1
                imu = mavlink.attitude if mavlink else None
                state = estimate_position(dets, marker_map, last_state, cam_matrix, imu_attitude=imu)
                if state is not None:
                    last_state = state
                # Log raw data
                for d in dets:
                    sp = f"{state.n:.4f},{state.e:.4f},{state.d:.4f},{state.yaw:.2f}" if state else ",,,"
                    csv_file.write(
                        f"{frames},{time.time():.3f},{d.marker_id},"
                        f"{d.centroid[0]:.1f},{d.centroid[1]:.1f},"
                        f"{d.tvec[0]:.4f},{d.tvec[1]:.4f},{d.tvec[2]:.4f},"
                        f"{d.rvec[0]:.4f},{d.rvec[1]:.4f},{d.rvec[2]:.4f},"
                        f"{sp}\n")
                if frames % 30 == 0:
                    csv_file.flush()
            elif last_state is not None and prev_gray is not None and prev_features is not None:
                # No marker detection — use optical flow to interpolate position
                t_flow = time.perf_counter()
                new_pts, status, _ = cv2.calcOpticalFlowPyrLK(
                    prev_gray, cur_gray, prev_features, None,
                    winSize=(21, 21), maxLevel=2)
                if new_pts is not None and status is not None:
                    good = status.flatten() == 1
                    if np.sum(good) >= 4:
                        old_xy = prev_features.reshape(-1, 2)[good]
                        new_xy = new_pts.reshape(-1, 2)[good]
                        dx_px = float(np.median(new_xy[:, 0] - old_xy[:, 0]))
                        dy_px = float(np.median(new_xy[:, 1] - old_xy[:, 1]))

                        # Subtract gyro-induced pixel shift (rotation, not translation)
                        dt = 1.0 / max(1, cfg.get("camera", {}).get("fps", 30))
                        rates = mavlink.attitude_rate if mavlink else None
                        if rates:
                            # pitchspeed rotates image in X, rollspeed in Y
                            dx_px -= rates[1] * fx * dt  # pitch rate
                            dy_px -= rates[0] * fy * dt  # roll rate

                        # Pixel displacement → meters (h * px / focal_length)
                        height = abs(last_state.d) if abs(last_state.d) > 0.5 else 2.5
                        # Camera frame: cam_X = body_right (negated by R_CB), cam_Y = body_down
                        # R_CB = [[-1,0,0],[0,1,0],[0,0,-1]] maps cam→body
                        d_fwd = height * dx_px / fx     # cam_X pixel shift → forward (sign from R_CB)
                        d_right = height * dy_px / fy   # cam_Y pixel shift → right

                        # Rotate body displacement by yaw to NED
                        yaw_rad = np.radians(last_state.yaw)
                        cos_y, sin_y = np.cos(yaw_rad), np.sin(yaw_rad)
                        dn = d_fwd * cos_y - d_right * sin_y
                        de = d_fwd * sin_y + d_right * cos_y

                        state = DroneState(
                            n=last_state.n + dn,
                            e=last_state.e + de,
                            d=last_state.d,
                            yaw=last_state.yaw,
                            marker_ids=["flow"],
                            confidence=last_state.confidence * 0.5,
                            timestamp=time.time(),
                            marker_weights={"flow": 1.0})
                        last_state = state
                        flow_count += 1
                        timing["flow"] = f"{(time.perf_counter()-t_flow)*1000:.0f}"

            # Update optical flow features for next frame
            if prev_features is None or frames % flow_redetect_interval == 0:
                # Re-pick features periodically
                prev_features = cv2.goodFeaturesToTrack(
                    cur_gray, maxCorners=80, qualityLevel=0.05,
                    minDistance=15, blockSize=7)
            elif prev_gray is not None:
                # Track existing features to current frame
                new_pts, status, _ = cv2.calcOpticalFlowPyrLK(
                    prev_gray, cur_gray, prev_features, None,
                    winSize=(21, 21), maxLevel=2)
                if new_pts is not None and status is not None:
                    good = status.flatten() == 1
                    if np.sum(good) >= 4:
                        prev_features = new_pts[good].reshape(-1, 1, 2)
                    else:
                        prev_features = None
                else:
                    prev_features = None
            prev_gray = cur_gray

            if args.mode == "run" and state and mavlink:
                is_flow = not dets
                mavlink.send_gps_input(
                    north=state.n, east=state.e, down=state.d,
                    yaw_deg=state.yaw, origin_lat=origin_lat,
                    origin_lon=origin_lon, origin_alt=origin_alt,
                    confidence=state.confidence,
                    horiz_accuracy=0.30 if is_flow else 0.10)

            if args.mode in ("test", "run"):
                if dets:
                    d = dets[0]
                    t = R_CB @ d.tvec  # body frame: fwd, right, down
                    r = np.degrees(R_CB @ d.rvec)
                    print(f"\rtvec:({t[0]:+.2f},{t[1]:+.2f},{t[2]:+.2f}) "
                          f"rvec:({r[0]:+.1f},{r[1]:+.1f},{r[2]:+.1f})° ", end="")
                    if state:
                        print(f"pos:({state.n:+.2f},{state.e:+.2f},{state.d:.2f}) "
                              f"yaw:{state.yaw:+.1f} conf:{state.confidence:.2f}", end="  ")
                elif state and "flow" in (state.marker_ids or []):
                    print(f"\r[FLOW] pos:({state.n:+.2f},{state.e:+.2f},{state.d:.2f}) "
                          f"yaw:{state.yaw:+.1f} flow#{flow_count}" + " " * 20, end="")
                else:
                    print("\rNo markers" + " " * 60, end="")

            # Camera-observed angles: pitch/roll of marker from vertical in camera frame
            cam_angles = None
            body_tvec = None
            if dets:
                t = dets[0].tvec
                # Pitch: forward/back tilt = atan2(tx, tz), Roll: left/right = atan2(ty, tz)
                cam_pitch = np.degrees(np.arctan2(t[0], t[2]))
                cam_roll = np.degrees(np.arctan2(t[1], t[2]))
                cam_angles = (cam_pitch, cam_roll)
                body_tvec = R_CB @ t  # body frame: [fwd, right, down]
            imu = mavlink.attitude if mavlink else None
            fc_pos = mavlink.fc_position if mavlink else None
            body_offset = getattr(estimate_single, '_debug_body', None)
            server.update(state, frames, detections_count, frame, timing,
                         imu_attitude=imu, cam_angles=cam_angles, fc_position=fc_pos,
                         body_tvec=body_tvec, body_offset=body_offset)

            # No sleep — run as fast as possible. Detection (~120ms) is the bottleneck.
    finally:
        csv_file.close()
        log.info(f"Debug log saved to {csv_path}")
        camera.stop()
        if mavlink:
            mavlink.disconnect()
        server.stop()
        rate = detections_count / max(1, frames)
        log.info(f"Done: {frames} frames, {detections_count} detections ({rate:.0%})")


if __name__ == "__main__":
    main()
