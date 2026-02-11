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
DroneState = namedtuple("DroneState", "x y z yaw marker_ids confidence timestamp")

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
    return matrix, coeffs


def load_marker_map(path):
    with open(path) as f:
        data = yaml.safe_load(f)
    markers = {}
    for m in data.get("markers", []):
        markers[str(m["id"])] = {
            "position": np.array(m["position"]),
            "orientation": m.get("orientation", 0),
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
        self._lock = threading.Lock()
        self._running = False

    def start(self):
        self._cap = cv2.VideoCapture(self.device_id)
        if not self._cap.isOpened():
            log.error(f"Failed to open camera {self.device_id}")
            return False
        self._cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self._cap.set(cv2.CAP_PROP_FPS, self.fps)
        self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
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
            return self._frame.copy() if self._frame is not None else None

    def _capture_loop(self):
        while self._running and self._cap:
            ret, frame = self._cap.read()
            if ret:
                with self._lock:
                    self._frame = frame
            else:
                time.sleep(0.001)

# --- Detection ---

def create_detector(dictionary="DICT_4X4_50"):
    aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary))
    params = cv2.aruco.DetectorParameters()
    params.adaptiveThreshConstant = 7
    params.adaptiveThreshWinSizeMin = 3
    params.adaptiveThreshWinSizeMax = 23
    params.adaptiveThreshWinSizeStep = 10
    params.minMarkerPerimeterRate = 0.01
    params.maxMarkerPerimeterRate = 4.0
    params.polygonalApproxAccuracyRate = 0.05
    params.minCornerDistanceRate = 0.01
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_CONTOUR
    params.minOtsuStdDev = 5.0
    params.perspectiveRemovePixelPerCell = 4
    params.perspectiveRemoveIgnoredMarginPerCell = 0.13
    return cv2.aruco.ArucoDetector(aruco_dict, params)


# Persistent PnP state for temporal consistency (per marker ID)
_last_pnp = {}  # marker_id -> (rvec, tvec)


def detect(frame, clahe, detector, cam_matrix, dist_coeffs, marker_size):
    t0 = time.perf_counter()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    t1 = time.perf_counter()
    enhanced = clahe.apply(gray)
    t2 = time.perf_counter()
    processed = cv2.cvtColor(enhanced, cv2.COLOR_GRAY2BGR)
    t3 = time.perf_counter()

    try:
        corners, ids, _ = detector.detectMarkers(processed)
    except cv2.error as e:
        log.warning(f"Detection error: {e}")
        return [], {}

    t4 = time.perf_counter()
    timing = {
        "gray": f"{(t1-t0)*1000:.0f}",
        "clahe": f"{(t2-t1)*1000:.0f}",
        "bgr": f"{(t3-t2)*1000:.0f}",
        "detect": f"{(t4-t3)*1000:.0f}",
        "total": f"{(t4-t0)*1000:.0f}",
    }

    if ids is None:
        return [], timing

    half = marker_size / 2
    obj_pts = np.array([[-half, half, 0], [half, half, 0],
                         [half, -half, 0], [-half, -half, 0]], dtype=np.float32)
    ts = time.time()
    detections = []
    for i, mid in enumerate(ids.flatten()):
        corners_2d = corners[i].reshape(-1, 2)
        mid_int = int(mid)

        # Use previous frame's solution as initial guess (temporal consistency)
        # This keeps solvePnP near the correct solution frame-to-frame
        prev = _last_pnp.get(mid_int)
        if prev is not None:
            # Temporal consistency: start from previous frame's solution
            rvec_init, tvec_init = prev
            ok, rvec, tvec = cv2.solvePnP(
                obj_pts, corners_2d, cam_matrix, dist_coeffs,
                rvec=rvec_init.copy(), tvec=tvec_init.copy(),
                useExtrinsicGuess=True,
                flags=cv2.SOLVEPNP_ITERATIVE)
        else:
            # First detection: use IPPE_SQUARE (no initial guess needed)
            ok, rvec, tvec = cv2.solvePnP(
                obj_pts, corners_2d, cam_matrix, dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE)

        if ok:
            _last_pnp[mid_int] = (rvec.copy(), tvec.copy())
            tvec_flat = tvec.flatten()
            centroid = corners_2d.mean(axis=0)
            detections.append(Detection(
                mid_int, rvec.flatten(), tvec_flat,
                float(np.linalg.norm(tvec_flat)), ts, centroid))

    return detections, timing

# --- Position estimation ---

# Camera-to-body fixed rotation (physical mounting, upward-facing camera)
# Camera top → drone right: cam_X → body_Y, cam_Y → body_X, cam_Z → body_Z
R_CB = np.array([[0, 1, 0], [1, 0, 0], [0, 0, 1]], dtype=float)


def marker_to_world_rotation(orientation_deg):
    """Fixed rotation from marker frame to world (ENU) for ceiling-mounted marker.
    Marker Z+ (out of face) points down → world -Z."""
    theta = np.radians(orientation_deg)
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, -1]])


def estimate_single(detection, marker_cfg, imu_attitude=None):
    # Rotation from marker frame to camera frame (from solvePnP)
    R_cm, _ = cv2.Rodrigues(detection.rvec)

    # Camera position in marker frame: correct inverse of solvePnP transform
    cam_in_marker = (-R_cm.T @ detection.tvec.reshape(3, 1)).flatten()

    # Fixed marker-to-world rotation (marker is glued to ceiling, known pose)
    R_mw = marker_to_world_rotation(marker_cfg["orientation"])

    # Drone position in world = marker_world_pos + camera_offset_in_world
    pos = marker_cfg["position"] + R_mw @ cam_in_marker

    # Yaw from full rotation chain: body→world = R_mw @ R_cm^T @ R_CB^T
    R_bw = R_mw @ R_cm.T @ R_CB.T
    vision_yaw = np.degrees(np.arctan2(R_bw[1, 0], R_bw[0, 0]))

    if imu_attitude is not None:
        drone_yaw = np.degrees(imu_attitude[2])
    else:
        drone_yaw = vision_yaw

    return pos, drone_yaw


def estimate_position(detections, marker_map, last_state, cam_matrix, alpha=0.3, imu_attitude=None):
    valid = [(d, marker_map[str(d.marker_id)]) for d in detections if str(d.marker_id) in marker_map]
    if not valid:
        return last_state

    if len(valid) == 1:
        d, cfg = valid[0]
        pos, yaw = estimate_single(d, cfg, imu_attitude)
        ids = [str(d.marker_id)]
        conf = 1.0
    else:
        positions, yaws, weights, ids = [], [], [], []
        for d, cfg in valid:
            pos, yaw = estimate_single(d, cfg, imu_attitude)
            w = 1.0 / (d.distance + 0.1)
            positions.append(pos * w)
            yaws.append((yaw, w))
            weights.append(w)
            ids.append(str(d.marker_id))
        tw = sum(weights)
        pos = sum(positions) / tw
        sin_sum = sum(np.sin(np.radians(y)) * w for y, w in yaws)
        cos_sum = sum(np.cos(np.radians(y)) * w for y, w in yaws)
        yaw = np.degrees(np.arctan2(sin_sum, cos_sum))
        stds = np.std([p / w for p, w in zip(positions, weights)], axis=0).mean()
        conf = min(1.0, len(valid) / 3.0) * np.exp(-stds)

    # Low-pass filter
    if last_state is not None:
        prev_pos = np.array([last_state.x, last_state.y, last_state.z])
        pos = alpha * pos + (1 - alpha) * prev_pos
        yaw_diff = yaw - last_state.yaw
        if yaw_diff > 180: yaw_diff -= 360
        elif yaw_diff < -180: yaw_diff += 360
        yaw = last_state.yaw + alpha * yaw_diff

    return DroneState(float(pos[0]), float(pos[1]), float(pos[2]),
                      float(yaw), ids, float(conf), time.time())

# --- HTTP Position Server ---

class PositionServer:
    def __init__(self, port=8001):
        self.port = port
        self.state = None
        self.frame_jpeg = None
        self.stats = {"frames": 0, "detections": 0, "start": time.time(), "timing": {}}
        self._lock = threading.Lock()
        self._server = None

    def update(self, state, frames, detections, debug_frame=None, timing=None):
        with self._lock:
            self.state = state
            self.stats["frames"] = frames
            self.stats["detections"] = detections
            if timing:
                self.stats["timing"] = timing
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
                base.update({"x": round(s.x, 3), "y": round(s.y, 3), "z": round(s.z, 3),
                             "yaw": round(s.yaw, 1), "marker_ids": s.marker_ids,
                             "confidence": round(s.confidence, 2)})
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
                elif self.path == "/debug-frame":
                    with ref._lock:
                        jpg = ref.frame_jpeg
                    if jpg:
                        self.send_response(200)
                        self.send_header("Content-Type", "image/jpeg")
                        self.send_header("Cache-Control", "no-cache")
                        self.end_headers()
                        self.wfile.write(jpg)
                    else:
                        self.send_error(503)
                else:
                    self.send_error(404)
        self._server = HTTPServer(("0.0.0.0", self.port), H)
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
    cam_matrix, dist_coeffs = load_camera_params(
        config_dir / "camera_params.yaml", target_width=cam_w, target_height=cam_h)
    marker_map = load_marker_map(config_dir / "marker_map.yaml")
    aruco_cfg = cfg.get("aruco", {})
    marker_size = aruco_cfg.get("marker_size_m", 0.18)

    camera = Camera(
        device_id=cfg.get("camera", {}).get("device_id", 0),
        width=cfg.get("camera", {}).get("width", 1280),
        height=cfg.get("camera", {}).get("height", 720),
        fps=cfg.get("camera", {}).get("fps", 30),
    )
    if not camera.start():
        sys.exit(1)

    clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8))
    detector = create_detector(aruco_cfg.get("dictionary", "DICT_4X4_50"))
    loop_period = 1.0 / cfg.get("control", {}).get("loop_rate_hz", 20)

    mavlink = None
    if args.mode == "run":
        from .mavlink_bridge import MAVLinkBridge
        serial_cfg = cfg.get("serial", {})
        mavlink = MAVLinkBridge(serial_cfg.get("port", "/dev/serial0"), serial_cfg.get("baud", 921600))
        if not mavlink.connect(timeout=30.0):
            log.error("Failed to connect to flight controller")
            sys.exit(1)
        origin = cfg.get("ekf_origin", {})
        origin_lat = origin.get("lat", 52.2297)
        origin_lon = origin.get("lon", 21.0122)
        origin_alt = origin.get("alt", 100.0)
        gps_mode = cfg.get("gps_emulation", True)
        if gps_mode:
            log.info("Using GPS emulation mode")
        else:
            mavlink.configure_origin(origin_lat, origin_lon, origin_alt)
            log.info("Using VISION_POSITION_ESTIMATE mode")

    server = PositionServer(port=args.port)
    server.start()

    shutdown = [False]
    signal.signal(signal.SIGINT, lambda *_: shutdown.__setitem__(0, True))
    signal.signal(signal.SIGTERM, lambda *_: shutdown.__setitem__(0, True))

    # CSV log for debugging position estimation
    csv_path = Path(__file__).parent.parent / "debug_log.csv"
    csv_file = open(csv_path, "w")
    csv_file.write("frame,time,marker_id,cx,cy,tvec_x,tvec_y,tvec_z,"
                   "rvec_x,rvec_y,rvec_z,pos_x,pos_y,pos_z,yaw,imu_roll,imu_pitch,imu_yaw\n")
    log.info(f"Debug CSV: {csv_path}")

    log.info(f"Running Vision GPS ({args.mode} mode)")
    frames, detections_count, last_state = 0, 0, None

    try:
        while not shutdown[0]:
            t_start = time.time()
            frame = camera.get_frame()
            if frame is None:
                time.sleep(0.01)
                continue

            dets, timing = detect(frame, clahe, detector, cam_matrix, dist_coeffs, marker_size)
            frames += 1
            state = None

            if dets:
                detections_count += 1
                imu = mavlink.attitude if mavlink else None
                state = estimate_position(dets, marker_map, last_state, cam_matrix, imu_attitude=imu)
                if state is not None:
                    last_state = state
                # Log raw data
                for d in dets:
                    ir = f"{imu[0]:.4f},{imu[1]:.4f},{imu[2]:.4f}" if imu else ",,"
                    sp = f"{state.x:.4f},{state.y:.4f},{state.z:.4f},{state.yaw:.2f}" if state else ",,,"
                    csv_file.write(
                        f"{frames},{time.time():.3f},{d.marker_id},"
                        f"{d.centroid[0]:.1f},{d.centroid[1]:.1f},"
                        f"{d.tvec[0]:.4f},{d.tvec[1]:.4f},{d.tvec[2]:.4f},"
                        f"{d.rvec[0]:.4f},{d.rvec[1]:.4f},{d.rvec[2]:.4f},"
                        f"{sp},{ir}\n")
                    csv_file.flush()

            if args.mode == "run" and state and mavlink:
                if gps_mode:
                    mavlink.send_gps_input(
                        x_enu=state.x, y_enu=state.y, z_enu=state.z,
                        yaw_deg=state.yaw, origin_lat=origin_lat,
                        origin_lon=origin_lon, origin_alt=origin_alt,
                        confidence=state.confidence)
                else:
                    mavlink.send_vision_position(
                        x=state.y, y=state.x, z=-state.z,  # ENU -> NED
                        yaw=np.radians(state.yaw), confidence=state.confidence)

            if args.mode in ("test", "run"):
                if dets:
                    d = dets[0]
                    t = d.tvec
                    r = np.degrees(d.rvec)
                    print(f"\rtvec:({t[0]:+.2f},{t[1]:+.2f},{t[2]:+.2f}) "
                          f"rvec:({r[0]:+.1f},{r[1]:+.1f},{r[2]:+.1f})° ", end="")
                    if state:
                        print(f"pos:({state.x:+.2f},{state.y:+.2f},{state.z:.2f}) "
                              f"yaw:{state.yaw:+.1f} conf:{state.confidence:.2f}", end="  ")
                else:
                    print("\rNo markers" + " " * 60, end="")

            server.update(state, frames, detections_count, frame, timing)

            elapsed = time.time() - t_start
            if elapsed < loop_period:
                time.sleep(loop_period - elapsed)
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
