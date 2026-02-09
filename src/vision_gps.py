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

Detection = namedtuple("Detection", "marker_id rvec tvec distance timestamp")
DroneState = namedtuple("DroneState", "x y z yaw marker_ids confidence timestamp")

# --- Config loaders ---

def load_camera_params(path):
    with open(path) as f:
        data = yaml.safe_load(f)
    cm = data["camera_matrix"]
    matrix = np.array(cm["data"] if isinstance(cm, dict) and "data" in cm else cm).reshape(3, 3)
    dc = data["distortion_coefficients"]
    coeffs = np.array(dc["data"] if isinstance(dc, dict) and "data" in dc else dc)
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
        ok, rvec, tvec = cv2.solvePnP(obj_pts, corners[i].reshape(-1, 2), cam_matrix, dist_coeffs)
        if ok:
            tvec = tvec.flatten()
            detections.append(Detection(int(mid), rvec.flatten(), tvec, float(np.linalg.norm(tvec)), ts))
    return detections, timing

# --- Position estimation ---

CAM_TO_BODY = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]], dtype=float)
BODY_TO_WORLD_BASE = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]], dtype=float)


def euler_from_rvec(rvec):
    rmat, _ = cv2.Rodrigues(rvec)
    sy = np.sqrt(rmat[0, 0]**2 + rmat[1, 0]**2)
    if sy > 1e-6:
        yaw = np.arctan2(rmat[1, 0], rmat[0, 0])
    else:
        yaw = 0.0
    return np.degrees(yaw)


def estimate_single(detection, marker_cfg):
    marker_in_body = CAM_TO_BODY @ detection.tvec
    yaw_camera = euler_from_rvec(detection.rvec)
    drone_yaw = -yaw_camera + marker_cfg["orientation"]
    yaw_rad = np.radians(drone_yaw)
    c, s = np.cos(yaw_rad), np.sin(yaw_rad)
    yaw_rot = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    b2w = yaw_rot @ BODY_TO_WORLD_BASE
    pos = marker_cfg["position"] - b2w @ marker_in_body
    return pos, drone_yaw


def estimate_position(detections, marker_map, last_state, alpha=0.7):
    valid = [(d, marker_map[str(d.marker_id)]) for d in detections if str(d.marker_id) in marker_map]
    if not valid:
        return last_state

    if len(valid) == 1:
        d, cfg = valid[0]
        pos, yaw = estimate_single(d, cfg)
        ids = [str(d.marker_id)]
        conf = 1.0
    else:
        positions, yaws, weights, ids = [], [], [], []
        for d, cfg in valid:
            pos, yaw = estimate_single(d, cfg)
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

    cam_matrix, dist_coeffs = load_camera_params(config_dir / "camera_params.yaml")
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
        mavlink.set_ekf_origin()

    server = None
    if args.mode == "stream":
        server = PositionServer(port=args.port)
        server.start()

    shutdown = [False]
    signal.signal(signal.SIGINT, lambda *_: shutdown.__setitem__(0, True))
    signal.signal(signal.SIGTERM, lambda *_: shutdown.__setitem__(0, True))

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
                state = estimate_position(dets, marker_map, last_state)
                last_state = state

            if args.mode == "run" and state and mavlink:
                mavlink.send_vision_position(
                    x=state.y, y=state.x, z=-state.z,  # ENU -> NED
                    yaw=np.radians(state.yaw), confidence=state.confidence)

            if args.mode == "test":
                if state:
                    print(f"\rPos: ({state.x:+.2f}, {state.y:+.2f}, {state.z:.2f})m | "
                          f"Yaw: {state.yaw:+.1f}° | Markers: {state.marker_ids} | "
                          f"Conf: {state.confidence:.2f}", end="")
                else:
                    print("\rNo markers detected" + " " * 40, end="")

            if args.mode == "stream" and server:
                server.update(state, frames, detections_count, frame, timing)

            elapsed = time.time() - t_start
            if elapsed < loop_period:
                time.sleep(loop_period - elapsed)
    finally:
        camera.stop()
        if mavlink:
            mavlink.disconnect()
        if server:
            server.stop()
        rate = detections_count / max(1, frames)
        log.info(f"Done: {frames} frames, {detections_count} detections ({rate:.0%})")


if __name__ == "__main__":
    main()
