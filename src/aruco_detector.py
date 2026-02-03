"""
ArUco Marker Detection Module

Detects ArUco markers in camera frames and estimates their 3D pose
relative to the camera using OpenCV's ArUco module.
"""

import cv2
import numpy as np
from dataclasses import dataclass
from typing import List, Optional, Dict, Tuple, Union
import logging
import threading
import time

from .camera_calibration import CameraCalibration

logger = logging.getLogger(__name__)

# ArUco dictionary mapping
ARUCO_DICTIONARIES = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
}


@dataclass
class MarkerDetection:
    """Represents a detected ArUco marker with pose information."""

    marker_id: int
    corners: np.ndarray           # 4 corner points in image
    rvec: np.ndarray              # Rotation vector (Rodrigues)
    tvec: np.ndarray              # Translation vector (x, y, z in meters)
    distance: float               # Distance to marker in meters
    timestamp: float              # Detection timestamp

    @property
    def position(self) -> Tuple[float, float, float]:
        """Get marker position (x, y, z) relative to camera."""
        return (
            float(self.tvec[0]),
            float(self.tvec[1]),
            float(self.tvec[2])
        )

    @property
    def rotation_matrix(self) -> np.ndarray:
        """Convert rotation vector to rotation matrix."""
        rmat, _ = cv2.Rodrigues(self.rvec)
        return rmat

    def get_euler_angles(self) -> Tuple[float, float, float]:
        """
        Get Euler angles (roll, pitch, yaw) in degrees.

        Returns:
            Tuple of (roll, pitch, yaw) in degrees
        """
        rmat = self.rotation_matrix

        # Extract Euler angles from rotation matrix
        sy = np.sqrt(rmat[0, 0] ** 2 + rmat[1, 0] ** 2)

        if sy > 1e-6:
            roll = np.arctan2(rmat[2, 1], rmat[2, 2])
            pitch = np.arctan2(-rmat[2, 0], sy)
            yaw = np.arctan2(rmat[1, 0], rmat[0, 0])
        else:
            roll = np.arctan2(-rmat[1, 2], rmat[1, 1])
            pitch = np.arctan2(-rmat[2, 0], sy)
            yaw = 0

        return (
            np.degrees(roll),
            np.degrees(pitch),
            np.degrees(yaw)
        )


@dataclass
class DiamondDetection:
    """Represents a detected ArUco Diamond marker with pose information."""

    diamond_id: Tuple[int, int, int, int]  # (id0, id1, id2, id3)
    corners: np.ndarray           # 4 corner points in image
    rvec: np.ndarray              # Rotation vector (Rodrigues)
    tvec: np.ndarray              # Translation vector (x, y, z in meters)
    distance: float               # Distance to marker in meters
    timestamp: float              # Detection timestamp

    @property
    def id_string(self) -> str:
        """Get diamond ID as string (e.g., '0_1_2_3')."""
        return "_".join(str(i) for i in self.diamond_id)

    @property
    def position(self) -> Tuple[float, float, float]:
        """Get marker position (x, y, z) relative to camera."""
        return (
            float(self.tvec[0]),
            float(self.tvec[1]),
            float(self.tvec[2])
        )

    @property
    def rotation_matrix(self) -> np.ndarray:
        """Convert rotation vector to rotation matrix."""
        rmat, _ = cv2.Rodrigues(self.rvec)
        return rmat

    def get_euler_angles(self) -> Tuple[float, float, float]:
        """
        Get Euler angles (roll, pitch, yaw) in degrees.

        Returns:
            Tuple of (roll, pitch, yaw) in degrees
        """
        rmat = self.rotation_matrix

        # Extract Euler angles from rotation matrix
        sy = np.sqrt(rmat[0, 0] ** 2 + rmat[1, 0] ** 2)

        if sy > 1e-6:
            roll = np.arctan2(rmat[2, 1], rmat[2, 2])
            pitch = np.arctan2(-rmat[2, 0], sy)
            yaw = np.arctan2(rmat[1, 0], rmat[0, 0])
        else:
            roll = np.arctan2(-rmat[1, 2], rmat[1, 1])
            pitch = np.arctan2(-rmat[2, 0], sy)
            yaw = 0

        return (
            np.degrees(roll),
            np.degrees(pitch),
            np.degrees(yaw)
        )


class ArucoDetector:
    """
    ArUco marker detector with pose estimation.

    Handles camera initialization, marker detection, and pose estimation
    using calibrated camera parameters.
    """

    def __init__(
        self,
        camera_id: int = 0,
        camera_matrix: Optional[np.ndarray] = None,
        dist_coeffs: Optional[np.ndarray] = None,
        dictionary: str = "DICT_6X6_250",
        marker_size_m: float = 0.20,
        resolution: Tuple[int, int] = (640, 480),
        fps: int = 30
    ):
        """
        Initialize the ArUco detector.

        Args:
            camera_id: Camera device ID
            camera_matrix: 3x3 camera intrinsic matrix
            dist_coeffs: Distortion coefficients
            dictionary: ArUco dictionary name
            marker_size_m: Physical marker size in meters
            resolution: Camera resolution (width, height)
            fps: Target frames per second
        """
        self.camera_id = camera_id
        self.marker_size_m = marker_size_m
        self.resolution = resolution
        self.target_fps = fps

        # Camera parameters
        if camera_matrix is not None:
            self.camera_matrix = camera_matrix
        else:
            # Default camera matrix (should be calibrated!)
            logger.warning("Using default camera matrix - calibration recommended!")
            self.camera_matrix = np.array([
                [resolution[0], 0, resolution[0] / 2],
                [0, resolution[0], resolution[1] / 2],
                [0, 0, 1]
            ], dtype=np.float32)

        if dist_coeffs is not None:
            self.dist_coeffs = dist_coeffs
        else:
            self.dist_coeffs = np.zeros(5, dtype=np.float32)

        # ArUco setup
        if dictionary not in ARUCO_DICTIONARIES:
            raise ValueError(f"Unknown dictionary: {dictionary}")

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(
            ARUCO_DICTIONARIES[dictionary]
        )
        self.aruco_params = cv2.aruco.DetectorParameters()

        # Optimize detection parameters for robustness
        self.aruco_params.adaptiveThreshConstant = 7
        self.aruco_params.minMarkerPerimeterRate = 0.02  # Detect smaller markers
        self.aruco_params.maxMarkerPerimeterRate = 4.0
        self.aruco_params.polygonalApproxAccuracyRate = 0.05  # More forgiving shape
        self.aruco_params.minCornerDistanceRate = 0.02  # Allow closer corners
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.aruco_params.cornerRefinementWinSize = 5
        self.aruco_params.cornerRefinementMaxIterations = 50

        # Create detector
        self.detector = cv2.aruco.ArucoDetector(
            self.aruco_dict, self.aruco_params
        )

        # Camera capture
        self.cap: Optional[cv2.VideoCapture] = None
        self._running = False
        self._frame_lock = threading.Lock()
        self._latest_frame: Optional[np.ndarray] = None
        self._capture_thread: Optional[threading.Thread] = None

        # Statistics
        self.frames_processed = 0
        self.detection_rate = 0.0
        self._last_fps_time = time.time()
        self._fps_frame_count = 0

    def start(self) -> bool:
        """
        Start the camera capture.

        Returns:
            True if camera started successfully
        """
        if self._running:
            return True

        self.cap = cv2.VideoCapture(self.camera_id)

        if not self.cap.isOpened():
            logger.error(f"Failed to open camera {self.camera_id}")
            return False

        # Configure camera
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])
        self.cap.set(cv2.CAP_PROP_FPS, self.target_fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Minimize latency

        self._running = True

        # Start capture thread
        self._capture_thread = threading.Thread(
            target=self._capture_loop,
            daemon=True
        )
        self._capture_thread.start()

        logger.info(
            f"Camera {self.camera_id} started at "
            f"{self.resolution[0]}x{self.resolution[1]}"
        )
        return True

    def stop(self):
        """Stop the camera capture."""
        self._running = False

        if self._capture_thread is not None:
            self._capture_thread.join(timeout=1.0)
            self._capture_thread = None

        if self.cap is not None:
            self.cap.release()
            self.cap = None

        logger.info("Camera stopped")

    def _capture_loop(self):
        """Background thread for continuous frame capture."""
        while self._running and self.cap is not None:
            ret, frame = self.cap.read()
            if ret:
                with self._frame_lock:
                    self._latest_frame = frame
            else:
                time.sleep(0.001)

    def get_frame(self) -> Optional[np.ndarray]:
        """
        Get the latest camera frame.

        Returns:
            Frame as numpy array or None if no frame available
        """
        with self._frame_lock:
            if self._latest_frame is not None:
                return self._latest_frame.copy()
        return None

    def detect(
        self,
        frame: Optional[np.ndarray] = None
    ) -> List[MarkerDetection]:
        """
        Detect ArUco markers in frame and estimate poses.

        Args:
            frame: Input frame (BGR). If None, captures from camera.

        Returns:
            List of detected markers with pose information
        """
        if frame is None:
            frame = self.get_frame()
            if frame is None:
                return []

        timestamp = time.time()
        detections: List[MarkerDetection] = []

        # Detect markers
        corners, ids, rejected = self.detector.detectMarkers(frame)

        if ids is None or len(ids) == 0:
            return detections

        # Estimate pose for each marker
        for i, marker_id in enumerate(ids.flatten()):
            marker_corners = corners[i]

            # Estimate pose using solvePnP
            obj_points = np.array([
                [-self.marker_size_m / 2, self.marker_size_m / 2, 0],
                [self.marker_size_m / 2, self.marker_size_m / 2, 0],
                [self.marker_size_m / 2, -self.marker_size_m / 2, 0],
                [-self.marker_size_m / 2, -self.marker_size_m / 2, 0]
            ], dtype=np.float32)

            success, rvec, tvec = cv2.solvePnP(
                obj_points,
                marker_corners.reshape(-1, 2),
                self.camera_matrix,
                self.dist_coeffs
            )

            if success:
                tvec = tvec.flatten()
                rvec = rvec.flatten()
                distance = np.linalg.norm(tvec)

                detection = MarkerDetection(
                    marker_id=int(marker_id),
                    corners=marker_corners,
                    rvec=rvec,
                    tvec=tvec,
                    distance=distance,
                    timestamp=timestamp
                )
                detections.append(detection)

        # Update statistics
        self.frames_processed += 1
        self._fps_frame_count += 1

        current_time = time.time()
        if current_time - self._last_fps_time >= 1.0:
            self.detection_rate = self._fps_frame_count / (
                current_time - self._last_fps_time
            )
            self._fps_frame_count = 0
            self._last_fps_time = current_time

        return detections

    def detect_single(
        self,
        frame: Optional[np.ndarray] = None,
        preferred_id: Optional[int] = None
    ) -> Optional[MarkerDetection]:
        """
        Detect a single marker (closest or preferred ID).

        Args:
            frame: Input frame
            preferred_id: Prefer marker with this ID if visible

        Returns:
            Single marker detection or None
        """
        detections = self.detect(frame)

        if not detections:
            return None

        # Return preferred ID if found
        if preferred_id is not None:
            for d in detections:
                if d.marker_id == preferred_id:
                    return d

        # Return closest marker
        return min(detections, key=lambda d: d.distance)

    def draw_detections(
        self,
        frame: np.ndarray,
        detections: List[MarkerDetection],
        draw_axes: bool = True,
        axis_length: float = 0.1
    ) -> np.ndarray:
        """
        Draw detected markers and their poses on frame.

        Args:
            frame: Input frame
            detections: List of detections to draw
            draw_axes: Whether to draw coordinate axes
            axis_length: Length of axes in meters

        Returns:
            Frame with drawings
        """
        output = frame.copy()

        for detection in detections:
            # Draw marker outline
            corners = detection.corners.reshape(-1, 2).astype(int)
            cv2.polylines(
                output, [corners], True, (0, 255, 0), 2
            )

            # Draw marker ID
            center = corners.mean(axis=0).astype(int)
            cv2.putText(
                output,
                f"ID:{detection.marker_id}",
                (center[0] - 30, center[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (0, 255, 0), 2
            )

            # Draw distance
            cv2.putText(
                output,
                f"{detection.distance:.2f}m",
                (center[0] - 30, center[1] + 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (255, 255, 0), 2
            )

            # Draw coordinate axes
            if draw_axes:
                cv2.drawFrameAxes(
                    output,
                    self.camera_matrix,
                    self.dist_coeffs,
                    detection.rvec,
                    detection.tvec,
                    axis_length
                )

        # Draw FPS
        cv2.putText(
            output,
            f"FPS: {self.detection_rate:.1f}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7, (0, 255, 255), 2
        )

        return output

    @classmethod
    def from_config(
        cls,
        config: Dict,
        calibration_path: str
    ) -> 'ArucoDetector':
        """
        Create detector from configuration dictionary.

        Args:
            config: Configuration dictionary
            calibration_path: Path to camera calibration file

        Returns:
            Configured ArucoDetector instance
        """
        # Load camera calibration
        camera_matrix, dist_coeffs = CameraCalibration.load_calibration(
            calibration_path
        )

        return cls(
            camera_id=config.get('camera', {}).get('device_id', 0),
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs,
            dictionary=config.get('aruco', {}).get('dictionary', 'DICT_6X6_250'),
            marker_size_m=config.get('aruco', {}).get('marker_size_m', 0.20),
            resolution=(
                config.get('camera', {}).get('width', 640),
                config.get('camera', {}).get('height', 480)
            ),
            fps=config.get('camera', {}).get('fps', 30)
        )


class CharucoDetector(ArucoDetector):
    """
    ChArUco board detector with sub-pixel pose accuracy.

    Extends ArucoDetector to detect ChArUco boards (ArUco + chessboard).
    Uses chessboard corner interpolation for more accurate pose estimation
    than plain ArUco markers.
    """

    def __init__(
        self,
        boards: dict,
        camera_id: int = 0,
        camera_matrix: Optional[np.ndarray] = None,
        dist_coeffs: Optional[np.ndarray] = None,
        dictionary: str = "DICT_6X6_250",
        resolution: tuple = (640, 480),
        fps: int = 30
    ):
        """
        Args:
            boards: Dict mapping board_id -> {squares_x, squares_y,
                    square_size_m, marker_size_m, marker_ids}
            camera_id: Camera device ID
            camera_matrix: Camera intrinsic matrix
            dist_coeffs: Distortion coefficients
            dictionary: ArUco dictionary name
            resolution: Camera resolution
            fps: Target FPS
        """
        # Initialize parent (marker_size_m is not used for charuco)
        super().__init__(
            camera_id=camera_id,
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs,
            dictionary=dictionary,
            marker_size_m=0.0,
            resolution=resolution,
            fps=fps
        )

        # Build CharucoBoard objects
        self.charuco_boards = {}
        for board_id, cfg in boards.items():
            # Calculate marker IDs
            if 'marker_ids' in cfg:
                ids = np.array(cfg['marker_ids'], dtype=np.int32)
            else:
                markers_per_board = (cfg['squares_x'] * cfg['squares_y']) // 2
                offset = board_id * markers_per_board
                ids = np.arange(offset, offset + markers_per_board, dtype=np.int32)

            # Create board with IDs (5th parameter for OpenCV 4.x)
            board = cv2.aruco.CharucoBoard(
                (cfg['squares_x'], cfg['squares_y']),
                cfg['square_size_m'],
                cfg['marker_size_m'],
                self.aruco_dict,
                ids
            )
            board.setLegacyPattern(False)

            self.charuco_boards[board_id] = board

        # Create CharucoDetector for each board
        self.charuco_detectors = {}
        charuco_params = cv2.aruco.CharucoParameters()
        for board_id, board in self.charuco_boards.items():
            self.charuco_detectors[board_id] = cv2.aruco.CharucoDetector(
                board, charuco_params, self.aruco_params
            )

    def detect(self, frame: Optional[np.ndarray] = None) -> List[MarkerDetection]:
        """
        Detect ChArUco boards and estimate pose with sub-pixel accuracy.

        Returns MarkerDetection objects where marker_id is the board_id
        and pose comes from ChArUco corner interpolation.
        """
        if frame is None:
            frame = self.get_frame()
            if frame is None:
                return []

        timestamp = time.time()
        detections: List[MarkerDetection] = []

        for board_id, charuco_det in self.charuco_detectors.items():
            board = self.charuco_boards[board_id]

            charuco_corners, charuco_ids, marker_corners, marker_ids = \
                charuco_det.detectBoard(frame)

            if charuco_corners is not None and len(charuco_corners) >= 4:
                # Estimate pose from ChArUco corners (sub-pixel accuracy)
                success, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
                    charuco_corners, charuco_ids, board,
                    self.camera_matrix, self.dist_coeffs,
                    None, None
                )

                if success:
                    tvec = tvec.flatten()
                    rvec = rvec.flatten()
                    distance = np.linalg.norm(tvec)

                    detection = MarkerDetection(
                        marker_id=int(board_id),
                        corners=marker_corners[0] if marker_corners is not None and len(marker_corners) > 0 else np.zeros((1, 4, 2)),
                        rvec=rvec,
                        tvec=tvec,
                        distance=distance,
                        timestamp=timestamp
                    )
                    detections.append(detection)

        # Update statistics
        self.frames_processed += 1
        self._fps_frame_count += 1
        current_time = time.time()
        if current_time - self._last_fps_time >= 1.0:
            self.detection_rate = self._fps_frame_count / (
                current_time - self._last_fps_time
            )
            self._fps_frame_count = 0
            self._last_fps_time = current_time

        return detections

    @classmethod
    def from_config(
        cls,
        config: dict,
        calibration_path: str
    ) -> 'CharucoDetector':
        """Create CharucoDetector from config with charuco section."""
        camera_matrix, dist_coeffs = CameraCalibration.load_calibration(
            calibration_path
        )

        charuco_config = config.get('charuco', {})
        boards = {}
        for board_cfg in charuco_config.get('boards', []):
            board_id = board_cfg['id']
            boards[board_id] = {
                'squares_x': board_cfg.get('squares_x', 5),
                'squares_y': board_cfg.get('squares_y', 5),
                'square_size_m': board_cfg.get('square_size_m', 0.04),
                'marker_size_m': board_cfg.get('marker_size_m', 0.03),
            }

        return cls(
            boards=boards,
            camera_id=config.get('camera', {}).get('device_id', 0),
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs,
            dictionary=config.get('aruco', {}).get('dictionary', 'DICT_6X6_250'),
            resolution=(
                config.get('camera', {}).get('width', 640),
                config.get('camera', {}).get('height', 480)
            ),
            fps=config.get('camera', {}).get('fps', 30)
        )


class DiamondDetector(ArucoDetector):
    """
    ArUco Diamond marker detector with pose estimation.

    Detects ArUco Diamond markers (4 ArUco markers in a diamond pattern)
    using detectCharucoDiamond. Provides better robustness than single
    markers due to redundancy.

    Diamond structure:
            [id0]
         [id3]  [id1]
            [id2]
    """

    def __init__(
        self,
        camera_id: int = 0,
        camera_matrix: Optional[np.ndarray] = None,
        dist_coeffs: Optional[np.ndarray] = None,
        dictionary: str = "DICT_4X4_50",
        square_size_m: float = 0.20,
        marker_size_m: float = 0.10,
        resolution: Tuple[int, int] = (640, 480),
        fps: int = 30
    ):
        """
        Initialize the Diamond detector.

        Args:
            camera_id: Camera device ID
            camera_matrix: 3x3 camera intrinsic matrix
            dist_coeffs: Distortion coefficients
            dictionary: ArUco dictionary name
            square_size_m: Size of center square in meters
            marker_size_m: Size of each ArUco marker in meters
            resolution: Camera resolution (width, height)
            fps: Target frames per second
        """
        # Initialize parent with marker_size for internal use
        super().__init__(
            camera_id=camera_id,
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs,
            dictionary=dictionary,
            marker_size_m=marker_size_m,
            resolution=resolution,
            fps=fps
        )

        self.square_size_m = square_size_m
        self.diamond_marker_size_m = marker_size_m

    def detect(
        self,
        frame: Optional[np.ndarray] = None
    ) -> List[DiamondDetection]:
        """
        Detect ArUco Diamond markers in frame and estimate poses.

        Args:
            frame: Input frame (BGR). If None, captures from camera.

        Returns:
            List of detected diamond markers with pose information
        """
        if frame is None:
            frame = self.get_frame()
            if frame is None:
                return []

        timestamp = time.time()
        detections: List[DiamondDetection] = []

        # First detect individual ArUco markers
        corners, ids, rejected = self.detector.detectMarkers(frame)

        if ids is None or len(ids) < 4:
            # Need at least 4 markers for a diamond
            return detections

        # Detect diamond markers
        diamond_corners, diamond_ids = cv2.aruco.detectCharucoDiamond(
            frame,
            corners,
            ids,
            self.square_size_m / self.diamond_marker_size_m,
            cameraMatrix=self.camera_matrix,
            distCoeffs=self.dist_coeffs
        )

        if diamond_ids is None or len(diamond_ids) == 0:
            return detections

        # Estimate pose for each diamond
        for i, diamond_id in enumerate(diamond_ids):
            diamond_id_tuple = tuple(int(x) for x in diamond_id.flatten())
            diamond_corner = diamond_corners[i]

            # Diamond corners represent the 4 corners of the center square
            # Use solvePnP with the square corners
            half_size = self.square_size_m / 2
            obj_points = np.array([
                [-half_size, half_size, 0],   # Top-left
                [half_size, half_size, 0],    # Top-right
                [half_size, -half_size, 0],   # Bottom-right
                [-half_size, -half_size, 0]   # Bottom-left
            ], dtype=np.float32)

            success, rvec, tvec = cv2.solvePnP(
                obj_points,
                diamond_corner.reshape(-1, 2),
                self.camera_matrix,
                self.dist_coeffs
            )

            if success:
                tvec = tvec.flatten()
                rvec = rvec.flatten()
                distance = np.linalg.norm(tvec)

                detection = DiamondDetection(
                    diamond_id=diamond_id_tuple,
                    corners=diamond_corner,
                    rvec=rvec,
                    tvec=tvec,
                    distance=distance,
                    timestamp=timestamp
                )
                detections.append(detection)

        # Update statistics
        self.frames_processed += 1
        self._fps_frame_count += 1

        current_time = time.time()
        if current_time - self._last_fps_time >= 1.0:
            self.detection_rate = self._fps_frame_count / (
                current_time - self._last_fps_time
            )
            self._fps_frame_count = 0
            self._last_fps_time = current_time

        return detections

    def draw_detections(
        self,
        frame: np.ndarray,
        detections: List[DiamondDetection],
        draw_axes: bool = True,
        axis_length: float = 0.1
    ) -> np.ndarray:
        """
        Draw detected diamond markers and their poses on frame.

        Args:
            frame: Input frame
            detections: List of detections to draw
            draw_axes: Whether to draw coordinate axes
            axis_length: Length of axes in meters

        Returns:
            Frame with drawings
        """
        output = frame.copy()

        for detection in detections:
            # Draw diamond outline
            corners = detection.corners.reshape(-1, 2).astype(int)
            cv2.polylines(
                output, [corners], True, (0, 255, 0), 2
            )

            # Draw diamond ID
            center = corners.mean(axis=0).astype(int)
            cv2.putText(
                output,
                f"D:{detection.id_string}",
                (center[0] - 50, center[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (0, 255, 0), 2
            )

            # Draw distance
            cv2.putText(
                output,
                f"{detection.distance:.2f}m",
                (center[0] - 30, center[1] + 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (255, 255, 0), 2
            )

            # Draw coordinate axes
            if draw_axes:
                cv2.drawFrameAxes(
                    output,
                    self.camera_matrix,
                    self.dist_coeffs,
                    detection.rvec,
                    detection.tvec,
                    axis_length
                )

        # Draw FPS
        cv2.putText(
            output,
            f"FPS: {self.detection_rate:.1f}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7, (0, 255, 255), 2
        )

        return output

    @classmethod
    def from_config(
        cls,
        config: Dict,
        calibration_path: str
    ) -> 'DiamondDetector':
        """
        Create detector from configuration dictionary.

        Args:
            config: Configuration dictionary
            calibration_path: Path to camera calibration file

        Returns:
            Configured DiamondDetector instance
        """
        # Load camera calibration
        camera_matrix, dist_coeffs = CameraCalibration.load_calibration(
            calibration_path
        )

        diamond_config = config.get('diamond', {})

        return cls(
            camera_id=config.get('camera', {}).get('device_id', 0),
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs,
            dictionary=config.get('aruco', {}).get('dictionary', 'DICT_4X4_50'),
            square_size_m=diamond_config.get('square_size_m', 0.20),
            marker_size_m=diamond_config.get('marker_size_m', 0.10),
            resolution=(
                config.get('camera', {}).get('width', 640),
                config.get('camera', {}).get('height', 480)
            ),
            fps=config.get('camera', {}).get('fps', 30)
        )


def test_detection():
    """Simple test function for marker detection."""
    logging.basicConfig(level=logging.INFO)

    detector = ArucoDetector(
        camera_id=0,
        marker_size_m=0.15,
        resolution=(640, 480)
    )

    if not detector.start():
        print("Failed to start camera")
        return

    print("Press 'q' to quit")

    try:
        while True:
            frame = detector.get_frame()
            if frame is None:
                continue

            detections = detector.detect(frame)
            display = detector.draw_detections(frame, detections)

            for d in detections:
                x, y, z = d.position
                roll, pitch, yaw = d.get_euler_angles()
                print(
                    f"ID:{d.marker_id} pos=({x:.3f}, {y:.3f}, {z:.3f}) "
                    f"yaw={yaw:.1f}"
                )

            cv2.imshow("ArUco Detection", display)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        detector.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    test_detection()
