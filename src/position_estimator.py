"""
Position Estimator Module

Transforms ArUco marker detections into world-frame drone position estimates.
Handles coordinate frame transformations and multi-marker fusion.
"""

import numpy as np
import yaml
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple
import logging
from pathlib import Path

from .aruco_detector import MarkerDetection

logger = logging.getLogger(__name__)


@dataclass
class MarkerConfig:
    """Configuration for a ceiling-mounted ArUco marker."""

    marker_id: int
    world_position: np.ndarray    # (x, y, z) in world frame
    orientation_deg: float        # Marker rotation in degrees
    description: str = ""

    @property
    def rotation_matrix(self) -> np.ndarray:
        """Get marker orientation as rotation matrix (about Z axis)."""
        angle_rad = np.radians(self.orientation_deg)
        cos_a, sin_a = np.cos(angle_rad), np.sin(angle_rad)
        return np.array([
            [cos_a, -sin_a, 0],
            [sin_a, cos_a, 0],
            [0, 0, 1]
        ])


@dataclass
class DroneState:
    """Estimated drone state in world frame."""

    position: np.ndarray          # (x, y, z) in meters
    yaw: float                    # Yaw angle in degrees
    timestamp: float              # Estimation timestamp
    marker_ids: List[int]         # Markers used for estimation
    confidence: float             # Estimation confidence (0-1)

    @property
    def x(self) -> float:
        return float(self.position[0])

    @property
    def y(self) -> float:
        return float(self.position[1])

    @property
    def z(self) -> float:
        return float(self.position[2])

    def distance_to(self, target: np.ndarray) -> float:
        """Calculate 3D distance to a target position."""
        return float(np.linalg.norm(self.position - target))

    def horizontal_distance_to(self, target: np.ndarray) -> float:
        """Calculate horizontal (XY) distance to target."""
        return float(np.linalg.norm(self.position[:2] - target[:2]))


class PositionEstimator:
    """
    Estimates drone position in world frame from ArUco marker detections.

    Coordinate Frames:
    - Camera frame: X-right, Y-down, Z-forward (OpenCV convention)
    - Body frame: X-forward, Y-right, Z-down (drone convention)
    - World frame: X-East, Y-North, Z-Up (ENU convention)

    The camera is mounted facing UP on the drone, looking at ceiling markers.
    """

    def __init__(
        self,
        marker_map_path: Optional[str] = None,
        camera_offset: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    ):
        """
        Initialize position estimator.

        Args:
            marker_map_path: Path to marker map YAML file
            camera_offset: Camera offset from drone center (x, y, z) in body frame
        """
        self.markers: Dict[int, MarkerConfig] = {}
        self.camera_offset = np.array(camera_offset)

        # Camera to body frame rotation (camera facing up)
        # Camera: Z forward, X right, Y down
        # Body: X forward, Y right, Z down
        # With camera facing up: Camera Z = Body -Z, Camera X = Body Y, Camera Y = Body X
        self._camera_to_body = np.array([
            [0, 1, 0],      # Body X = Camera Y
            [1, 0, 0],      # Body Y = Camera X
            [0, 0, -1]      # Body Z = -Camera Z
        ])

        # Body to world (ENU) rotation (assuming drone is level, yaw=0)
        # Body: X forward, Y right, Z down
        # World: X East, Y North, Z Up
        # Assuming drone faces North: Body X = World Y, Body Y = World X, Body Z = -World Z
        self._body_to_world_base = np.array([
            [0, 1, 0],      # World X = Body Y
            [1, 0, 0],      # World Y = Body X
            [0, 0, -1]      # World Z = -Body Z
        ])

        # State filtering
        self._last_state: Optional[DroneState] = None
        self._position_filter_alpha = 0.7  # Low-pass filter coefficient

        if marker_map_path:
            self.load_marker_map(marker_map_path)

    def load_marker_map(self, filepath: str) -> bool:
        """
        Load marker map from YAML file.

        Args:
            filepath: Path to marker map file

        Returns:
            True if loaded successfully
        """
        try:
            with open(filepath, 'r') as f:
                data = yaml.safe_load(f)

            self.markers.clear()

            for marker_data in data.get('markers', []):
                marker = MarkerConfig(
                    marker_id=marker_data['id'],
                    world_position=np.array(marker_data['position']),
                    orientation_deg=marker_data.get('orientation', 0),
                    description=marker_data.get('description', '')
                )
                self.markers[marker.marker_id] = marker

            logger.info(f"Loaded {len(self.markers)} markers from {filepath}")
            return True

        except Exception as e:
            logger.error(f"Failed to load marker map: {e}")
            return False

    def add_marker(
        self,
        marker_id: int,
        world_position: Tuple[float, float, float],
        orientation_deg: float = 0,
        description: str = ""
    ):
        """
        Add or update a marker in the map.

        Args:
            marker_id: ArUco marker ID
            world_position: Marker position in world frame (x, y, z)
            orientation_deg: Marker rotation about Z axis
            description: Optional description
        """
        self.markers[marker_id] = MarkerConfig(
            marker_id=marker_id,
            world_position=np.array(world_position),
            orientation_deg=orientation_deg,
            description=description
        )

    def estimate_from_single_marker(
        self,
        detection: MarkerDetection
    ) -> Optional[DroneState]:
        """
        Estimate drone position from a single marker detection.

        Args:
            detection: Detected marker with pose

        Returns:
            Estimated drone state or None if marker unknown
        """
        if detection.marker_id not in self.markers:
            logger.warning(f"Unknown marker ID: {detection.marker_id}")
            return None

        marker = self.markers[detection.marker_id]

        # Get marker position relative to camera
        # tvec is marker position in camera frame
        marker_in_camera = detection.tvec

        # Transform to body frame
        marker_in_body = self._camera_to_body @ marker_in_camera

        # The marker is above the drone, so drone position is:
        # marker_world_pos - (marker position relative to drone in world frame)

        # Get drone's yaw from marker orientation
        # When camera sees marker, the marker's rotation tells us drone yaw
        roll, pitch, yaw_camera = detection.get_euler_angles()

        # Adjust yaw for marker orientation and camera mounting
        drone_yaw = -yaw_camera + marker.orientation_deg

        # Create body-to-world rotation with current yaw
        yaw_rad = np.radians(drone_yaw)
        cos_y, sin_y = np.cos(yaw_rad), np.sin(yaw_rad)
        yaw_rotation = np.array([
            [cos_y, -sin_y, 0],
            [sin_y, cos_y, 0],
            [0, 0, 1]
        ])

        body_to_world = yaw_rotation @ self._body_to_world_base

        # Transform marker-relative position to world frame
        marker_in_world_relative = body_to_world @ marker_in_body

        # Drone position = marker world position - relative offset
        # Since marker is on ceiling and we're measuring from drone to marker,
        # we need to subtract to get drone position
        drone_position = marker.world_position - marker_in_world_relative

        # Account for camera offset from drone center
        camera_offset_world = body_to_world @ self.camera_offset
        drone_position -= camera_offset_world

        return DroneState(
            position=drone_position,
            yaw=drone_yaw,
            timestamp=detection.timestamp,
            marker_ids=[detection.marker_id],
            confidence=1.0
        )

    def estimate_from_multiple_markers(
        self,
        detections: List[MarkerDetection]
    ) -> Optional[DroneState]:
        """
        Estimate drone position from multiple marker detections.

        Uses weighted average based on detection quality (distance).

        Args:
            detections: List of detected markers

        Returns:
            Fused drone state estimate or None
        """
        if not detections:
            return None

        # Filter to known markers
        valid_detections = [
            d for d in detections
            if d.marker_id in self.markers
        ]

        if not valid_detections:
            return None

        if len(valid_detections) == 1:
            return self.estimate_from_single_marker(valid_detections[0])

        # Compute individual estimates
        estimates: List[Tuple[DroneState, float]] = []

        for detection in valid_detections:
            state = self.estimate_from_single_marker(detection)
            if state is not None:
                # Weight by inverse distance (closer markers are more accurate)
                weight = 1.0 / (detection.distance + 0.1)
                estimates.append((state, weight))

        if not estimates:
            return None

        # Weighted average of positions
        total_weight = sum(w for _, w in estimates)
        weighted_position = sum(
            state.position * w for state, w in estimates
        ) / total_weight

        # Weighted average of yaw (handle wraparound)
        sin_sum = sum(np.sin(np.radians(s.yaw)) * w for s, w in estimates)
        cos_sum = sum(np.cos(np.radians(s.yaw)) * w for s, w in estimates)
        weighted_yaw = np.degrees(np.arctan2(sin_sum, cos_sum))

        # Use latest timestamp
        timestamp = max(s.timestamp for s, _ in estimates)

        # Collect all marker IDs used
        marker_ids = [s.marker_ids[0] for s, _ in estimates]

        # Confidence based on number of markers and agreement
        positions = np.array([s.position for s, _ in estimates])
        position_std = np.std(positions, axis=0).mean()
        confidence = min(1.0, len(estimates) / 3.0) * np.exp(-position_std)

        return DroneState(
            position=weighted_position,
            yaw=weighted_yaw,
            timestamp=timestamp,
            marker_ids=marker_ids,
            confidence=confidence
        )

    def estimate(
        self,
        detections: List[MarkerDetection],
        filter_enabled: bool = True
    ) -> Optional[DroneState]:
        """
        Main estimation function with optional filtering.

        Args:
            detections: List of marker detections
            filter_enabled: Apply low-pass filter to position

        Returns:
            Filtered drone state estimate
        """
        raw_state = self.estimate_from_multiple_markers(detections)

        if raw_state is None:
            return self._last_state

        if filter_enabled and self._last_state is not None:
            # Low-pass filter on position
            alpha = self._position_filter_alpha
            filtered_position = (
                alpha * raw_state.position +
                (1 - alpha) * self._last_state.position
            )

            # Low-pass filter on yaw (handle wraparound)
            yaw_diff = raw_state.yaw - self._last_state.yaw
            if yaw_diff > 180:
                yaw_diff -= 360
            elif yaw_diff < -180:
                yaw_diff += 360

            filtered_yaw = self._last_state.yaw + alpha * yaw_diff

            raw_state = DroneState(
                position=filtered_position,
                yaw=filtered_yaw,
                timestamp=raw_state.timestamp,
                marker_ids=raw_state.marker_ids,
                confidence=raw_state.confidence
            )

        self._last_state = raw_state
        return raw_state

    def get_offset_from_point(
        self,
        state: DroneState,
        target: np.ndarray
    ) -> Tuple[float, float, float]:
        """
        Calculate offset from drone to target in body frame.

        Useful for control: positive x means target is ahead,
        positive y means target is to the right.

        Args:
            state: Current drone state
            target: Target position in world frame

        Returns:
            Offset (dx, dy, dz) in drone body frame
        """
        # World frame offset
        world_offset = target - state.position

        # Rotate to body frame using drone's yaw
        yaw_rad = np.radians(-state.yaw)  # Negative for world-to-body
        cos_y, sin_y = np.cos(yaw_rad), np.sin(yaw_rad)

        body_offset_x = cos_y * world_offset[0] - sin_y * world_offset[1]
        body_offset_y = sin_y * world_offset[0] + cos_y * world_offset[1]
        body_offset_z = world_offset[2]

        return (body_offset_x, body_offset_y, body_offset_z)

    def reset_filter(self):
        """Reset the position filter state."""
        self._last_state = None

    @property
    def last_state(self) -> Optional[DroneState]:
        """Get the last estimated state."""
        return self._last_state


class SimplePositionEstimator:
    """
    Simplified position estimator for Phase 1 single-marker navigation.

    Reports drone offset from marker center without world frame transformation.
    Useful for initial testing: just hover under a single marker.
    """

    def __init__(self):
        """Initialize simple estimator."""
        # Camera to drone body transformation
        # Camera facing up: Z points up toward marker
        # Positive camera X = drone right
        # Positive camera Y = drone forward (after transform)
        pass

    def get_offset_from_marker(
        self,
        detection: MarkerDetection
    ) -> Tuple[float, float, float, float]:
        """
        Get drone offset from marker center.

        Args:
            detection: Marker detection

        Returns:
            Tuple of (forward_offset, right_offset, altitude, yaw_to_marker)
            - forward_offset: positive = drone is behind marker center
            - right_offset: positive = drone is left of marker center
            - altitude: distance to marker (height)
            - yaw_to_marker: angle to align with marker
        """
        # Marker position in camera frame
        x_cam, y_cam, z_cam = detection.position

        # Camera facing up, marker above:
        # Camera X (right) = Drone right offset
        # Camera Y (down in normal camera, but marker is above) = Drone forward offset
        # Camera Z (forward, toward marker) = Altitude

        # The signs depend on mounting, but typically:
        forward_offset = y_cam      # How far forward/back drone is from marker center
        right_offset = x_cam        # How far right/left
        altitude = z_cam            # Distance to marker

        # Yaw angle from marker rotation
        _, _, yaw = detection.get_euler_angles()

        return (forward_offset, right_offset, altitude, yaw)


def test_estimator():
    """Test position estimation with simulated data."""
    logging.basicConfig(level=logging.INFO)

    estimator = PositionEstimator()

    # Add a test marker at origin, 3m high
    estimator.add_marker(0, (0.0, 0.0, 3.0), 0, "Test marker")

    # Simulate a detection
    # Marker at (0, 0, 3) world, drone at (0.5, 0.5, 1.0)
    # Marker should appear at offset in camera frame

    print("Position estimator test - add real detections for testing")


if __name__ == "__main__":
    test_estimator()
