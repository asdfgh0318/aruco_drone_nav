"""
Position Predictor Module

Handles position prediction when ArUco markers are temporarily lost.
Uses velocity-based dead reckoning and marker position prediction.
"""

import time
import logging
import numpy as np
from collections import deque
from dataclasses import dataclass
from typing import Optional, List, Tuple, Dict

logger = logging.getLogger(__name__)


@dataclass
class PositionHistory:
    """Single position history entry."""

    timestamp: float
    position: np.ndarray     # [x, y, z]
    yaw: float               # degrees
    marker_ids: List[int]
    confidence: float


class PositionPredictor:
    """
    Predicts drone position during marker loss using dead reckoning.

    Uses recent position history to estimate velocity and extrapolate
    position forward in time. Also predicts where markers should appear
    in the camera frame to aid re-acquisition.
    """

    def __init__(
        self,
        history_length: int = 50,
        max_prediction_time: float = 2.0,
        velocity_smoothing: float = 0.3
    ):
        """
        Initialize position predictor.

        Args:
            history_length: Number of position samples to keep
            max_prediction_time: Maximum time to predict (seconds)
            velocity_smoothing: Exponential smoothing factor for velocity
        """
        self.history_length = history_length
        self.max_prediction_time = max_prediction_time
        self.velocity_smoothing = velocity_smoothing

        self._history: deque = deque(maxlen=history_length)
        self._velocity = np.zeros(3)  # Smoothed velocity estimate
        self._yaw_rate = 0.0          # Smoothed yaw rate estimate
        self._last_update_time: Optional[float] = None
        self._prediction_start_time: Optional[float] = None

    def update(
        self,
        position: np.ndarray,
        yaw: float,
        marker_ids: List[int],
        confidence: float = 1.0,
        timestamp: Optional[float] = None
    ):
        """
        Update predictor with new measured position.

        Args:
            position: Current position [x, y, z]
            yaw: Current yaw angle (degrees)
            marker_ids: Visible marker IDs
            confidence: Position confidence (0-1)
            timestamp: Measurement timestamp (default: now)
        """
        current_time = timestamp or time.time()

        # Create history entry
        entry = PositionHistory(
            timestamp=current_time,
            position=position.copy(),
            yaw=yaw,
            marker_ids=marker_ids,
            confidence=confidence
        )
        self._history.append(entry)

        # Update velocity estimate
        if self._last_update_time is not None and len(self._history) >= 2:
            dt = current_time - self._last_update_time
            if dt > 0.001:  # Avoid division by tiny dt
                prev = self._history[-2]
                new_velocity = (position - prev.position) / dt

                # Exponential smoothing
                alpha = self.velocity_smoothing
                self._velocity = alpha * new_velocity + (1 - alpha) * self._velocity

                # Yaw rate
                yaw_diff = yaw - prev.yaw
                if yaw_diff > 180:
                    yaw_diff -= 360
                elif yaw_diff < -180:
                    yaw_diff += 360
                new_yaw_rate = yaw_diff / dt
                self._yaw_rate = alpha * new_yaw_rate + (1 - alpha) * self._yaw_rate

        self._last_update_time = current_time
        self._prediction_start_time = None  # Reset prediction mode

    def predict(self, timestamp: Optional[float] = None) -> Optional[Tuple[np.ndarray, float, float]]:
        """
        Predict position at given timestamp using dead reckoning.

        Args:
            timestamp: Time to predict for (default: now)

        Returns:
            Tuple of (position, yaw, confidence) or None if cannot predict
        """
        if len(self._history) < 2:
            return None

        current_time = timestamp or time.time()
        last_entry = self._history[-1]

        # Calculate time since last measurement
        dt = current_time - last_entry.timestamp

        if dt < 0:
            return None

        # Track prediction start for timeout
        if self._prediction_start_time is None:
            self._prediction_start_time = current_time
            logger.warning("Entering prediction mode - markers lost")

        prediction_duration = current_time - self._prediction_start_time

        if prediction_duration > self.max_prediction_time:
            logger.error(
                f"Prediction timeout ({prediction_duration:.1f}s > "
                f"{self.max_prediction_time:.1f}s)"
            )
            return None

        # Dead reckon position
        predicted_position = last_entry.position + self._velocity * dt

        # Dead reckon yaw
        predicted_yaw = last_entry.yaw + self._yaw_rate * dt
        while predicted_yaw > 180:
            predicted_yaw -= 360
        while predicted_yaw < -180:
            predicted_yaw += 360

        # Confidence decays with prediction time
        confidence = max(0.1, 1.0 - prediction_duration / self.max_prediction_time)

        return predicted_position, predicted_yaw, confidence

    def predict_marker_position(
        self,
        marker_world_pos: np.ndarray,
        camera_matrix: np.ndarray,
        predicted_position: np.ndarray,
        predicted_yaw: float
    ) -> Optional[Tuple[float, float]]:
        """
        Predict where a marker should appear in the camera frame.

        Useful for directed search when re-acquiring markers.

        Args:
            marker_world_pos: Marker position in world frame
            camera_matrix: Camera intrinsic matrix
            predicted_position: Predicted drone position
            predicted_yaw: Predicted drone yaw

        Returns:
            Tuple of (u, v) pixel coordinates or None if out of frame
        """
        # Calculate marker position relative to drone
        relative_pos = marker_world_pos - predicted_position

        # Rotate to body frame
        yaw_rad = np.radians(-predicted_yaw)
        cos_y, sin_y = np.cos(yaw_rad), np.sin(yaw_rad)
        rotation = np.array([
            [cos_y, -sin_y, 0],
            [sin_y, cos_y, 0],
            [0, 0, 1]
        ])
        body_pos = rotation @ relative_pos

        # Transform to camera frame (camera facing up)
        # Body X -> Camera Y, Body Y -> Camera X, Body Z -> -Camera Z
        camera_pos = np.array([body_pos[1], body_pos[0], -body_pos[2]])

        if camera_pos[2] <= 0:
            # Marker behind camera
            return None

        # Project to image plane
        fx = camera_matrix[0, 0]
        fy = camera_matrix[1, 1]
        cx = camera_matrix[0, 2]
        cy = camera_matrix[1, 2]

        u = fx * camera_pos[0] / camera_pos[2] + cx
        v = fy * camera_pos[1] / camera_pos[2] + cy

        return (u, v)

    def get_velocity(self) -> np.ndarray:
        """Get current velocity estimate."""
        return self._velocity.copy()

    def get_yaw_rate(self) -> float:
        """Get current yaw rate estimate (deg/s)."""
        return self._yaw_rate

    def get_speed(self) -> float:
        """Get current speed (magnitude of velocity)."""
        return float(np.linalg.norm(self._velocity))

    def get_prediction_duration(self) -> float:
        """Get time spent in prediction mode."""
        if self._prediction_start_time is None:
            return 0.0
        return time.time() - self._prediction_start_time

    def is_predicting(self) -> bool:
        """Check if currently in prediction mode."""
        return self._prediction_start_time is not None

    def reset(self):
        """Reset predictor state."""
        self._history.clear()
        self._velocity = np.zeros(3)
        self._yaw_rate = 0.0
        self._last_update_time = None
        self._prediction_start_time = None

    def get_statistics(self) -> Dict:
        """Get predictor statistics."""
        if len(self._history) < 2:
            return {'samples': len(self._history)}

        positions = np.array([h.position for h in self._history])
        timestamps = np.array([h.timestamp for h in self._history])

        dt = timestamps[-1] - timestamps[0]
        if dt > 0:
            avg_rate = len(self._history) / dt
        else:
            avg_rate = 0

        return {
            'samples': len(self._history),
            'time_span': dt,
            'avg_sample_rate': avg_rate,
            'velocity': self._velocity.tolist(),
            'speed': self.get_speed(),
            'yaw_rate': self._yaw_rate,
            'is_predicting': self.is_predicting(),
            'prediction_duration': self.get_prediction_duration()
        }


class KalmanPositionFilter:
    """
    Simple Kalman filter for position estimation.

    Provides smoother position estimates by fusing measurements
    with a constant-velocity motion model.
    """

    def __init__(
        self,
        process_noise: float = 0.1,
        measurement_noise: float = 0.05
    ):
        """
        Initialize Kalman filter.

        Args:
            process_noise: Process noise variance
            measurement_noise: Measurement noise variance
        """
        # State: [x, y, z, vx, vy, vz]
        self.state = np.zeros(6)
        self.covariance = np.eye(6) * 1.0

        # Process noise
        self.Q = np.eye(6) * process_noise
        self.Q[3:, 3:] *= 2  # Higher noise for velocity

        # Measurement noise
        self.R = np.eye(3) * measurement_noise

        # Measurement matrix (we only measure position)
        self.H = np.zeros((3, 6))
        self.H[0, 0] = 1
        self.H[1, 1] = 1
        self.H[2, 2] = 1

        self._last_time: Optional[float] = None
        self._initialized = False

    def predict(self, dt: float):
        """Predict state forward by dt seconds."""
        # State transition matrix
        F = np.eye(6)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt

        # Predict state
        self.state = F @ self.state

        # Predict covariance
        self.covariance = F @ self.covariance @ F.T + self.Q * dt

    def update(
        self,
        measurement: np.ndarray,
        timestamp: Optional[float] = None
    ) -> np.ndarray:
        """
        Update filter with new measurement.

        Args:
            measurement: Position measurement [x, y, z]
            timestamp: Measurement timestamp

        Returns:
            Filtered position estimate
        """
        current_time = timestamp or time.time()

        if not self._initialized:
            self.state[:3] = measurement
            self._last_time = current_time
            self._initialized = True
            return measurement

        # Predict to current time
        dt = current_time - self._last_time
        if dt > 0:
            self.predict(dt)

        # Kalman update
        y = measurement - self.H @ self.state  # Innovation
        S = self.H @ self.covariance @ self.H.T + self.R  # Innovation covariance
        K = self.covariance @ self.H.T @ np.linalg.inv(S)  # Kalman gain

        self.state = self.state + K @ y
        self.covariance = (np.eye(6) - K @ self.H) @ self.covariance

        self._last_time = current_time

        return self.state[:3].copy()

    def get_position(self) -> np.ndarray:
        """Get current position estimate."""
        return self.state[:3].copy()

    def get_velocity(self) -> np.ndarray:
        """Get current velocity estimate."""
        return self.state[3:].copy()

    def reset(self):
        """Reset filter state."""
        self.state = np.zeros(6)
        self.covariance = np.eye(6) * 1.0
        self._last_time = None
        self._initialized = False


if __name__ == "__main__":
    # Test position predictor
    import time

    predictor = PositionPredictor(max_prediction_time=3.0)

    # Simulate flying in a line
    print("Simulating flight with velocity [1, 0, 0] m/s...")

    for i in range(20):
        t = i * 0.1
        pos = np.array([t, 0.0, 1.5])
        predictor.update(pos, yaw=0.0, marker_ids=[0])
        time.sleep(0.1)

    print(f"Velocity estimate: {predictor.get_velocity()}")
    print(f"Speed: {predictor.get_speed():.3f} m/s")

    # Test prediction
    print("\nSimulating marker loss for 2 seconds...")
    for i in range(20):
        result = predictor.predict()
        if result:
            pos, yaw, conf = result
            print(
                f"Predicted: pos={pos}, conf={conf:.2f}, "
                f"duration={predictor.get_prediction_duration():.1f}s"
            )
        time.sleep(0.1)
