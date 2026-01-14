"""
Flight Recorder Module

Records flight path data for playback in VR software.
Supports the same JSON format as mission input for easy iteration.
"""

import json
import time
import logging
import threading
from datetime import datetime
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Dict, Any

import numpy as np

logger = logging.getLogger(__name__)


@dataclass
class FlightSample:
    """Single sample of flight data."""

    timestamp: float              # Time since recording start (seconds)
    x: float                      # X position (meters)
    y: float                      # Y position (meters)
    z: float                      # Z position (meters)
    yaw: float                    # Yaw angle (degrees)
    vx: float = 0.0               # X velocity (m/s)
    vy: float = 0.0               # Y velocity (m/s)
    vz: float = 0.0               # Z velocity (m/s)
    marker_ids: List[int] = field(default_factory=list)  # Visible markers
    confidence: float = 1.0       # Position confidence

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary."""
        return {
            't': round(self.timestamp, 3),
            'x': round(self.x, 4),
            'y': round(self.y, 4),
            'z': round(self.z, 4),
            'yaw': round(self.yaw, 2),
            'vx': round(self.vx, 3),
            'vy': round(self.vy, 3),
            'vz': round(self.vz, 3),
            'markers': self.marker_ids,
            'conf': round(self.confidence, 2)
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'FlightSample':
        """Create from dictionary."""
        return cls(
            timestamp=data.get('t', 0),
            x=data.get('x', 0),
            y=data.get('y', 0),
            z=data.get('z', 0),
            yaw=data.get('yaw', 0),
            vx=data.get('vx', 0),
            vy=data.get('vy', 0),
            vz=data.get('vz', 0),
            marker_ids=data.get('markers', []),
            confidence=data.get('conf', 1.0)
        )


@dataclass
class FlightRecording:
    """Complete flight recording."""

    recording_id: str
    mission_id: str = ""
    start_time: str = ""
    samples: List[FlightSample] = field(default_factory=list)
    metadata: Dict[str, Any] = field(default_factory=dict)

    @property
    def duration(self) -> float:
        """Get recording duration in seconds."""
        if not self.samples:
            return 0.0
        return self.samples[-1].timestamp

    @property
    def num_samples(self) -> int:
        """Get number of samples."""
        return len(self.samples)

    @property
    def sample_rate(self) -> float:
        """Get average sample rate in Hz."""
        if self.duration > 0:
            return self.num_samples / self.duration
        return 0.0

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary (VR-compatible format)."""
        return {
            'recording_id': self.recording_id,
            'mission_id': self.mission_id,
            'start_time': self.start_time,
            'duration': round(self.duration, 2),
            'num_samples': self.num_samples,
            'sample_rate_hz': round(self.sample_rate, 1),
            'samples': [s.to_dict() for s in self.samples],
            'metadata': self.metadata
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'FlightRecording':
        """Create from dictionary."""
        samples = [
            FlightSample.from_dict(s)
            for s in data.get('samples', [])
        ]
        return cls(
            recording_id=data.get('recording_id', ''),
            mission_id=data.get('mission_id', ''),
            start_time=data.get('start_time', ''),
            samples=samples,
            metadata=data.get('metadata', {})
        )

    def save_to_json(self, filepath: str):
        """Save recording to JSON file."""
        Path(filepath).parent.mkdir(parents=True, exist_ok=True)

        with open(filepath, 'w') as f:
            json.dump(self.to_dict(), f, indent=2)

        logger.info(f"Saved recording to {filepath}")

    @classmethod
    def load_from_json(cls, filepath: str) -> 'FlightRecording':
        """Load recording from JSON file."""
        with open(filepath, 'r') as f:
            data = json.load(f)
        return cls.from_dict(data)

    def get_path_as_waypoints(self, interval: float = 1.0) -> List[Dict[str, float]]:
        """
        Convert recording to waypoint list (for VR reimport).

        Args:
            interval: Time interval between waypoints (seconds)

        Returns:
            List of waypoint dictionaries
        """
        waypoints = []
        last_time = -interval

        for sample in self.samples:
            if sample.timestamp - last_time >= interval:
                waypoints.append({
                    'x': sample.x,
                    'y': sample.y,
                    'z': sample.z,
                    'yaw': sample.yaw,
                    'hold_time': 0.0
                })
                last_time = sample.timestamp

        return waypoints


class FlightRecorder:
    """
    Records flight path data during mission execution.

    Thread-safe recording with configurable sample rate.
    """

    def __init__(
        self,
        sample_rate_hz: float = 10.0,
        output_dir: str = "recordings"
    ):
        """
        Initialize flight recorder.

        Args:
            sample_rate_hz: Target sample rate in Hz
            output_dir: Directory for saving recordings
        """
        self.sample_rate = sample_rate_hz
        self.sample_interval = 1.0 / sample_rate_hz
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self._recording: Optional[FlightRecording] = None
        self._recording_active = False
        self._start_time: Optional[float] = None
        self._last_sample_time = 0.0
        self._lock = threading.Lock()

    @property
    def is_recording(self) -> bool:
        """Check if recording is active."""
        return self._recording_active

    @property
    def current_recording(self) -> Optional[FlightRecording]:
        """Get current recording."""
        return self._recording

    def start_recording(
        self,
        mission_id: str = "",
        metadata: Optional[Dict[str, Any]] = None
    ) -> str:
        """
        Start a new recording.

        Args:
            mission_id: Associated mission ID
            metadata: Additional metadata

        Returns:
            Recording ID
        """
        with self._lock:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            recording_id = f"flight_{timestamp}"

            self._recording = FlightRecording(
                recording_id=recording_id,
                mission_id=mission_id,
                start_time=datetime.now().isoformat(),
                metadata=metadata or {}
            )

            self._start_time = time.time()
            self._last_sample_time = 0.0
            self._recording_active = True

            logger.info(f"Started recording: {recording_id}")
            return recording_id

    def stop_recording(self, save: bool = True) -> Optional[str]:
        """
        Stop recording and optionally save.

        Args:
            save: Whether to save the recording

        Returns:
            Path to saved file or None
        """
        with self._lock:
            if not self._recording_active:
                return None

            self._recording_active = False
            filepath = None

            if save and self._recording:
                filename = f"{self._recording.recording_id}.json"
                filepath = str(self.output_dir / filename)
                self._recording.save_to_json(filepath)

            logger.info(
                f"Stopped recording: {self._recording.num_samples} samples, "
                f"{self._recording.duration:.1f}s duration"
            )

            return filepath

    def add_sample(
        self,
        x: float,
        y: float,
        z: float,
        yaw: float,
        vx: float = 0.0,
        vy: float = 0.0,
        vz: float = 0.0,
        marker_ids: Optional[List[int]] = None,
        confidence: float = 1.0,
        force: bool = False
    ) -> bool:
        """
        Add a sample to the recording.

        Args:
            x, y, z: Position in meters
            yaw: Yaw angle in degrees
            vx, vy, vz: Velocity in m/s
            marker_ids: List of visible marker IDs
            confidence: Position confidence (0-1)
            force: Force adding sample regardless of interval

        Returns:
            True if sample was added
        """
        if not self._recording_active:
            return False

        current_time = time.time() - self._start_time

        # Rate limiting
        if not force and (current_time - self._last_sample_time) < self.sample_interval:
            return False

        with self._lock:
            sample = FlightSample(
                timestamp=current_time,
                x=x,
                y=y,
                z=z,
                yaw=yaw,
                vx=vx,
                vy=vy,
                vz=vz,
                marker_ids=marker_ids or [],
                confidence=confidence
            )

            self._recording.samples.append(sample)
            self._last_sample_time = current_time

        return True

    def add_state(
        self,
        position: np.ndarray,
        yaw: float,
        velocity: Optional[np.ndarray] = None,
        marker_ids: Optional[List[int]] = None,
        confidence: float = 1.0
    ) -> bool:
        """
        Add sample from state arrays.

        Args:
            position: Position array [x, y, z]
            yaw: Yaw angle in degrees
            velocity: Velocity array [vx, vy, vz]
            marker_ids: Visible marker IDs
            confidence: Position confidence

        Returns:
            True if sample was added
        """
        vel = velocity if velocity is not None else np.zeros(3)
        return self.add_sample(
            x=float(position[0]),
            y=float(position[1]),
            z=float(position[2]),
            yaw=yaw,
            vx=float(vel[0]),
            vy=float(vel[1]),
            vz=float(vel[2]),
            marker_ids=marker_ids,
            confidence=confidence
        )

    def get_statistics(self) -> Dict[str, Any]:
        """Get recording statistics."""
        if not self._recording:
            return {}

        samples = self._recording.samples

        if not samples:
            return {
                'num_samples': 0,
                'duration': 0,
                'sample_rate': 0
            }

        positions = np.array([[s.x, s.y, s.z] for s in samples])

        return {
            'num_samples': len(samples),
            'duration': self._recording.duration,
            'sample_rate': self._recording.sample_rate,
            'position_min': positions.min(axis=0).tolist(),
            'position_max': positions.max(axis=0).tolist(),
            'position_mean': positions.mean(axis=0).tolist(),
            'avg_confidence': np.mean([s.confidence for s in samples])
        }


def merge_recordings(
    recordings: List[FlightRecording],
    output_path: str
) -> FlightRecording:
    """
    Merge multiple recordings into one.

    Args:
        recordings: List of recordings to merge
        output_path: Path to save merged recording

    Returns:
        Merged recording
    """
    if not recordings:
        raise ValueError("No recordings to merge")

    merged = FlightRecording(
        recording_id=f"merged_{datetime.now().strftime('%Y%m%d_%H%M%S')}",
        start_time=recordings[0].start_time,
        metadata={'merged_from': [r.recording_id for r in recordings]}
    )

    time_offset = 0.0

    for recording in recordings:
        for sample in recording.samples:
            new_sample = FlightSample(
                timestamp=sample.timestamp + time_offset,
                x=sample.x,
                y=sample.y,
                z=sample.z,
                yaw=sample.yaw,
                vx=sample.vx,
                vy=sample.vy,
                vz=sample.vz,
                marker_ids=sample.marker_ids,
                confidence=sample.confidence
            )
            merged.samples.append(new_sample)

        time_offset += recording.duration + 1.0  # 1 second gap

    merged.save_to_json(output_path)
    return merged


if __name__ == "__main__":
    # Test recording
    import numpy as np

    recorder = FlightRecorder(sample_rate_hz=10)
    recorder.start_recording(mission_id="test_flight")

    # Simulate flight path
    for t in range(100):
        angle = t * 0.1
        x = np.cos(angle) * 2
        y = np.sin(angle) * 2
        z = 1.5 + np.sin(t * 0.05) * 0.2

        recorder.add_sample(
            x=x, y=y, z=z,
            yaw=np.degrees(angle),
            marker_ids=[0] if t % 3 == 0 else []
        )
        time.sleep(0.1)

    filepath = recorder.stop_recording()
    print(f"Saved to: {filepath}")
    print(f"Statistics: {recorder.get_statistics()}")
