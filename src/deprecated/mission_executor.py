"""
Mission Executor Module

Handles loading, parsing, and executing flight missions from JSON files.
Manages waypoint navigation and mission state machine.
"""

import json
import time
import logging
import numpy as np
from dataclasses import dataclass, field
from enum import Enum, auto
from pathlib import Path
from typing import List, Optional, Dict, Any, Callable

logger = logging.getLogger(__name__)


class MissionState(Enum):
    """Mission execution state."""
    IDLE = auto()
    LOADING = auto()
    READY = auto()
    TAKING_OFF = auto()
    NAVIGATING = auto()
    APPROACHING_WAYPOINT = auto()
    AT_WAYPOINT = auto()
    HOLDING = auto()
    LANDING = auto()
    COMPLETE = auto()
    ABORTED = auto()
    ERROR = auto()


@dataclass
class Waypoint:
    """Single waypoint in a mission."""

    x: float                      # X position (meters)
    y: float                      # Y position (meters)
    z: float                      # Z position (altitude in meters)
    yaw: float = 0.0              # Yaw angle (degrees)
    hold_time: float = 0.0        # Time to hold at waypoint (seconds)
    speed: Optional[float] = None # Override speed for this segment
    name: str = ""                # Optional waypoint name

    @property
    def position(self) -> np.ndarray:
        """Get position as numpy array."""
        return np.array([self.x, self.y, self.z])

    def distance_to(self, other: 'Waypoint') -> float:
        """Calculate distance to another waypoint."""
        return float(np.linalg.norm(self.position - other.position))

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'Waypoint':
        """Create waypoint from dictionary."""
        return cls(
            x=float(data.get('x', 0)),
            y=float(data.get('y', 0)),
            z=float(data.get('z', 1.0)),
            yaw=float(data.get('yaw', 0)),
            hold_time=float(data.get('hold_time', 0)),
            speed=data.get('speed'),
            name=data.get('name', '')
        )

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary."""
        return {
            'x': self.x,
            'y': self.y,
            'z': self.z,
            'yaw': self.yaw,
            'hold_time': self.hold_time,
            'speed': self.speed,
            'name': self.name
        }


@dataclass
class MissionSettings:
    """Mission-wide settings."""

    max_speed: float = 0.5             # Maximum speed (m/s)
    position_tolerance: float = 0.1    # Position tolerance (meters)
    yaw_tolerance: float = 5.0         # Yaw tolerance (degrees)
    takeoff_altitude: float = 1.0      # Initial takeoff altitude
    landing_speed: float = 0.2         # Landing descent speed (m/s)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'MissionSettings':
        """Create settings from dictionary."""
        return cls(
            max_speed=float(data.get('max_speed', 0.5)),
            position_tolerance=float(data.get('position_tolerance', 0.1)),
            yaw_tolerance=float(data.get('yaw_tolerance', 5.0)),
            takeoff_altitude=float(data.get('takeoff_altitude', 1.0)),
            landing_speed=float(data.get('landing_speed', 0.2))
        )


@dataclass
class Mission:
    """Complete mission definition."""

    mission_id: str
    waypoints: List[Waypoint] = field(default_factory=list)
    settings: MissionSettings = field(default_factory=MissionSettings)
    metadata: Dict[str, Any] = field(default_factory=dict)

    @property
    def num_waypoints(self) -> int:
        """Get number of waypoints."""
        return len(self.waypoints)

    @property
    def total_distance(self) -> float:
        """Calculate total mission distance."""
        if len(self.waypoints) < 2:
            return 0.0

        distance = 0.0
        for i in range(1, len(self.waypoints)):
            distance += self.waypoints[i].distance_to(self.waypoints[i-1])
        return distance

    @classmethod
    def from_json_file(cls, filepath: str) -> 'Mission':
        """Load mission from JSON file."""
        with open(filepath, 'r') as f:
            data = json.load(f)
        return cls.from_dict(data)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'Mission':
        """Create mission from dictionary."""
        waypoints = [
            Waypoint.from_dict(wp)
            for wp in data.get('waypoints', [])
        ]

        settings = MissionSettings.from_dict(
            data.get('settings', {})
        )

        return cls(
            mission_id=data.get('mission_id', 'unknown'),
            waypoints=waypoints,
            settings=settings,
            metadata=data.get('metadata', {})
        )

    def to_dict(self) -> Dict[str, Any]:
        """Convert mission to dictionary."""
        return {
            'mission_id': self.mission_id,
            'waypoints': [wp.to_dict() for wp in self.waypoints],
            'settings': {
                'max_speed': self.settings.max_speed,
                'position_tolerance': self.settings.position_tolerance,
                'yaw_tolerance': self.settings.yaw_tolerance,
                'takeoff_altitude': self.settings.takeoff_altitude,
                'landing_speed': self.settings.landing_speed
            },
            'metadata': self.metadata
        }

    def save_to_json(self, filepath: str):
        """Save mission to JSON file."""
        with open(filepath, 'w') as f:
            json.dump(self.to_dict(), f, indent=2)


class MissionExecutor:
    """
    Executes flight missions with waypoint navigation.

    Handles the state machine for mission execution including
    takeoff, waypoint navigation, holding, and landing.
    """

    def __init__(
        self,
        position_callback: Callable[[], Optional[np.ndarray]],
        yaw_callback: Callable[[], float],
        velocity_command_callback: Callable[[float, float, float, float], None],
        arm_callback: Callable[[], bool],
        disarm_callback: Callable[[], bool],
        takeoff_callback: Callable[[float], bool],
        land_callback: Callable[[], bool]
    ):
        """
        Initialize mission executor.

        Args:
            position_callback: Returns current position (x, y, z) or None
            yaw_callback: Returns current yaw in degrees
            velocity_command_callback: Sends velocity command (vx, vy, vz, yaw_rate)
            arm_callback: Arms the drone
            disarm_callback: Disarms the drone
            takeoff_callback: Commands takeoff to altitude
            land_callback: Commands landing
        """
        self.get_position = position_callback
        self.get_yaw = yaw_callback
        self.send_velocity = velocity_command_callback
        self.arm = arm_callback
        self.disarm = disarm_callback
        self.takeoff = takeoff_callback
        self.land = land_callback

        # Mission state
        self.mission: Optional[Mission] = None
        self.state = MissionState.IDLE
        self.current_waypoint_index = 0
        self._waypoint_start_time: Optional[float] = None
        self._state_start_time = time.time()

        # Callbacks
        self._on_state_change: Optional[Callable[[MissionState], None]] = None
        self._on_waypoint_reached: Optional[Callable[[int, Waypoint], None]] = None

        # Control parameters
        self.position_kp = 0.5
        self.yaw_kp = 0.02

    def load_mission(self, filepath: str) -> bool:
        """
        Load mission from JSON file.

        Args:
            filepath: Path to mission JSON file

        Returns:
            True if loaded successfully
        """
        try:
            self.mission = Mission.from_json_file(filepath)
            self._set_state(MissionState.READY)

            logger.info(
                f"Loaded mission '{self.mission.mission_id}' "
                f"with {self.mission.num_waypoints} waypoints"
            )
            return True

        except Exception as e:
            logger.error(f"Failed to load mission: {e}")
            self._set_state(MissionState.ERROR)
            return False

    def load_mission_dict(self, data: Dict[str, Any]) -> bool:
        """
        Load mission from dictionary.

        Args:
            data: Mission data dictionary

        Returns:
            True if loaded successfully
        """
        try:
            self.mission = Mission.from_dict(data)
            self._set_state(MissionState.READY)
            return True
        except Exception as e:
            logger.error(f"Failed to load mission: {e}")
            return False

    def _set_state(self, new_state: MissionState):
        """Update state and notify callback."""
        if new_state != self.state:
            old_state = self.state
            self.state = new_state
            self._state_start_time = time.time()
            logger.info(f"Mission state: {old_state.name} -> {new_state.name}")

            if self._on_state_change:
                self._on_state_change(new_state)

    @property
    def current_waypoint(self) -> Optional[Waypoint]:
        """Get current target waypoint."""
        if self.mission and 0 <= self.current_waypoint_index < len(self.mission.waypoints):
            return self.mission.waypoints[self.current_waypoint_index]
        return None

    def start(self) -> bool:
        """
        Start mission execution.

        Returns:
            True if mission started
        """
        if self.mission is None or self.mission.num_waypoints == 0:
            logger.error("No mission loaded or mission is empty")
            return False

        if self.state != MissionState.READY:
            logger.error(f"Cannot start from state {self.state.name}")
            return False

        self.current_waypoint_index = 0
        self._set_state(MissionState.TAKING_OFF)

        # Arm and takeoff
        if not self.arm():
            logger.error("Failed to arm")
            self._set_state(MissionState.ERROR)
            return False

        time.sleep(0.5)  # Wait for arm

        if not self.takeoff(self.mission.settings.takeoff_altitude):
            logger.error("Failed to takeoff")
            self._set_state(MissionState.ERROR)
            return False

        return True

    def abort(self):
        """Abort mission and land."""
        logger.warning("Mission aborted!")
        self._set_state(MissionState.ABORTED)
        self.land()

    def update(self) -> bool:
        """
        Update mission execution (call in control loop).

        Returns:
            True if mission is still active
        """
        if self.state in [MissionState.COMPLETE, MissionState.ABORTED, MissionState.ERROR]:
            return False

        position = self.get_position()
        if position is None:
            # Lost position - hold
            self.send_velocity(0, 0, 0, 0)
            return True

        current_yaw = self.get_yaw()

        if self.state == MissionState.TAKING_OFF:
            self._handle_takeoff(position)

        elif self.state == MissionState.NAVIGATING:
            self._handle_navigation(position, current_yaw)

        elif self.state == MissionState.AT_WAYPOINT:
            self._handle_at_waypoint(position, current_yaw)

        elif self.state == MissionState.LANDING:
            self._handle_landing()

        return True

    def _handle_takeoff(self, position: np.ndarray):
        """Handle takeoff state."""
        takeoff_alt = self.mission.settings.takeoff_altitude

        if position[2] >= takeoff_alt * 0.9:
            logger.info("Takeoff complete, starting navigation")
            self._set_state(MissionState.NAVIGATING)

    def _handle_navigation(self, position: np.ndarray, current_yaw: float):
        """Handle navigation to waypoint."""
        waypoint = self.current_waypoint
        if waypoint is None:
            self._set_state(MissionState.LANDING)
            return

        # Calculate position error
        target = waypoint.position
        error = target - position

        distance = np.linalg.norm(error[:2])  # Horizontal distance
        altitude_error = error[2]

        # Calculate yaw error
        yaw_error = waypoint.yaw - current_yaw
        while yaw_error > 180:
            yaw_error -= 360
        while yaw_error < -180:
            yaw_error += 360

        # Check if waypoint reached
        tolerance = self.mission.settings.position_tolerance
        yaw_tolerance = self.mission.settings.yaw_tolerance

        if distance < tolerance and abs(altitude_error) < tolerance:
            if abs(yaw_error) < yaw_tolerance or waypoint.hold_time == 0:
                logger.info(f"Reached waypoint {self.current_waypoint_index}")
                self._waypoint_start_time = time.time()
                self._set_state(MissionState.AT_WAYPOINT)

                if self._on_waypoint_reached:
                    self._on_waypoint_reached(self.current_waypoint_index, waypoint)
                return

        # Calculate velocity command
        speed = waypoint.speed or self.mission.settings.max_speed

        # Normalize and scale velocity
        if distance > 0.01:
            direction = error[:2] / distance
            velocity_mag = min(speed, distance * self.position_kp)
            vx, vy = direction * velocity_mag
        else:
            vx, vy = 0.0, 0.0

        vz = np.clip(altitude_error * self.position_kp, -speed, speed)
        yaw_rate = yaw_error * self.yaw_kp

        self.send_velocity(vx, vy, vz, yaw_rate)

    def _handle_at_waypoint(self, position: np.ndarray, current_yaw: float):
        """Handle holding at waypoint."""
        waypoint = self.current_waypoint
        if waypoint is None:
            return

        # Hold position
        target = waypoint.position
        error = target - position

        vx = error[0] * self.position_kp
        vy = error[1] * self.position_kp
        vz = error[2] * self.position_kp

        yaw_error = waypoint.yaw - current_yaw
        while yaw_error > 180:
            yaw_error -= 360
        while yaw_error < -180:
            yaw_error += 360
        yaw_rate = yaw_error * self.yaw_kp

        speed = self.mission.settings.max_speed
        vx = np.clip(vx, -speed, speed)
        vy = np.clip(vy, -speed, speed)
        vz = np.clip(vz, -speed, speed)

        self.send_velocity(vx, vy, vz, yaw_rate)

        # Check hold time
        elapsed = time.time() - self._waypoint_start_time
        if elapsed >= waypoint.hold_time:
            # Move to next waypoint
            self.current_waypoint_index += 1

            if self.current_waypoint_index >= len(self.mission.waypoints):
                logger.info("All waypoints reached, landing")
                self._set_state(MissionState.LANDING)
                self.land()
            else:
                logger.info(f"Moving to waypoint {self.current_waypoint_index}")
                self._set_state(MissionState.NAVIGATING)

    def _handle_landing(self):
        """Handle landing state."""
        # Landing is handled by flight controller
        # Just monitor for completion
        position = self.get_position()
        if position is not None and position[2] < 0.1:
            self._set_state(MissionState.COMPLETE)
            self.disarm()
            logger.info("Mission complete!")

    def set_on_state_change(self, callback: Callable[[MissionState], None]):
        """Set callback for state changes."""
        self._on_state_change = callback

    def set_on_waypoint_reached(self, callback: Callable[[int, Waypoint], None]):
        """Set callback for waypoint reached events."""
        self._on_waypoint_reached = callback


def create_sample_mission() -> Mission:
    """Create a sample mission for testing."""
    waypoints = [
        Waypoint(x=0.0, y=0.0, z=1.5, yaw=0, hold_time=2.0, name="Start"),
        Waypoint(x=1.0, y=0.0, z=1.5, yaw=0, hold_time=1.0, name="Point A"),
        Waypoint(x=1.0, y=1.0, z=1.5, yaw=90, hold_time=1.0, name="Point B"),
        Waypoint(x=0.0, y=1.0, z=1.5, yaw=180, hold_time=1.0, name="Point C"),
        Waypoint(x=0.0, y=0.0, z=1.5, yaw=270, hold_time=2.0, name="Return"),
    ]

    return Mission(
        mission_id="sample_square",
        waypoints=waypoints,
        settings=MissionSettings(
            max_speed=0.3,
            position_tolerance=0.15,
            takeoff_altitude=1.5
        ),
        metadata={
            "description": "Simple square pattern flight",
            "author": "ArUco Nav System"
        }
    )


if __name__ == "__main__":
    # Create and save sample mission
    mission = create_sample_mission()
    mission.save_to_json("missions/sample_square.json")
    print(f"Created sample mission: {mission.mission_id}")
    print(f"Waypoints: {mission.num_waypoints}")
    print(f"Total distance: {mission.total_distance:.2f}m")
