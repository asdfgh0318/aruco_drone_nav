"""
Camera Calibration Module

Provides camera intrinsic calibration using chessboard pattern detection.
Computes camera matrix and distortion coefficients needed for accurate
ArUco marker pose estimation.
"""

import cv2
import numpy as np
import yaml
from pathlib import Path
from datetime import datetime
from typing import Tuple, Optional, List
import logging

logger = logging.getLogger(__name__)


class CameraCalibration:
    """Handles camera calibration using chessboard pattern."""

    def __init__(
        self,
        chessboard_size: Tuple[int, int] = (9, 6),
        square_size_m: float = 0.025
    ):
        """
        Initialize calibration handler.

        Args:
            chessboard_size: Number of inner corners (width, height)
            square_size_m: Size of each chessboard square in meters
        """
        self.chessboard_size = chessboard_size
        self.square_size_m = square_size_m

        # Prepare object points (3D points in real world space)
        self.objp = np.zeros(
            (chessboard_size[0] * chessboard_size[1], 3),
            np.float32
        )
        self.objp[:, :2] = np.mgrid[
            0:chessboard_size[0],
            0:chessboard_size[1]
        ].T.reshape(-1, 2)
        self.objp *= square_size_m

        # Storage for calibration points
        self.obj_points: List[np.ndarray] = []  # 3D points
        self.img_points: List[np.ndarray] = []  # 2D points
        self.image_size: Optional[Tuple[int, int]] = None

        # Calibration results
        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None
        self.reprojection_error: float = -1.0

    def find_chessboard(
        self,
        frame: np.ndarray,
        draw: bool = True
    ) -> Tuple[bool, np.ndarray, Optional[np.ndarray]]:
        """
        Find chessboard corners in a frame.

        Args:
            frame: Input image (BGR)
            draw: Whether to draw corners on the frame

        Returns:
            Tuple of (found, output_frame, corners)
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        output_frame = frame.copy()

        # Find chessboard corners
        flags = (
            cv2.CALIB_CB_ADAPTIVE_THRESH +
            cv2.CALIB_CB_NORMALIZE_IMAGE +
            cv2.CALIB_CB_FAST_CHECK
        )
        found, corners = cv2.findChessboardCorners(
            gray, self.chessboard_size, flags
        )

        if found:
            # Refine corner positions
            criteria = (
                cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                30, 0.001
            )
            corners = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1), criteria
            )

            if draw:
                cv2.drawChessboardCorners(
                    output_frame, self.chessboard_size, corners, found
                )

        return found, output_frame, corners if found else None

    def add_calibration_image(
        self,
        frame: np.ndarray
    ) -> bool:
        """
        Add a frame to calibration dataset if chessboard is found.

        Args:
            frame: Input image (BGR)

        Returns:
            True if chessboard was found and added
        """
        found, _, corners = self.find_chessboard(frame, draw=False)

        if found:
            self.obj_points.append(self.objp.copy())
            self.img_points.append(corners)

            if self.image_size is None:
                self.image_size = (frame.shape[1], frame.shape[0])

            logger.info(
                f"Added calibration image {len(self.img_points)}"
            )
            return True

        return False

    def calibrate(self, min_images: int = 10, wide_angle: bool = False) -> bool:
        """
        Perform camera calibration using collected images.

        Args:
            min_images: Minimum number of images required
            wide_angle: Use 8-coefficient rational model for wide-angle lenses
                        (FOV > 80°). Better handles barrel distortion.

        Returns:
            True if calibration succeeded
        """
        if len(self.img_points) < min_images:
            logger.error(
                f"Need at least {min_images} images, "
                f"have {len(self.img_points)}"
            )
            return False

        if self.image_size is None:
            logger.error("No image size available")
            return False

        logger.info(
            f"Calibrating with {len(self.img_points)} images "
            f"({'rational model' if wide_angle else 'standard model'})..."
        )

        # Use rational model for wide-angle lenses (8 coefficients)
        flags = cv2.CALIB_RATIONAL_MODEL if wide_angle else 0

        # Perform calibration
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
            self.obj_points,
            self.img_points,
            self.image_size,
            None,
            None,
            flags=flags
        )

        if ret:
            self.camera_matrix = mtx
            self.dist_coeffs = dist
            self.reprojection_error = ret
            self._wide_angle = wide_angle

            logger.info(
                f"Calibration successful! "
                f"Reprojection error: {ret:.4f} pixels "
                f"({len(dist.flatten())} distortion coefficients)"
            )
            return True
        else:
            logger.error("Calibration failed")
            return False

    def save_calibration(self, filepath: str) -> bool:
        """
        Save calibration parameters to YAML file.

        Args:
            filepath: Output file path

        Returns:
            True if saved successfully
        """
        if self.camera_matrix is None or self.dist_coeffs is None:
            logger.error("No calibration data to save")
            return False

        data = {
            'camera_matrix': {
                'rows': 3,
                'cols': 3,
                'data': self.camera_matrix.flatten().tolist()
            },
            'distortion_coefficients': {
                'rows': 1,
                'cols': len(self.dist_coeffs.flatten()),
                'data': self.dist_coeffs.flatten().tolist()
            },
            'image_size': {
                'width': self.image_size[0],
                'height': self.image_size[1]
            },
            'calibration_info': {
                'date': datetime.now().isoformat(),
                'reprojection_error': float(self.reprojection_error),
                'num_images': len(self.img_points),
                'chessboard_size': list(self.chessboard_size),
                'square_size_m': self.square_size_m,
                'distortion_model': 'rational' if getattr(self, '_wide_angle', False) else 'standard',
                'image_width': self.image_size[0],
                'image_height': self.image_size[1]
            }
        }

        Path(filepath).parent.mkdir(parents=True, exist_ok=True)

        with open(filepath, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)

        logger.info(f"Calibration saved to {filepath}")
        return True

    @classmethod
    def load_calibration(
        cls,
        filepath: str
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Load calibration parameters from YAML file.

        Args:
            filepath: Input file path

        Returns:
            Tuple of (camera_matrix, dist_coeffs) or (None, None) on error
        """
        try:
            with open(filepath, 'r') as f:
                data = yaml.safe_load(f)

            # Handle both old format (with 'data' key) and new format (direct list)
            cm_data = data['camera_matrix']
            if isinstance(cm_data, dict) and 'data' in cm_data:
                camera_matrix = np.array(cm_data['data']).reshape(3, 3)
            else:
                camera_matrix = np.array(cm_data).reshape(3, 3)

            dc_data = data['distortion_coefficients']
            if isinstance(dc_data, dict) and 'data' in dc_data:
                dist_coeffs = np.array(dc_data['data'])
            else:
                dist_coeffs = np.array(dc_data)

            # Check if this is just placeholder data
            if data.get('calibration_info', {}).get('date') == 'NOT_CALIBRATED':
                logger.warning(
                    "Camera not calibrated - using placeholder values!"
                )

            logger.info(f"Loaded calibration from {filepath}")
            return camera_matrix, dist_coeffs

        except Exception as e:
            logger.error(f"Failed to load calibration: {e}")
            return None, None

    def clear(self):
        """Clear all collected calibration data."""
        self.obj_points.clear()
        self.img_points.clear()
        self.image_size = None
        self.camera_matrix = None
        self.dist_coeffs = None
        self.reprojection_error = -1.0
        logger.info("Calibration data cleared")


def interactive_calibration(
    camera_id: int = 0,
    output_path: str = "config/camera_params.yaml",
    num_images: int = 20,
    chessboard_size: Tuple[int, int] = (9, 6),
    square_size_m: float = 0.025,
    wide_angle: bool = False
) -> bool:
    """
    Run interactive camera calibration.

    Args:
        camera_id: Camera device ID
        output_path: Path to save calibration
        num_images: Number of images to capture (20+ recommended)
        chessboard_size: Chessboard inner corners
        square_size_m: Square size in meters
        wide_angle: Use rational model for wide-angle lenses (FOV > 80°)

    Returns:
        True if calibration completed successfully
    """
    calibration = CameraCalibration(chessboard_size, square_size_m)
    cap = cv2.VideoCapture(camera_id)

    if not cap.isOpened():
        logger.error(f"Failed to open camera {camera_id}")
        return False

    model_name = "rational (8-coeff)" if wide_angle else "standard (5-coeff)"

    print(f"\n=== Camera Calibration ===")
    print(f"Chessboard size: {chessboard_size[0]}x{chessboard_size[1]} corners")
    print(f"Square size: {square_size_m * 100:.1f} cm")
    print(f"Distortion model: {model_name}")
    print(f"Images needed: {num_images}")
    print(f"\nTips for good calibration (<1px error):")
    print(f"  - Cover the entire frame (center, corners, edges)")
    print(f"  - Vary the angle (tilt board 20-45 degrees)")
    print(f"  - Vary the distance (near, medium, far)")
    print(f"  - At least {num_images} images, more is better")
    print(f"\nControls:")
    print(f"  SPACE - Capture image when chessboard is detected")
    print(f"  C     - Run calibration with current images")
    print(f"  R     - Reset and start over")
    print(f"  Q     - Quit\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        found, display_frame, _ = calibration.find_chessboard(frame)

        # Add status text
        status = f"Images: {len(calibration.img_points)}/{num_images}"
        if found:
            status += " - CHESSBOARD DETECTED (press SPACE)"
            color = (0, 255, 0)
        else:
            status += " - Point camera at chessboard"
            color = (0, 0, 255)

        cv2.putText(
            display_frame, status, (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2
        )

        cv2.imshow("Camera Calibration", display_frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord(' ') and found:
            if calibration.add_calibration_image(frame):
                print(f"Captured image {len(calibration.img_points)}")

                if len(calibration.img_points) >= num_images:
                    print("\nEnough images captured! Press 'C' to calibrate.")

        elif key == ord('c'):
            if calibration.calibrate(min_images=5, wide_angle=wide_angle):
                if calibration.save_calibration(output_path):
                    print(f"\nCalibration saved to {output_path}")
                    break
            else:
                print("Calibration failed - try capturing more images")

        elif key == ord('r'):
            calibration.clear()
            print("Calibration reset")

        elif key == ord('q'):
            print("Calibration cancelled")
            break

    cap.release()
    cv2.destroyAllWindows()

    return calibration.camera_matrix is not None


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    interactive_calibration()
