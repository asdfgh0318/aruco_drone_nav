#!/usr/bin/env python3
"""Test ArUco detection locally on a saved image."""

import cv2
import numpy as np
import sys

def test_detection(image_path: str):
    """Test detection with different parameters."""

    img = cv2.imread(image_path)
    if img is None:
        print(f"Failed to load: {image_path}")
        return

    print(f"Image size: {img.shape}")
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Test different dictionaries
    dictionaries = [
        ("DICT_4X4_50", cv2.aruco.DICT_4X4_50),
        ("DICT_4X4_100", cv2.aruco.DICT_4X4_100),
        ("DICT_5X5_50", cv2.aruco.DICT_5X5_50),
        ("DICT_6X6_50", cv2.aruco.DICT_6X6_50),
    ]

    print("\n=== Testing different dictionaries (default params) ===")
    for name, dict_id in dictionaries:
        aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, params)
        corners, ids, rejected = detector.detectMarkers(img)

        n_detected = len(ids) if ids is not None else 0
        n_rejected = len(rejected) if rejected is not None else 0
        id_list = ids.flatten().tolist() if ids is not None else []
        print(f"  {name}: {n_detected} detected {id_list}, {n_rejected} rejected")

    print("\n=== Testing DICT_4X4_50 with relaxed parameters ===")
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

    # Test with different parameter settings
    param_sets = [
        ("Default", {}),
        ("Lower minPerimeter", {"minMarkerPerimeterRate": 0.01}),
        ("Higher adaptiveThresh", {"adaptiveThreshConstant": 10}),
        ("More lenient polygon", {"polygonalApproxAccuracyRate": 0.08}),
        ("Corner refinement OFF", {"cornerRefinementMethod": cv2.aruco.CORNER_REFINE_NONE}),
        ("All relaxed", {
            "minMarkerPerimeterRate": 0.01,
            "adaptiveThreshConstant": 10,
            "polygonalApproxAccuracyRate": 0.08,
            "minCornerDistanceRate": 0.01,
        }),
    ]

    for name, param_overrides in param_sets:
        params = cv2.aruco.DetectorParameters()
        for k, v in param_overrides.items():
            setattr(params, k, v)

        detector = cv2.aruco.ArucoDetector(aruco_dict, params)
        corners, ids, rejected = detector.detectMarkers(img)

        n_detected = len(ids) if ids is not None else 0
        n_rejected = len(rejected) if rejected is not None else 0
        id_list = ids.flatten().tolist() if ids is not None else []
        print(f"  {name}: {n_detected} detected {id_list}, {n_rejected} rejected")

        # If we get 4, save the result
        if n_detected >= 4:
            output = img.copy()
            cv2.aruco.drawDetectedMarkers(output, corners, ids)
            out_path = f"detection_success_{name.replace(' ', '_')}.jpg"
            cv2.imwrite(out_path, output)
            print(f"    -> Saved to {out_path}")

    # Test on grayscale with histogram equalization
    print("\n=== Testing with preprocessing ===")
    gray_eq = cv2.equalizeHist(gray)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    gray_clahe = clahe.apply(gray)

    for name, test_img in [("Grayscale", gray), ("Histogram EQ", gray_eq), ("CLAHE", gray_clahe)]:
        params = cv2.aruco.DetectorParameters()
        params.minMarkerPerimeterRate = 0.01
        params.adaptiveThreshConstant = 10
        detector = cv2.aruco.ArucoDetector(aruco_dict, params)

        # Convert back to BGR for detection
        test_bgr = cv2.cvtColor(test_img, cv2.COLOR_GRAY2BGR)
        corners, ids, rejected = detector.detectMarkers(test_bgr)

        n_detected = len(ids) if ids is not None else 0
        id_list = ids.flatten().tolist() if ids is not None else []
        print(f"  {name}: {n_detected} detected {id_list}")


if __name__ == "__main__":
    if len(sys.argv) > 1:
        test_detection(sys.argv[1])
    else:
        # Find latest debug image
        import glob
        images = sorted(glob.glob("/home/adam/ŻYCIE/PRACA/aruco_drone_nav/debug_*.jpg"))
        if images:
            test_detection(images[-1])
        else:
            print("No debug images found")
