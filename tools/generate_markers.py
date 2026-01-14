#!/usr/bin/env python3
"""
ArUco Marker Generator Tool

Generates printable ArUco marker images and PDFs for ceiling mounting.

Usage:
    python generate_markers.py [--ids 0,1,2,3,4] [--size 200] [--output markers/]
"""

import argparse
import sys
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

import cv2
import numpy as np

try:
    from reportlab.lib.pagesizes import A4, letter
    from reportlab.lib.units import mm, cm
    from reportlab.pdfgen import canvas
    REPORTLAB_AVAILABLE = True
except ImportError:
    REPORTLAB_AVAILABLE = False
    print("Warning: reportlab not installed, PDF generation disabled")

from src.aruco_detector import ARUCO_DICTIONARIES


def generate_marker_image(
    marker_id: int,
    size_pixels: int = 200,
    dictionary: str = "DICT_6X6_250",
    border_bits: int = 1
) -> np.ndarray:
    """
    Generate a single ArUco marker image.

    Args:
        marker_id: Marker ID
        size_pixels: Marker size in pixels
        dictionary: ArUco dictionary name
        border_bits: White border size in marker units

    Returns:
        Marker image as numpy array
    """
    if dictionary not in ARUCO_DICTIONARIES:
        raise ValueError(f"Unknown dictionary: {dictionary}")

    aruco_dict = cv2.aruco.getPredefinedDictionary(
        ARUCO_DICTIONARIES[dictionary]
    )

    marker = cv2.aruco.generateImageMarker(
        aruco_dict, marker_id, size_pixels
    )

    # Add white border
    if border_bits > 0:
        # Calculate border size based on marker internal size
        dict_size = int(dictionary.split('_')[1].split('X')[0])
        cell_size = size_pixels / (dict_size + 2)  # +2 for black border
        border_pixels = int(cell_size * border_bits)

        bordered = np.ones(
            (size_pixels + 2 * border_pixels, size_pixels + 2 * border_pixels),
            dtype=np.uint8
        ) * 255
        bordered[border_pixels:-border_pixels, border_pixels:-border_pixels] = marker
        marker = bordered

    return marker


def save_marker_image(
    marker_id: int,
    output_path: str,
    size_pixels: int = 200,
    dictionary: str = "DICT_6X6_250",
    add_label: bool = True
) -> str:
    """
    Generate and save a marker image.

    Args:
        marker_id: Marker ID
        output_path: Output file path
        size_pixels: Marker size in pixels
        dictionary: ArUco dictionary name
        add_label: Add ID label below marker

    Returns:
        Path to saved image
    """
    marker = generate_marker_image(marker_id, size_pixels, dictionary)

    if add_label:
        # Add label below marker
        label_height = 40
        labeled = np.ones(
            (marker.shape[0] + label_height, marker.shape[1]),
            dtype=np.uint8
        ) * 255
        labeled[:marker.shape[0], :] = marker

        # Add text
        label = f"ID: {marker_id}"
        font = cv2.FONT_HERSHEY_SIMPLEX
        text_size = cv2.getTextSize(label, font, 0.8, 2)[0]
        text_x = (marker.shape[1] - text_size[0]) // 2
        text_y = marker.shape[0] + label_height - 10

        cv2.putText(
            labeled, label, (text_x, text_y),
            font, 0.8, (0, 0, 0), 2
        )
        marker = labeled

    cv2.imwrite(output_path, marker)
    return output_path


def generate_marker_pdf(
    marker_ids: list,
    output_path: str,
    marker_size_cm: float = 20.0,
    dictionary: str = "DICT_6X6_250",
    page_size=A4,
    markers_per_page: int = 1
):
    """
    Generate PDF with printable markers.

    Args:
        marker_ids: List of marker IDs
        output_path: Output PDF path
        marker_size_cm: Physical marker size in cm
        dictionary: ArUco dictionary name
        page_size: Page size (A4, letter, etc.)
        markers_per_page: Markers per page (1 or 4)
    """
    if not REPORTLAB_AVAILABLE:
        print("PDF generation requires reportlab: pip install reportlab")
        return

    c = canvas.Canvas(output_path, pagesize=page_size)
    page_width, page_height = page_size

    marker_size_pts = marker_size_cm * cm

    for i, marker_id in enumerate(marker_ids):
        # Generate marker image
        marker = generate_marker_image(
            marker_id, 1000, dictionary
        )

        # Save temporary PNG
        temp_path = f"/tmp/marker_{marker_id}.png"
        cv2.imwrite(temp_path, marker)

        if markers_per_page == 1:
            # Center on page
            x = (page_width - marker_size_pts) / 2
            y = (page_height - marker_size_pts) / 2

            c.drawImage(
                temp_path, x, y,
                width=marker_size_pts, height=marker_size_pts
            )

            # Add label
            c.setFont("Helvetica-Bold", 24)
            label = f"ArUco ID: {marker_id}"
            label_width = c.stringWidth(label, "Helvetica-Bold", 24)
            c.drawString(
                (page_width - label_width) / 2,
                y - 40,
                label
            )

            # Add size info
            c.setFont("Helvetica", 12)
            info = f"Size: {marker_size_cm:.1f} cm x {marker_size_cm:.1f} cm | Dict: {dictionary}"
            info_width = c.stringWidth(info, "Helvetica", 12)
            c.drawString(
                (page_width - info_width) / 2,
                y - 60,
                info
            )

            c.showPage()

        else:  # 4 markers per page
            positions = [
                (50, page_height - 50 - marker_size_pts/2),
                (page_width/2 + 25, page_height - 50 - marker_size_pts/2),
                (50, page_height/2 - marker_size_pts/2),
                (page_width/2 + 25, page_height/2 - marker_size_pts/2),
            ]

            pos_idx = i % 4
            x, y = positions[pos_idx]
            smaller_size = marker_size_pts * 0.45

            c.drawImage(
                temp_path, x, y,
                width=smaller_size, height=smaller_size
            )

            c.setFont("Helvetica", 10)
            c.drawString(x, y - 15, f"ID: {marker_id}")

            if pos_idx == 3 or i == len(marker_ids) - 1:
                c.showPage()

        # Clean up temp file
        Path(temp_path).unlink(missing_ok=True)

    c.save()
    print(f"Saved PDF to {output_path}")


def main():
    parser = argparse.ArgumentParser(
        description="Generate printable ArUco markers"
    )
    parser.add_argument(
        '--ids', '-i',
        type=str,
        default='0,1,2,3,4',
        help='Comma-separated marker IDs (default: 0,1,2,3,4)'
    )
    parser.add_argument(
        '--dictionary', '-d',
        type=str,
        default='DICT_6X6_250',
        choices=list(ARUCO_DICTIONARIES.keys()),
        help='ArUco dictionary (default: DICT_6X6_250)'
    )
    parser.add_argument(
        '--size', '-s',
        type=float,
        default=20.0,
        help='Physical marker size in cm (default: 20)'
    )
    parser.add_argument(
        '--pixels',
        type=int,
        default=1000,
        help='Image resolution in pixels (default: 1000)'
    )
    parser.add_argument(
        '--output', '-o',
        type=str,
        default='markers/',
        help='Output directory (default: markers/)'
    )
    parser.add_argument(
        '--format', '-f',
        type=str,
        choices=['png', 'pdf', 'both'],
        default='both',
        help='Output format (default: both)'
    )
    parser.add_argument(
        '--per-page',
        type=int,
        choices=[1, 4],
        default=1,
        help='Markers per PDF page (default: 1)'
    )

    args = parser.parse_args()

    # Parse marker IDs
    try:
        marker_ids = [int(x.strip()) for x in args.ids.split(',')]
    except ValueError:
        print("Error: Invalid marker IDs. Use comma-separated integers.")
        sys.exit(1)

    # Create output directory
    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    print(f"\n{'='*50}")
    print("     ArUco MARKER GENERATOR")
    print(f"{'='*50}")
    print(f"\nDictionary: {args.dictionary}")
    print(f"Marker IDs: {marker_ids}")
    print(f"Physical size: {args.size} cm")
    print(f"Output: {output_dir}")
    print()

    # Generate PNG images
    if args.format in ['png', 'both']:
        print("Generating PNG images...")
        for marker_id in marker_ids:
            output_path = output_dir / f"marker_{marker_id}.png"
            save_marker_image(
                marker_id,
                str(output_path),
                size_pixels=args.pixels,
                dictionary=args.dictionary
            )
            print(f"  Created: {output_path}")

    # Generate PDF
    if args.format in ['pdf', 'both']:
        if REPORTLAB_AVAILABLE:
            print("\nGenerating PDF...")
            pdf_path = output_dir / f"markers_{args.dictionary}.pdf"
            generate_marker_pdf(
                marker_ids,
                str(pdf_path),
                marker_size_cm=args.size,
                dictionary=args.dictionary,
                markers_per_page=args.per_page
            )
        else:
            print("\nSkipping PDF (install reportlab: pip install reportlab)")

    print("\nDone!")
    print(f"\nPrint markers at 100% scale ({args.size}cm x {args.size}cm)")
    print("Mount flat on ceiling facing down")


if __name__ == "__main__":
    main()
