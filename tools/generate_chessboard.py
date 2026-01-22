#!/usr/bin/env python3
"""
Chessboard Pattern Generator for Camera Calibration

Generates a printable chessboard pattern for camera calibration.

Usage:
    python generate_chessboard.py [--width 9] [--height 6] [--square-size 25]
"""

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

try:
    from reportlab.lib.pagesizes import A4
    from reportlab.lib.units import mm
    from reportlab.pdfgen import canvas
    REPORTLAB_AVAILABLE = True
except ImportError:
    REPORTLAB_AVAILABLE = False


def generate_chessboard_image(
    width: int = 9,
    height: int = 6,
    square_size_px: int = 100
) -> np.ndarray:
    """
    Generate a chessboard pattern image.

    Args:
        width: Number of inner corners horizontally
        height: Number of inner corners vertically
        square_size_px: Size of each square in pixels

    Returns:
        Chessboard image as numpy array
    """
    # Board dimensions (one more square than corners in each direction)
    board_w = (width + 1) * square_size_px
    board_h = (height + 1) * square_size_px

    # Create white image
    board = np.ones((board_h, board_w), dtype=np.uint8) * 255

    # Draw black squares
    for row in range(height + 1):
        for col in range(width + 1):
            if (row + col) % 2 == 0:
                x1 = col * square_size_px
                y1 = row * square_size_px
                x2 = x1 + square_size_px
                y2 = y1 + square_size_px
                board[y1:y2, x1:x2] = 0

    return board


def generate_chessboard_pdf(
    output_path: str,
    width: int = 9,
    height: int = 6,
    square_size_mm: float = 25.0,
    page_size=A4
):
    """
    Generate a PDF with printable chessboard pattern.

    Args:
        output_path: Output PDF path
        width: Number of inner corners horizontally
        height: Number of inner corners vertically
        square_size_mm: Size of each square in millimeters
        page_size: Page size tuple
    """
    if not REPORTLAB_AVAILABLE:
        print("PDF generation requires reportlab: pip install reportlab")
        return False

    c = canvas.Canvas(output_path, pagesize=page_size)
    page_width, page_height = page_size

    # Board dimensions
    board_w_mm = (width + 1) * square_size_mm
    board_h_mm = (height + 1) * square_size_mm

    # Check if it fits on page (with margins)
    max_w = (page_width / mm) - 20  # 10mm margins
    max_h = (page_height / mm) - 40  # Extra for title

    if board_w_mm > max_w or board_h_mm > max_h:
        print(f"Warning: Board ({board_w_mm:.0f}x{board_h_mm:.0f}mm) may not fit on page")
        scale = min(max_w / board_w_mm, max_h / board_h_mm)
        square_size_mm *= scale
        board_w_mm = (width + 1) * square_size_mm
        board_h_mm = (height + 1) * square_size_mm
        print(f"Scaled to {square_size_mm:.1f}mm squares")

    # Center on page
    start_x = (page_width - board_w_mm * mm) / 2
    start_y = (page_height - board_h_mm * mm) / 2 - 10 * mm  # Offset for title

    # Draw squares
    for row in range(height + 1):
        for col in range(width + 1):
            x = start_x + col * square_size_mm * mm
            y = start_y + (height - row) * square_size_mm * mm

            if (row + col) % 2 == 0:
                c.setFillColorRGB(0, 0, 0)
            else:
                c.setFillColorRGB(1, 1, 1)

            c.rect(x, y, square_size_mm * mm, square_size_mm * mm, fill=1, stroke=0)

    # Draw border
    c.setStrokeColorRGB(0, 0, 0)
    c.setLineWidth(1)
    c.rect(start_x, start_y, board_w_mm * mm, board_h_mm * mm, fill=0, stroke=1)

    # Title
    c.setFillColorRGB(0, 0, 0)
    c.setFont("Helvetica-Bold", 16)
    title = "Camera Calibration Chessboard"
    title_width = c.stringWidth(title, "Helvetica-Bold", 16)
    c.drawString((page_width - title_width) / 2, start_y + board_h_mm * mm + 15 * mm, title)

    # Info
    c.setFont("Helvetica", 10)
    info = f"Inner corners: {width} x {height} | Square size: {square_size_mm:.1f} mm"
    info_width = c.stringWidth(info, "Helvetica", 10)
    c.drawString((page_width - info_width) / 2, start_y - 8 * mm, info)

    c.setFont("Helvetica", 8)
    note = "Print at 100% scale (no scaling). Measure squares to verify size."
    note_width = c.stringWidth(note, "Helvetica", 8)
    c.drawString((page_width - note_width) / 2, start_y - 14 * mm, note)

    c.save()
    return True


def main():
    parser = argparse.ArgumentParser(
        description="Generate chessboard pattern for camera calibration"
    )
    parser.add_argument(
        '--width', '-w',
        type=int,
        default=9,
        help='Number of inner corners horizontally (default: 9)'
    )
    parser.add_argument(
        '--height',
        type=int,
        default=6,
        help='Number of inner corners vertically (default: 6)'
    )
    parser.add_argument(
        '--square-size', '-s',
        type=float,
        default=25.0,
        help='Square size in mm (default: 25)'
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

    args = parser.parse_args()

    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    print(f"\n{'='*50}")
    print("     CHESSBOARD GENERATOR")
    print(f"{'='*50}")
    print(f"\nInner corners: {args.width} x {args.height}")
    print(f"Square size: {args.square_size} mm")
    print(f"Output: {output_dir}")
    print()

    # Generate PNG
    if args.format in ['png', 'both']:
        print("Generating PNG image...")
        # Use high DPI for printing (300 DPI = ~12 pixels per mm)
        px_per_mm = 12
        square_px = int(args.square_size * px_per_mm)
        board = generate_chessboard_image(args.width, args.height, square_px)

        png_path = output_dir / f"chessboard_{args.width}x{args.height}.png"
        cv2.imwrite(str(png_path), board)
        print(f"  Created: {png_path}")

    # Generate PDF
    if args.format in ['pdf', 'both']:
        if REPORTLAB_AVAILABLE:
            print("Generating PDF...")
            pdf_path = output_dir / f"chessboard_{args.width}x{args.height}.pdf"
            if generate_chessboard_pdf(
                str(pdf_path),
                args.width,
                args.height,
                args.square_size
            ):
                print(f"  Created: {pdf_path}")
        else:
            print("Skipping PDF (install reportlab)")

    print("\nDone!")
    print(f"\nFor calibration use: --board-width {args.width} --board-height {args.height} --square-size {args.square_size / 1000:.4f}")


if __name__ == "__main__":
    main()
