#!/usr/bin/env python3
"""Terminal mini map — curses-based live position viewer for ArUco Vision GPS.

Usage:
    python3 tools/terminal_map.py --host aruconav.local
"""

import argparse
import curses
import json
import math
import time
import threading
from collections import deque
from pathlib import Path
from urllib.request import urlopen
from urllib.error import URLError

import yaml


def load_markers(path):
    with open(path) as f:
        data = yaml.safe_load(f)
    return [{"id": m["id"], "x": m["position"][0], "y": m["position"][1]}
            for m in data.get("markers", [])]


YAW_ARROWS = [
    (0, "^"), (45, "/"), (90, ">"), (135, "\\"),
    (180, "v"), (225, "/"), (270, "<"), (315, "\\"),
]


def yaw_char(yaw_deg):
    yaw = yaw_deg % 360
    best = min(YAW_ARROWS, key=lambda a: min(abs(yaw - a[0]), 360 - abs(yaw - a[0])))
    return best[1]


class TerminalMap:
    def __init__(self, host, port, markers, poll_hz=10):
        self.url = f"http://{host}:{port}/position"
        self.markers = markers
        self.poll_hz = poll_hz
        self.data = None
        self.trail = deque(maxlen=80)
        self.connected = False
        self.error = ""
        self._running = False
        self._lock = threading.Lock()
        # World bounds (meters) — auto-fit markers with margin
        xs = [m["x"] for m in markers] + [0]
        ys = [m["y"] for m in markers] + [0]
        margin = 1.5
        self.bounds = [min(xs) - margin, max(xs) + margin,
                       min(ys) - margin, max(ys) + margin]

    def _poll(self):
        while self._running:
            try:
                resp = urlopen(self.url, timeout=0.5)
                d = json.loads(resp.read().decode())
                with self._lock:
                    self.data = d
                    self.connected = True
                    self.error = ""
                    if d.get("x") is not None:
                        x, y = d["x"], d["y"]
                        if not self.trail or (
                            (x - self.trail[-1][0])**2 + (y - self.trail[-1][1])**2 > 0.0001
                        ):
                            self.trail.append((x, y))
            except (URLError, Exception) as e:
                with self._lock:
                    self.connected = False
                    self.error = str(e)[:40]
            time.sleep(1.0 / self.poll_hz)

    def world_to_map(self, x, y, map_w, map_h):
        xmin, xmax, ymin, ymax = self.bounds
        col = int((x - xmin) / (xmax - xmin) * (map_w - 1))
        row = int((1 - (y - ymin) / (ymax - ymin)) * (map_h - 1))  # flip Y
        return max(0, min(map_h - 1, row)), max(0, min(map_w - 1, col))

    def zoom(self, factor):
        xmin, xmax, ymin, ymax = self.bounds
        cx, cy = (xmin + xmax) / 2, (ymin + ymax) / 2
        hw, hh = (xmax - xmin) / 2 * factor, (ymax - ymin) / 2 * factor
        self.bounds = [cx - hw, cx + hw, cy - hh, cy + hh]

    def run(self, stdscr):
        curses.curs_set(0)
        stdscr.nodelay(True)
        curses.start_color()
        curses.use_default_colors()
        curses.init_pair(1, curses.COLOR_GREEN, -1)   # drone
        curses.init_pair(2, curses.COLOR_RED, -1)      # markers
        curses.init_pair(3, curses.COLOR_YELLOW, -1)   # warning
        curses.init_pair(4, curses.COLOR_CYAN, -1)     # stats
        curses.init_pair(5, curses.COLOR_WHITE, -1)    # grid

        self._running = True
        t = threading.Thread(target=self._poll, daemon=True)
        t.start()

        while self._running:
            stdscr.erase()
            h, w = stdscr.getmaxyx()
            if h < 10 or w < 30:
                stdscr.addstr(0, 0, "Terminal too small")
                stdscr.refresh()
                time.sleep(0.1)
                continue

            stats_w = 22
            map_w = w - stats_w - 2
            map_h = h - 2  # leave top and bottom rows
            if map_w < 10:
                map_w = w - 2
                stats_w = 0

            with self._lock:
                data = dict(self.data) if self.data else None
                trail = list(self.trail)
                connected = self.connected
                error = self.error

            # Header
            status = "LIVE" if connected else "---"
            status_color = curses.color_pair(1) if connected else curses.color_pair(3)
            header = f" ArUco Map [{status}]"
            try:
                stdscr.addstr(0, 0, header[:w-1], curses.A_BOLD | status_color)
            except curses.error:
                pass

            # Draw origin crosshair
            or_, oc = self.world_to_map(0, 0, map_w, map_h)
            or_ += 1  # offset for header
            for c in range(max(0, oc - 2), min(map_w, oc + 3)):
                try:
                    stdscr.addch(or_, c, '-', curses.color_pair(5) | curses.A_DIM)
                except curses.error:
                    pass
            for r in range(1, map_h + 1):
                try:
                    stdscr.addch(r, oc, '|' if r != or_ else '+', curses.color_pair(5) | curses.A_DIM)
                except curses.error:
                    pass
            try:
                stdscr.addch(or_, oc, '+', curses.color_pair(5))
            except curses.error:
                pass

            # Draw markers
            for m in self.markers:
                mr, mc = self.world_to_map(m["x"], m["y"], map_w, map_h)
                mr += 1
                label = f"M{m['id']}"
                try:
                    stdscr.addstr(mr, mc, label[:map_w - mc], curses.color_pair(2) | curses.A_BOLD)
                except curses.error:
                    pass

            # Draw trail
            for i, (tx, ty) in enumerate(trail):
                tr, tc = self.world_to_map(tx, ty, map_w, map_h)
                tr += 1
                attr = curses.color_pair(1) | (curses.A_DIM if i < len(trail) - 5 else 0)
                try:
                    stdscr.addch(tr, tc, '.', attr)
                except curses.error:
                    pass

            # Draw drone
            if data and data.get("x") is not None:
                dr, dc = self.world_to_map(data["x"], data["y"], map_w, map_h)
                dr += 1
                yaw = data.get("yaw", 0)
                ch = yaw_char(yaw)
                try:
                    stdscr.addstr(dr, dc, ch, curses.color_pair(1) | curses.A_BOLD)
                except curses.error:
                    pass

            # Stats sidebar
            if stats_w > 0:
                sx = map_w + 1
                row = 1
                def stat(label, val, color=4):
                    nonlocal row
                    text = f"{label}: {val}"
                    try:
                        stdscr.addstr(row, sx, text[:stats_w], curses.color_pair(color))
                    except curses.error:
                        pass
                    row += 1

                stat("", "-- Stats --", 5)
                if not connected:
                    stat("", "DISCONNECTED", 3)
                    if error:
                        stat("", error[:stats_w], 3)
                elif data:
                    if data.get("x") is not None:
                        stat(" x", f"{data['x']:+.3f}m")
                        stat(" y", f"{data['y']:+.3f}m")
                        stat(" z", f"{data['z']:.3f}m")
                        stat("yaw", f"{data.get('yaw', 0):+.1f} deg")
                        stat("conf", f"{data.get('confidence', 0):.2f}")
                    else:
                        stat("", "NO FIX", 3)
                    det = data.get("detection_rate", 0)
                    stat("det", f"{det:.0%}", 1 if det > 0.5 else 3)
                    ids = data.get("marker_ids", [])
                    stat("mkr", ",".join(str(i) for i in ids) if ids else "none")
                    row += 1
                    stat("", f"frames: {data.get('frame_count', 0)}", 5)

            # Footer
            footer = " q:quit  c:clear  +/-:zoom"
            try:
                stdscr.addstr(h - 1, 0, footer[:w-1], curses.A_DIM)
            except curses.error:
                pass

            stdscr.refresh()

            # Input
            key = stdscr.getch()
            if key == ord("q"):
                break
            elif key == ord("c"):
                with self._lock:
                    self.trail.clear()
            elif key in (ord("+"), ord("=")):
                self.zoom(0.8)
            elif key == ord("-"):
                self.zoom(1.25)

            time.sleep(0.05)

        self._running = False


def main():
    parser = argparse.ArgumentParser(description="Terminal mini map for ArUco Vision GPS")
    parser.add_argument("--host", default="aruconav.local")
    parser.add_argument("--port", type=int, default=8001)
    parser.add_argument("--marker-map", default=None)
    args = parser.parse_args()

    marker_path = args.marker_map
    if marker_path is None:
        default = Path(__file__).parent.parent / "config" / "marker_map.yaml"
        if default.exists():
            marker_path = str(default)

    markers = load_markers(marker_path) if marker_path else []
    tm = TerminalMap(args.host, args.port, markers)

    try:
        curses.wrapper(tm.run)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
