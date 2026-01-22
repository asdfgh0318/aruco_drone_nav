#!/usr/bin/env python3
"""
ArUco Drone Navigation - GUI Configurator

Simple GUI application for testing and configuring the drone navigation system.
Provides easy access to all testing tools and configuration options.

Usage:
    python configurator_gui.py
"""

import sys
import os
import subprocess
import threading
import queue
import yaml
from pathlib import Path
from datetime import datetime

import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox, filedialog

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))


class ProcessRunner:
    """Runs subprocesses and captures output."""

    def __init__(self, output_callback):
        self.output_callback = output_callback
        self.process = None
        self.running = False

    def run(self, cmd, cwd=None):
        """Run a command in a separate thread."""
        if self.running:
            self.output_callback("A process is already running!\n")
            return

        self.running = True
        thread = threading.Thread(target=self._run_process, args=(cmd, cwd))
        thread.daemon = True
        thread.start()

    def _run_process(self, cmd, cwd):
        """Internal method to run process."""
        try:
            self.output_callback(f"$ {' '.join(cmd)}\n")
            self.output_callback("-" * 50 + "\n")

            self.process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                cwd=cwd,
                text=True,
                bufsize=1
            )

            for line in iter(self.process.stdout.readline, ''):
                if not self.running:
                    break
                self.output_callback(line)

            self.process.wait()
            self.output_callback("-" * 50 + "\n")
            self.output_callback(f"Process exited with code {self.process.returncode}\n\n")

        except Exception as e:
            self.output_callback(f"Error: {e}\n")

        finally:
            self.running = False
            self.process = None

    def stop(self):
        """Stop the running process."""
        if self.process and self.running:
            self.running = False
            self.process.terminate()
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.kill()


class ConfiguratorGUI:
    """Main GUI application."""

    def __init__(self):
        self.root = tk.Tk()
        self.root.title("ArUco Drone Navigation - Configurator")
        self.root.geometry("900x700")
        self.root.minsize(800, 600)

        # Paths
        self.project_dir = Path(__file__).parent.parent
        self.config_path = self.project_dir / "config" / "system_config.yaml"
        self.venv_python = self.project_dir / "venv" / "bin" / "python3"

        # If venv doesn't exist, use system python
        if not self.venv_python.exists():
            self.venv_python = "python3"
        else:
            self.venv_python = str(self.venv_python)

        # Process runner
        self.runner = ProcessRunner(self._append_output)

        # Configuration
        self.config = self._load_config()

        # Build UI
        self._create_menu()
        self._create_main_layout()

        # Status bar
        self._create_status_bar()

        self._log(f"Project directory: {self.project_dir}")
        self._log(f"Python: {self.venv_python}")
        self._log("Ready.\n")

    def _load_config(self) -> dict:
        """Load configuration from YAML."""
        default_config = {
            'camera': {'device_id': 0, 'width': 640, 'height': 480},
            'aruco': {'dictionary': 'DICT_6X6_250', 'marker_size_m': 0.20},
            'serial': {'port': '/dev/serial0', 'baud': 921600},
            'calibration': {
                'board_width': 9,
                'board_height': 6,
                'square_size_m': 0.025
            }
        }

        if self.config_path.exists():
            try:
                with open(self.config_path) as f:
                    loaded = yaml.safe_load(f)
                    if loaded:
                        # Merge with defaults
                        for key in default_config:
                            if key in loaded:
                                default_config[key].update(loaded[key])
            except Exception as e:
                print(f"Error loading config: {e}")

        return default_config

    def _save_config(self):
        """Save configuration to YAML."""
        try:
            # Update config from UI
            self.config['camera']['device_id'] = int(self.camera_id_var.get())
            self.config['camera']['width'] = int(self.resolution_var.get().split('x')[0])
            self.config['camera']['height'] = int(self.resolution_var.get().split('x')[1])
            self.config['aruco']['marker_size_m'] = float(self.marker_size_var.get()) / 100
            self.config['serial']['port'] = self.serial_port_var.get()
            self.config['serial']['baud'] = int(self.baud_rate_var.get())

            self.config_path.parent.mkdir(parents=True, exist_ok=True)
            with open(self.config_path, 'w') as f:
                yaml.dump(self.config, f, default_flow_style=False)

            self._log(f"Configuration saved to {self.config_path}\n")
            messagebox.showinfo("Saved", "Configuration saved successfully!")

        except Exception as e:
            messagebox.showerror("Error", f"Failed to save config: {e}")

    def _create_menu(self):
        """Create menu bar."""
        menubar = tk.Menu(self.root)
        self.root.config(menu=menubar)

        # File menu
        file_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="File", menu=file_menu)
        file_menu.add_command(label="Save Config", command=self._save_config)
        file_menu.add_command(label="Reload Config", command=self._reload_config)
        file_menu.add_separator()
        file_menu.add_command(label="Open Project Folder", command=self._open_project_folder)
        file_menu.add_separator()
        file_menu.add_command(label="Exit", command=self._on_close)

        # Tools menu
        tools_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Tools", menu=tools_menu)
        tools_menu.add_command(label="Generate Markers", command=self._generate_markers)
        tools_menu.add_command(label="Generate Chessboard", command=self._generate_chessboard)
        tools_menu.add_separator()
        tools_menu.add_command(label="Open Markers Folder", command=self._open_markers_folder)

        # Help menu
        help_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Help", menu=help_menu)
        help_menu.add_command(label="Documentation", command=self._show_docs)
        help_menu.add_command(label="About", command=self._show_about)

    def _create_main_layout(self):
        """Create main layout."""
        # Main container
        main = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        main.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Left panel - Controls
        left_frame = ttk.Frame(main, width=350)
        main.add(left_frame, weight=1)

        # Right panel - Output
        right_frame = ttk.Frame(main)
        main.add(right_frame, weight=2)

        self._create_control_panel(left_frame)
        self._create_output_panel(right_frame)

    def _create_control_panel(self, parent):
        """Create control panel with settings and buttons."""
        canvas = tk.Canvas(parent)
        scrollbar = ttk.Scrollbar(parent, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)

        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )

        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        # === Camera Settings ===
        cam_frame = ttk.LabelFrame(scrollable_frame, text="Camera Settings", padding=10)
        cam_frame.pack(fill=tk.X, padx=5, pady=5)

        ttk.Label(cam_frame, text="Camera ID:").grid(row=0, column=0, sticky=tk.W, pady=2)
        self.camera_id_var = tk.StringVar(value=str(self.config['camera']['device_id']))
        ttk.Spinbox(cam_frame, from_=0, to=10, textvariable=self.camera_id_var, width=10).grid(row=0, column=1, sticky=tk.W, pady=2)

        ttk.Label(cam_frame, text="Resolution:").grid(row=1, column=0, sticky=tk.W, pady=2)
        self.resolution_var = tk.StringVar(value=f"{self.config['camera']['width']}x{self.config['camera']['height']}")
        res_combo = ttk.Combobox(cam_frame, textvariable=self.resolution_var, width=12)
        res_combo['values'] = ('320x240', '640x480', '800x600', '1280x720', '1920x1080')
        res_combo.grid(row=1, column=1, sticky=tk.W, pady=2)

        # === ArUco Settings ===
        aruco_frame = ttk.LabelFrame(scrollable_frame, text="ArUco Settings", padding=10)
        aruco_frame.pack(fill=tk.X, padx=5, pady=5)

        ttk.Label(aruco_frame, text="Marker Size (cm):").grid(row=0, column=0, sticky=tk.W, pady=2)
        self.marker_size_var = tk.StringVar(value=str(int(self.config['aruco']['marker_size_m'] * 100)))
        ttk.Spinbox(aruco_frame, from_=1, to=100, textvariable=self.marker_size_var, width=10).grid(row=0, column=1, sticky=tk.W, pady=2)

        ttk.Label(aruco_frame, text="Dictionary:").grid(row=1, column=0, sticky=tk.W, pady=2)
        self.dict_var = tk.StringVar(value=self.config['aruco']['dictionary'])
        dict_combo = ttk.Combobox(aruco_frame, textvariable=self.dict_var, width=15)
        dict_combo['values'] = ('DICT_4X4_50', 'DICT_4X4_100', 'DICT_5X5_50', 'DICT_5X5_100', 'DICT_6X6_250', 'DICT_7X7_250')
        dict_combo.grid(row=1, column=1, sticky=tk.W, pady=2)

        # === Serial Settings ===
        serial_frame = ttk.LabelFrame(scrollable_frame, text="MAVLink Serial", padding=10)
        serial_frame.pack(fill=tk.X, padx=5, pady=5)

        ttk.Label(serial_frame, text="Port:").grid(row=0, column=0, sticky=tk.W, pady=2)
        self.serial_port_var = tk.StringVar(value=self.config['serial']['port'])
        port_combo = ttk.Combobox(serial_frame, textvariable=self.serial_port_var, width=15)
        port_combo['values'] = ('/dev/serial0', '/dev/ttyS0', '/dev/ttyAMA0', '/dev/ttyUSB0', '/dev/ttyACM0', 'udp:127.0.0.1:14550')
        port_combo.grid(row=0, column=1, sticky=tk.W, pady=2)

        ttk.Label(serial_frame, text="Baud Rate:").grid(row=1, column=0, sticky=tk.W, pady=2)
        self.baud_rate_var = tk.StringVar(value=str(self.config['serial']['baud']))
        baud_combo = ttk.Combobox(serial_frame, textvariable=self.baud_rate_var, width=15)
        baud_combo['values'] = ('57600', '115200', '230400', '460800', '921600')
        baud_combo.grid(row=1, column=1, sticky=tk.W, pady=2)

        # === Test Tools ===
        tools_frame = ttk.LabelFrame(scrollable_frame, text="Test Tools", padding=10)
        tools_frame.pack(fill=tk.X, padx=5, pady=5)

        btn_style = {'width': 25, 'padding': 5}

        ttk.Button(tools_frame, text="Camera Calibration", command=self._run_calibration, **btn_style).pack(pady=3)
        ttk.Button(tools_frame, text="ArUco Detection Test", command=self._run_aruco_test, **btn_style).pack(pady=3)
        ttk.Button(tools_frame, text="Bench Test (Position + Cmds)", command=self._run_bench_test, **btn_style).pack(pady=3)
        ttk.Button(tools_frame, text="MAVLink Connection Test", command=self._run_mavlink_test, **btn_style).pack(pady=3)

        ttk.Separator(tools_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=10)

        ttk.Button(tools_frame, text="Ground Test Mode", command=self._run_ground_test, **btn_style).pack(pady=3)

        # === Control Buttons ===
        ctrl_frame = ttk.LabelFrame(scrollable_frame, text="Control", padding=10)
        ctrl_frame.pack(fill=tk.X, padx=5, pady=5)

        self.stop_btn = ttk.Button(ctrl_frame, text="STOP PROCESS", command=self._stop_process, style='Danger.TButton')
        self.stop_btn.pack(fill=tk.X, pady=5)

        ttk.Button(ctrl_frame, text="Clear Output", command=self._clear_output).pack(fill=tk.X, pady=2)
        ttk.Button(ctrl_frame, text="Save Config", command=self._save_config).pack(fill=tk.X, pady=2)

    def _create_output_panel(self, parent):
        """Create output panel with log viewer."""
        # Header
        header = ttk.Frame(parent)
        header.pack(fill=tk.X)

        ttk.Label(header, text="Output Log", font=('TkDefaultFont', 10, 'bold')).pack(side=tk.LEFT, padx=5)

        # Output text area
        self.output_text = scrolledtext.ScrolledText(
            parent,
            wrap=tk.WORD,
            font=('Consolas', 9),
            bg='#1e1e1e',
            fg='#d4d4d4',
            insertbackground='white'
        )
        self.output_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Configure tags for colored output
        self.output_text.tag_configure('error', foreground='#f44747')
        self.output_text.tag_configure('success', foreground='#6a9955')
        self.output_text.tag_configure('info', foreground='#569cd6')
        self.output_text.tag_configure('command', foreground='#dcdcaa')

    def _create_status_bar(self):
        """Create status bar at bottom."""
        self.status_var = tk.StringVar(value="Ready")
        status_bar = ttk.Label(
            self.root,
            textvariable=self.status_var,
            relief=tk.SUNKEN,
            anchor=tk.W,
            padding=(5, 2)
        )
        status_bar.pack(side=tk.BOTTOM, fill=tk.X)

    def _log(self, message, tag=None):
        """Add message to output log."""
        self.output_text.insert(tk.END, message, tag)
        self.output_text.see(tk.END)

    def _append_output(self, text):
        """Thread-safe output append."""
        self.root.after(0, lambda: self._log(text))

    def _clear_output(self):
        """Clear output text."""
        self.output_text.delete(1.0, tk.END)

    def _get_cmd_base(self):
        """Get base command with python path."""
        return [self.venv_python]

    def _run_calibration(self):
        """Run camera calibration tool."""
        cmd = self._get_cmd_base() + [
            'tools/calibrate_camera.py',
            '--camera', self.camera_id_var.get(),
            '--board-width', str(self.config['calibration']['board_width']),
            '--board-height', str(self.config['calibration']['board_height']),
            '--square-size', str(self.config['calibration']['square_size_m'])
        ]
        self.status_var.set("Running camera calibration...")
        self.runner.run(cmd, cwd=str(self.project_dir))

    def _run_aruco_test(self):
        """Run ArUco detection test."""
        marker_size = float(self.marker_size_var.get()) / 100
        res = self.resolution_var.get().split('x')

        cmd = self._get_cmd_base() + [
            'tools/test_aruco_detection.py',
            '--camera', self.camera_id_var.get(),
            '--marker-size', str(marker_size),
            '--dictionary', self.dict_var.get(),
            '--width', res[0],
            '--height', res[1]
        ]
        self.status_var.set("Running ArUco detection test...")
        self.runner.run(cmd, cwd=str(self.project_dir))

    def _run_bench_test(self):
        """Run bench test tool."""
        marker_size = float(self.marker_size_var.get()) / 100
        res = self.resolution_var.get().split('x')

        cmd = self._get_cmd_base() + [
            'tools/bench_test.py',
            '--camera', self.camera_id_var.get(),
            '--marker-size', str(marker_size),
            '--dictionary', self.dict_var.get(),
            '--width', res[0],
            '--height', res[1]
        ]
        self.status_var.set("Running bench test...")
        self.runner.run(cmd, cwd=str(self.project_dir))

    def _run_mavlink_test(self):
        """Run MAVLink connection test."""
        cmd = self._get_cmd_base() + [
            'tools/test_mavlink.py',
            '--port', self.serial_port_var.get(),
            '--baud', self.baud_rate_var.get(),
            '--simple'
        ]
        self.status_var.set("Running MAVLink test...")
        self.runner.run(cmd, cwd=str(self.project_dir))

    def _run_ground_test(self):
        """Run main system in ground test mode."""
        cmd = self._get_cmd_base() + [
            '-m', 'src.main',
            '--mode', 'ground_test',
            '--config', 'config/system_config.yaml'
        ]
        self.status_var.set("Running ground test mode...")
        self.runner.run(cmd, cwd=str(self.project_dir))

    def _stop_process(self):
        """Stop running process."""
        self.runner.stop()
        self.status_var.set("Process stopped")
        self._log("\n[STOPPED BY USER]\n\n", 'error')

    def _generate_markers(self):
        """Generate ArUco markers."""
        cmd = self._get_cmd_base() + [
            'tools/generate_markers.py',
            '--ids', '0,1,2,3,4,5,6,7,8,9',
            '--size', self.marker_size_var.get(),
            '--dictionary', self.dict_var.get(),
            '--output', 'markers/'
        ]
        self.runner.run(cmd, cwd=str(self.project_dir))

    def _generate_chessboard(self):
        """Generate calibration chessboard."""
        cmd = self._get_cmd_base() + [
            'tools/generate_chessboard.py',
            '--width', str(self.config['calibration']['board_width']),
            '--height', str(self.config['calibration']['board_height']),
            '--square-size', str(int(self.config['calibration']['square_size_m'] * 1000)),
            '--output', 'markers/'
        ]
        self.runner.run(cmd, cwd=str(self.project_dir))

    def _open_project_folder(self):
        """Open project folder in file manager."""
        subprocess.Popen(['xdg-open', str(self.project_dir)])

    def _open_markers_folder(self):
        """Open markers folder."""
        markers_dir = self.project_dir / 'markers'
        markers_dir.mkdir(exist_ok=True)
        subprocess.Popen(['xdg-open', str(markers_dir)])

    def _reload_config(self):
        """Reload configuration from file."""
        self.config = self._load_config()
        # Update UI
        self.camera_id_var.set(str(self.config['camera']['device_id']))
        self.resolution_var.set(f"{self.config['camera']['width']}x{self.config['camera']['height']}")
        self.marker_size_var.set(str(int(self.config['aruco']['marker_size_m'] * 100)))
        self.serial_port_var.set(self.config['serial']['port'])
        self.baud_rate_var.set(str(self.config['serial']['baud']))
        self._log("Configuration reloaded\n", 'info')

    def _show_docs(self):
        """Show documentation."""
        readme = self.project_dir / 'README.md'
        if readme.exists():
            subprocess.Popen(['xdg-open', str(readme)])
        else:
            messagebox.showinfo("Documentation", "See README.md in project folder")

    def _show_about(self):
        """Show about dialog."""
        messagebox.showinfo(
            "About",
            "ArUco Drone Navigation System\n\n"
            "Indoor drone navigation using ceiling-mounted ArUco markers.\n\n"
            "Warsaw University of Technology\n"
            "2024"
        )

    def _on_close(self):
        """Handle window close."""
        if self.runner.running:
            if messagebox.askyesno("Confirm", "A process is still running. Stop and exit?"):
                self.runner.stop()
            else:
                return
        self.root.destroy()

    def run(self):
        """Run the application."""
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.mainloop()


def main():
    app = ConfiguratorGUI()
    app.run()


if __name__ == "__main__":
    main()
