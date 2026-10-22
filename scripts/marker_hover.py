#!/usr/bin/env python3
"""One window for buffer calibration and the repeated marker-hover task."""

import argparse
from pathlib import Path
import queue
import sys
import threading
import tkinter as tk
from tkinter import ttk

import numpy as np
from PIL import Image, ImageTk

ROOT = Path(__file__).resolve().parents[1]
if not __package__:
    sys.path.insert(0, str(ROOT))

from auto_reset.calibration import BufferCalibration, capture_calibration
from auto_reset.calibration_report import CalibrationReport
from auto_reset.camera import ZedWristCamera
from auto_reset.config import StationConfig
from auto_reset.motion import HoverPlanner, MotionCancelled
from auto_reset.robot import DroidRobot
from auto_reset.task import EventLog, MarkerHoverTask, new_log_path


class Window:
    def __init__(self, root, config_path):
        self.root, self.config_path = root, config_path
        self.jobs, self.events = queue.Queue(), queue.Queue()
        self.stop = threading.Event()
        self.busy, self.ready, self.closing = False, False, False
        self.calibration = None
        self.raw_image = self.annotated_image = None
        self.is_stereo = False
        root.title("DroidAuto — buffer calibration and marker hover")
        root.geometry("1100x850")
        self.status = tk.StringVar(value="Loading saved calibration...")
        self.raw = tk.BooleanVar(value=False)
        self.eye = tk.StringVar(value="Both")
        self.one_cycle = tk.BooleanVar(value=False)
        controls = ttk.Frame(root, padding=10)
        controls.pack(fill="x")
        self.initialize = ttk.Button(controls, text="Initialize controllers", command=lambda: self.submit("initialize"))
        self.calibrate = ttk.Button(controls, text="Calibrate / Recalibrate", command=lambda: self.submit("calibrate"))
        self.start = ttk.Button(controls, text="Start loop", command=lambda: self.submit("run"))
        self.stop_button = ttk.Button(controls, text="Stop", command=self.request_stop)
        for button in (self.initialize, self.calibrate, self.start, self.stop_button):
            button.pack(side="left", padx=4)
        self.single_toggle = ttk.Checkbutton(controls, text="One cycle", variable=self.one_cycle)
        self.single_toggle.pack(side="left", padx=10)
        ttk.Checkbutton(controls, text="Raw image", variable=self.raw, command=self.draw_image).pack(side="left")
        selector = ttk.Combobox(controls, textvariable=self.eye, values=("Both", "Left", "Right"),
                                state="readonly", width=6)
        selector.pack(side="left", padx=4)
        selector.bind("<<ComboboxSelected>>", lambda _: self.draw_image())
        ttk.Label(root, textvariable=self.status, wraplength=1050, padding=10).pack(fill="x")
        self.preview = ttk.Label(root, text="Calibrate to capture and display the wrist-camera image.", anchor="center")
        self.preview.pack(fill="both", expand=True)
        ttk.Label(root, text=str(config_path), padding=5).pack(fill="x")
        root.protocol("WM_DELETE_WINDOW", self.close)
        threading.Thread(target=self.worker, daemon=True).start()
        self.submit("load")
        root.after(75, self.poll)

    def buttons(self):
        for button in (self.initialize, self.calibrate):
            button.configure(state="disabled" if self.busy else "normal")
        self.start.configure(state="normal" if self.ready and not self.busy else "disabled")
        self.stop_button.configure(state="normal" if self.busy else "disabled")
        self.single_toggle.configure(state="disabled" if self.busy else "normal")

    def submit(self, action):
        if self.busy:
            return
        self.busy = True
        self.stop.clear()
        self.status.set({"load": "Loading saved calibration...", "initialize": "Initializing controllers...",
                         "calibrate": "Capturing and localizing buffer...", "run": "Starting hover loop..."}[action])
        self.buttons()
        self.jobs.put((action, self.one_cycle.get()))

    def request_stop(self):
        self.stop.set()
        self.status.set("Stop requested; waiting for the current operation to return...")

    def close(self):
        if self.busy:
            self.closing = True
            self.request_stop()
        else:
            self.jobs.put(None)
            self.root.destroy()

    def draw_image(self):
        image = self.raw_image if self.raw.get() else self.annotated_image
        if image is None:
            return
        if self.is_stereo and self.eye.get() != "Both":
            middle = image.shape[1] // 2
            image = image[:, :middle] if self.eye.get() == "Left" else image[:, middle:]
        picture = Image.fromarray(image)
        picture.thumbnail((1060, 650))
        self.photo = ImageTk.PhotoImage(picture)
        self.preview.configure(image=self.photo, text="")

    def poll(self):
        try:
            while True:
                event, payload = self.events.get_nowait()
                if event == "image":
                    self.raw_image, self.annotated_image, self.is_stereo = payload
                    self.draw_image()
                elif event == "status":
                    self.status.set(payload)
                elif event == "done":
                    self.busy, self.ready = False, payload
                    self.buttons()
                    if self.closing:
                        self.close()
                        return
        except queue.Empty:
            pass
        self.root.after(75, self.poll)

    def worker(self):
        # ZeroRPC is created, used and closed in this single worker thread.
        while True:
            job = self.jobs.get()
            if job is None:
                return
            action, single = job
            try:
                config = StationConfig(self.config_path)
                path = config.resolve(config.data["calibration_file"])
                if action == "load":
                    if path.exists():
                        self.calibration = BufferCalibration.load(path)
                        HoverPlanner(config, self.calibration)
                        image_path = path.parent / self.calibration.record["source_image"]
                        report_path = Path(self.calibration.record.get("diagnostics_directory", path.parent / "missing"))
                        if (report_path / "input.png").exists() and (report_path / "annotated.png").exists():
                            raw = np.array(Image.open(report_path / "input.png").convert("RGB"))
                            annotated = np.array(Image.open(report_path / "annotated.png").convert("RGB"))
                            self.events.put(("image", (raw, annotated, len(self.calibration.record.get("eyes", {})) == 2)))
                        elif image_path.exists():
                            image = np.array(Image.open(image_path).convert("RGB"))
                            self.events.put(("image", (image, image, False)))
                        self.events.put(("status", "Saved calibration loaded. Start loop, or click Recalibrate."))
                    else:
                        self.events.put(("status", "Click Calibrate to capture the buffer."))
                elif action == "calibrate":
                    self.calibration = None
                    self.capture(config, path)
                else:
                    with DroidRobot(config.data["robot_endpoint"]) as robot:
                        if action == "initialize":
                            robot.initialize()
                            robot.read_calibration_pose()
                            self.events.put(("status", "Controllers ready. Calibrate or start the loop."))
                        elif action == "run":
                            self.run_task(config, robot, single)
            except (MotionCancelled, KeyboardInterrupt):
                self.events.put(("status", "Stopped."))
            except Exception as exc:
                if action in ("load", "calibrate"):
                    self.calibration = None
                self.events.put(("status", f"{type(exc).__name__}: {exc}"))
            finally:
                self.events.put(("done", self.calibration is not None))

    def capture(self, config, path):
        report = CalibrationReport(config.resolve(config.data["log_directory"]), show_image=False)
        try:
            with DroidRobot(config.data["robot_endpoint"]) as robot:
                with ZedWristCamera(str(config.camera["serial"]), config.camera["view"]) as camera:
                    calibration = capture_calibration(config, robot, camera, report=report)
            if self.stop.is_set():
                raise MotionCancelled("Calibration cancelled")
            HoverPlanner(config, calibration)
            calibration.save(path)
            self.calibration = calibration
        except BaseException as exc:
            report.finish(exc)
            raise
        else:
            report.finish()
            quality = calibration.record["quality"]
            self.events.put(("status", f"Calibrated via {quality['accepted_views']}: IDs {quality['visible_marker_ids']}, max RMS {quality['reprojection_rms_px']:.3f} px. Click Start loop."))
        finally:
            if report.frame is not None:
                self.events.put(("image", (report.raw_image_rgb, report.annotated_rgb, len(report.frames) == 2)))

    def run_task(self, config, robot, single):
        if self.calibration is None:
            raise RuntimeError("Calibrate before starting")
        log = EventLog(new_log_path(config.resolve(config.data["log_directory"])))
        original_emit = log.emit

        def emit(event, **fields):
            original_emit(event, **fields)
            if event in ("visit_started", "hover_arrived", "visit_completed", "cycle_completed"):
                self.events.put(("status", f"{event.replace('_', ' ')} — cycle {fields.get('cycle', '')}, marker {fields.get('marker_id', '')}"))

        log.emit = emit
        try:
            MarkerHoverTask(config, self.calibration, robot, log,
                            cancel_requested=self.stop.is_set).run(1 if single else None)
            self.events.put(("status", "Cycle completed. Start again or recalibrate."))
        finally:
            log.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "configs/marker_hover.local.json")
    args = parser.parse_args()
    root = tk.Tk()
    Window(root, args.config.resolve())
    root.mainloop()


if __name__ == "__main__":
    main()
