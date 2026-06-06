#!/usr/bin/env python3
"""GUI for manual PID testing and safe PID autotuning.

The GUI only sends JSON commands to the ESP32. Motor control and hard safety
remain inside the ESP32 FreeRTOS control task.

Install dependencies with:
    pip install optuna websocket-client pandas
"""

from __future__ import annotations

import csv
import json
import queue
import statistics
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from typing import Any

import websocket
import optuna


CSV_FIELDS = [
    "timestamp",
    "elapsed_s",
    "kp",
    "ki",
    "kd",
    "setpoint",
    "maxPwm",
    "angle",
    "error",
    "pidOutput",
    "leftPwm",
    "rightPwm",
    "motorsEnabled",
    "safetyStop",
    "faultMessage",
]

AUTOTUNE_HISTORY_FIELDS = [
    "iteration",
    "step_type",
    "kp",
    "ki",
    "kd",
    "score",
    "mean_abs_error",
    "max_abs_error",
    "angle_stddev",
    "saturation_percent",
    "safetyStop",
    "status",
    "faultMessage",
    "csv_path",
]

BAYESIAN_HISTORY_FIELDS = [
    "timestamp",
    "trial",
    "Kp",
    "Ki",
    "Kd",
    "score",
    "mean_abs_error",
    "max_abs_error",
    "angle_stddev",
    "saturation_percent",
    "safetyStop",
    "faultMessage",
]

ARMING_ANGLE_ERROR_LIMIT_DEG = 5.0
ARMING_STABLE_DELAY_S = 2.0


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def safe_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def safe_int(value: Any, default: int = 0) -> int:
    try:
        return int(float(value))
    except (TypeError, ValueError):
        return default


def angle_for_arming(state: dict[str, Any]) -> float:
    return safe_float(
        state.get("selectedAngle", state.get("kalmanAngle", state.get("angle", state.get("filteredAngle", 999.0)))),
        999.0,
    )


@dataclass
class TestMetrics:
    samples: int
    mean_abs_error: float
    max_abs_error: float
    angle_stddev: float
    saturation_percent: float
    near_setpoint_percent: float
    safety_stop: bool
    score: float
    fault_message: str


@dataclass
class AutoTuneTrial:
    iteration: int
    step_type: str
    kp: float
    ki: float
    kd: float
    score: float
    metrics: TestMetrics
    status: str
    fault_message: str
    csv_path: Path | None
    timestamp: str = ""


@dataclass
class NMPoint:
    values: tuple[float, ...]
    step_type: str
    score: float | None = None


class NelderMeadOptimizer:
    def __init__(self, kp: float, ki: float, kd: float, tune_ki: bool, tolerance: float) -> None:
        self.tune_ki = tune_ki
        self.tolerance = tolerance
        self.alpha = 1.0
        self.gamma = 2.0
        self.rho = 0.5
        self.sigma = 0.5
        self.cache: dict[tuple[float, ...], float] = {}
        self.simplex: list[NMPoint] = []
        self.initial_queue = self._initial_simplex(kp, ki, kd)
        self.pending_reflection: NMPoint | None = None
        self.shrink_queue: list[NMPoint] = []
        self.last_best_score: float | None = None
        self.stop_reason = ""

    def _pack(self, kp: float, ki: float, kd: float) -> tuple[float, ...]:
        if self.tune_ki:
            return self._clamp_values((kp, ki, kd))
        return self._clamp_values((kp, kd))

    def unpack(self, values: tuple[float, ...]) -> tuple[float, float, float]:
        if self.tune_ki:
            kp, ki, kd = values
            return kp, ki, kd
        kp, kd = values
        return kp, 0.0, kd

    def key(self, values: tuple[float, ...]) -> tuple[float, ...]:
        return tuple(round(value, 4) for value in values)

    def _clamp_values(self, values: tuple[float, ...]) -> tuple[float, ...]:
        if self.tune_ki:
            kp, ki, kd = values
            return (clamp(kp, 0.0, 100.0), clamp(ki, 0.0, 5.0), clamp(kd, 0.0, 20.0))
        kp, kd = values
        return (clamp(kp, 0.0, 100.0), clamp(kd, 0.0, 20.0))

    def _initial_simplex(self, kp: float, ki: float, kd: float) -> list[NMPoint]:
        kp_step = max(abs(kp) * 0.2, 1.0)
        kd_step = max(abs(kd) * 0.2, 0.1)
        if self.tune_ki:
            ki_step = max(abs(ki) * 0.2, 0.01)
            points = [
                (kp, ki, kd),
                (kp + kp_step, ki, kd),
                (kp, ki + ki_step, kd),
                (kp, ki, kd + kd_step),
            ]
        else:
            points = [
                (kp, kd),
                (kp + kp_step, kd),
                (kp, kd + kd_step),
            ]
        return [NMPoint(self._clamp_values(tuple(point)), "initial") for point in points]

    def ask(self) -> NMPoint | None:
        while self.initial_queue:
            point = self.initial_queue.pop(0)
            if self.key(point.values) not in self.cache:
                return point
            point.score = self.cache[self.key(point.values)]
            self.simplex.append(point)

        if self.shrink_queue:
            point = self.shrink_queue.pop(0)
            if self.key(point.values) not in self.cache:
                return point
            point.score = self.cache[self.key(point.values)]
            self.simplex.append(point)
            if not self.shrink_queue:
                self._sort_simplex()

        if len(self.simplex) < self.dimension + 1:
            return None

        self._sort_simplex()
        if self.should_stop():
            return None
        return self._reflection_point()

    @property
    def dimension(self) -> int:
        return 3 if self.tune_ki else 2

    def tell(self, point: NMPoint, score: float) -> NMPoint | None:
        point.score = score
        self.cache[self.key(point.values)] = score

        if point.step_type in {"initial", "shrink"}:
            self.simplex.append(point)
            self._sort_simplex()
            return None

        if point.step_type == "reflection":
            best = self.simplex[0]
            second_worst = self.simplex[-2]
            worst = self.simplex[-1]
            assert best.score is not None and second_worst.score is not None and worst.score is not None
            if point.score < best.score:
                self.pending_reflection = point
                return self._expansion_point(point)
            if point.score < second_worst.score:
                self.simplex[-1] = point
                self._sort_simplex()
                return None
            self.pending_reflection = point
            return self._contraction_point(point if point.score < worst.score else worst)

        if point.step_type == "expansion":
            assert self.pending_reflection is not None and self.pending_reflection.score is not None
            if point.score < self.pending_reflection.score:
                self.simplex[-1] = point
            else:
                self.simplex[-1] = self.pending_reflection
            self.pending_reflection = None
            self._sort_simplex()
            return None

        if point.step_type == "contraction":
            worst = self.simplex[-1]
            assert worst.score is not None
            if point.score < worst.score:
                self.simplex[-1] = point
                self._sort_simplex()
            else:
                self._start_shrink()
            self.pending_reflection = None
            return None

        return None

    def _sort_simplex(self) -> None:
        self.simplex.sort(key=lambda point: float("inf") if point.score is None else point.score)

    def _centroid_without_worst(self) -> tuple[float, ...]:
        points = self.simplex[:-1]
        return tuple(sum(point.values[i] for point in points) / len(points) for i in range(self.dimension))

    def _reflection_point(self) -> NMPoint:
        centroid = self._centroid_without_worst()
        worst = self.simplex[-1].values
        values = tuple(centroid[i] + self.alpha * (centroid[i] - worst[i]) for i in range(self.dimension))
        return NMPoint(self._clamp_values(values), "reflection")

    def _expansion_point(self, reflection: NMPoint) -> NMPoint:
        centroid = self._centroid_without_worst()
        values = tuple(centroid[i] + self.gamma * (reflection.values[i] - centroid[i]) for i in range(self.dimension))
        return NMPoint(self._clamp_values(values), "expansion")

    def _contraction_point(self, source: NMPoint) -> NMPoint:
        centroid = self._centroid_without_worst()
        values = tuple(centroid[i] + self.rho * (source.values[i] - centroid[i]) for i in range(self.dimension))
        return NMPoint(self._clamp_values(values), "contraction")

    def _start_shrink(self) -> None:
        best = self.simplex[0]
        old_simplex = self.simplex[1:]
        self.simplex = [best]
        self.shrink_queue = []
        for point in old_simplex:
            values = tuple(best.values[i] + self.sigma * (point.values[i] - best.values[i]) for i in range(self.dimension))
            self.shrink_queue.append(NMPoint(self._clamp_values(values), "shrink"))

    def should_stop(self) -> bool:
        if len(self.simplex) < self.dimension + 1:
            return False
        scores = [point.score for point in self.simplex if point.score is not None]
        if len(scores) < self.dimension + 1:
            return False
        score_spread = max(scores) - min(scores)
        max_size = max(
            abs(self.simplex[i].values[j] - self.simplex[0].values[j])
            for i in range(1, len(self.simplex))
            for j in range(self.dimension)
        )
        if max_size < 0.02:
            self.stop_reason = "simplex is very small"
            return True
        if self.last_best_score is not None and abs(self.last_best_score - scores[0]) < self.tolerance and score_spread < self.tolerance:
            self.stop_reason = "score improvement below tolerance"
            return True
        self.last_best_score = scores[0]
        return False

    def best(self) -> NMPoint | None:
        self._sort_simplex()
        return self.simplex[0] if self.simplex else None


class WebSocketWorker(threading.Thread):
    def __init__(self, url: str, incoming: queue.Queue[tuple[str, Any]]) -> None:
        super().__init__(daemon=True)
        self.url = url
        self.incoming = incoming
        self.outgoing: queue.Queue[dict[str, Any] | None] = queue.Queue()
        self.stop_event = threading.Event()
        self.ws: websocket.WebSocket | None = None

    def send(self, command: dict[str, Any]) -> None:
        self.outgoing.put(command)

    def close(self) -> None:
        self.stop_event.set()
        self.outgoing.put(None)
        if self.ws is not None:
            try:
                self.ws.close()
            except Exception:
                pass

    def run(self) -> None:
        try:
            self.ws = websocket.create_connection(self.url, timeout=2.0)
            self.ws.settimeout(0.1)
            self.incoming.put(("status", f"Connected to {self.url}"))
            while not self.stop_event.is_set():
                self._flush_outgoing()
                self._receive_one()
        except Exception as exc:
            self.incoming.put(("error", str(exc)))
        finally:
            if self.ws is not None:
                try:
                    self.ws.close()
                except Exception:
                    pass
            self.incoming.put(("closed", "Disconnected"))

    def _flush_outgoing(self) -> None:
        if self.ws is None:
            return
        while True:
            try:
                command = self.outgoing.get_nowait()
            except queue.Empty:
                return
            if command is None:
                self.stop_event.set()
                return
            self.ws.send(json.dumps(command, separators=(",", ":")))

    def _receive_one(self) -> None:
        if self.ws is None:
            return
        try:
            message = self.ws.recv()
        except websocket.WebSocketTimeoutException:
            return
        data = json.loads(message)
        if data.get("type") in {"ack", "error"}:
            self.incoming.put(("message", data))
        else:
            self.incoming.put(("state", data))


class PidTunerGui:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("Robot Balancin PID Tuner")
        self.incoming: queue.Queue[tuple[str, Any]] = queue.Queue()
        self.worker: WebSocketWorker | None = None
        self.latest_state: dict[str, Any] = {}
        self.output_dir = Path("tools/pid_logs")
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.recording = False
        self.test_mode = "manual"
        self.test_started_monotonic = 0.0
        self.test_duration_s = 0.0
        self.csv_file = None
        self.csv_writer: csv.DictWriter | None = None
        self.rows: list[dict[str, Any]] = []
        self.current_csv_path: Path | None = None
        self.active_test_params: dict[str, Any] = {}
        self.active_nm_point: NMPoint | None = None
        self.test_instability = False
        self.test_armed = True
        self.arming_deadline = 0.0
        self.arming_angle_ready_since = 0.0
        self.arming_enable_requested = False
        self.last_enable_retry = 0.0

        self.autotune_active = False
        self.autotune_confirmed = False
        self.autotune_phase = "idle"
        self.autotune_deadline = 0.0
        self.autotune_iteration = 0
        self.autotune_waiting_confirmation = False
        self.autotune_optimizer: NelderMeadOptimizer | None = None
        self.autotune_history: list[AutoTuneTrial] = []
        self.autotune_best_trial: AutoTuneTrial | None = None
        self.autotune_stop_requested = False
        self.autotune_waiting_confirmation = False
        self.autotune_stop_reason = "-"
        self.pending_candidate: NMPoint | None = None

        self.bayesian_active = False
        self.bayesian_stop_requested = False
        self.bayesian_phase = "idle"
        self.bayesian_deadline = 0.0
        self.bayesian_iteration = 0
        self.bayesian_settings: dict[str, Any] = {}
        self.bayesian_study: optuna.Study | None = None
        self.bayesian_trial: optuna.trial.Trial | None = None
        self.bayesian_history: list[AutoTuneTrial] = []
        self.bayesian_best_trial: AutoTuneTrial | None = None

        self._build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(50, self.process_queue)
        self.root.after(100, self.autotune_tick)
        self.root.after(100, self.bayesian_tick)

    def _build_ui(self) -> None:
        main = ttk.Frame(self.root, padding=8)
        main.grid(row=0, column=0, sticky="nsew")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        self.host_var = tk.StringVar(value="192.168.1.11")
        self.port_var = tk.StringVar(value="81")
        self.status_var = tk.StringVar(value="Disconnected")
        self.output_dir_var = tk.StringVar(value=str(self.output_dir))
        self.kp_var = tk.StringVar(value="10.0")
        self.ki_var = tk.StringVar(value="0.0")
        self.kd_var = tk.StringVar(value="0.5")
        self.setpoint_var = tk.StringVar(value="-7.5")
        self.max_pwm_var = tk.StringVar(value="120")
        self.duration_var = tk.StringVar(value="5")
        self.near_var = tk.StringVar(value="2.0")
        self.safe_angle_var = tk.StringVar(value="35")
        self.max_iterations_var = tk.StringVar(value="10")
        self.tolerance_var = tk.StringVar(value="0.05")
        self.tune_ki_var = tk.BooleanVar(value=False)

        self.bayes_trials_var = tk.StringVar(value="20")
        self.bayes_trial_duration_var = tk.StringVar(value="2.0")
        self.bayes_setpoint_var = tk.StringVar(value="-3.5")
        self.bayes_max_pwm_var = tk.StringVar(value="155")
        self.bayes_min_pwm_var = tk.StringVar(value="15")
        self.bayes_safe_angle_var = tk.StringVar(value="25")
        self.bayes_tune_ki_var = tk.BooleanVar(value=False)
        self.bayes_kp_min_var = tk.StringVar(value="20")
        self.bayes_kp_max_var = tk.StringVar(value="40")
        current_ki = safe_float(self.ki_var.get(), 0.0)
        self.bayes_ki_min_var = tk.StringVar(value=f"{max(0.0, current_ki - 25.0):.3f}")
        self.bayes_ki_max_var = tk.StringVar(value=f"{current_ki + 25.0:.3f}")
        self.bayes_kd_min_var = tk.StringVar(value="1.5")
        self.bayes_kd_max_var = tk.StringVar(value="4.0")

        connection = ttk.LabelFrame(main, text="Connection", padding=8)
        connection.grid(row=0, column=0, sticky="ew", padx=4, pady=4)
        self._entry(connection, "ESP32 IP", self.host_var, 0, 0)
        self._entry(connection, "WS port", self.port_var, 0, 2, width=8)
        ttk.Button(connection, text="Connect", command=self.connect).grid(row=0, column=4, padx=4)
        ttk.Button(connection, text="Disconnect", command=self.disconnect).grid(row=0, column=5, padx=4)
        ttk.Label(connection, textvariable=self.status_var).grid(row=1, column=0, columnspan=6, sticky="w")

        params = ttk.LabelFrame(main, text="PID parameters", padding=8)
        params.grid(row=1, column=0, sticky="ew", padx=4, pady=4)
        self._entry(params, "Kp", self.kp_var, 0, 0)
        self._entry(params, "Ki", self.ki_var, 0, 2)
        self._entry(params, "Kd", self.kd_var, 0, 4)
        ttk.Button(params, text="Send Kp/Ki/Kd", command=self.send_pid).grid(row=0, column=6, padx=4)
        self._entry(params, "Natural setpoint", self.setpoint_var, 1, 0)
        ttk.Button(params, text="Send setpoint", command=self.send_setpoint).grid(row=1, column=2, padx=4)
        self._entry(params, "Max PWM", self.max_pwm_var, 1, 3)
        ttk.Button(params, text="Send PWM limit", command=self.send_pwm_limit).grid(row=1, column=5, padx=4)
        self._entry(params, "Safe angle", self.safe_angle_var, 2, 0)
        self._entry(params, "Max iterations", self.max_iterations_var, 2, 2)
        self._entry(params, "Tolerance", self.tolerance_var, 2, 4)
        ttk.Checkbutton(params, text="Tune Ki", variable=self.tune_ki_var).grid(row=2, column=6, padx=4)

        commands = ttk.LabelFrame(main, text="Commands", padding=8)
        commands.grid(row=2, column=0, sticky="ew", padx=4, pady=4)
        for index, (text, command) in enumerate([
            ("Enable motors", self.enable_motors),
            ("Disable motors", lambda: self.send_json({"type": "disable_motors"})),
            ("STOP", self.stop_robot),
            ("Reset encoders", lambda: self.send_json({"type": "reset_encoders"})),
            ("Calibrate gyro", lambda: self.send_json({"type": "calibrate_gyro"})),
            ("Calibrate vertical", lambda: self.send_json({"type": "calibrate_vertical"})),
            ("Test left motor", lambda: self.send_json({"type": "test_left_motor"})),
            ("Test right motor", lambda: self.send_json({"type": "test_right_motor"})),
        ]):
            ttk.Button(commands, text=text, command=command).grid(row=index // 4, column=index % 4, padx=4, pady=3, sticky="ew")

        state_frame = ttk.LabelFrame(main, text="Live state", padding=8)
        state_frame.grid(row=3, column=0, sticky="nsew", padx=4, pady=4)
        self.state_vars: dict[str, tk.StringVar] = {}
        labels = [
            ("angle", "Angle"), ("setpoint", "Setpoint"), ("pidError", "PID error"),
            ("pidOutput", "PID output"), ("leftPwm", "PWM left"), ("rightPwm", "PWM right"),
            ("leftEncoder", "Encoder left"), ("rightEncoder", "Encoder right"),
            ("leftSpeed", "Speed left"), ("rightSpeed", "Speed right"),
            ("motorsEnabled", "Motors"), ("safetyStop", "Safety stop"),
            ("faultMessage", "Fault"), ("imuReady", "IMU ready"), ("gyroCalibrated", "Gyro calibrated"),
        ]
        for index, (key, label) in enumerate(labels):
            var = tk.StringVar(value="-")
            self.state_vars[key] = var
            ttk.Label(state_frame, text=label).grid(row=index // 3, column=(index % 3) * 2, sticky="w", padx=4, pady=2)
            ttk.Label(state_frame, textvariable=var, width=18).grid(row=index // 3, column=(index % 3) * 2 + 1, sticky="w", padx=4, pady=2)

        logging = ttk.LabelFrame(main, text="Manual test logging", padding=8)
        logging.grid(row=4, column=0, sticky="ew", padx=4, pady=4)
        self._entry(logging, "Duration s", self.duration_var, 0, 0)
        self._entry(logging, "Near deg", self.near_var, 0, 2)
        ttk.Label(logging, text="Run Test enables motors automatically").grid(row=0, column=4, padx=4)
        ttk.Button(logging, text="Start test", command=self.start_test).grid(row=0, column=5, padx=4)
        ttk.Button(logging, text="Stop test", command=self.stop_test).grid(row=0, column=6, padx=4)
        ttk.Entry(logging, textvariable=self.output_dir_var, width=50).grid(row=1, column=0, columnspan=5, sticky="ew", padx=4, pady=4)
        ttk.Button(logging, text="Select CSV folder", command=self.select_output_dir).grid(row=1, column=5, columnspan=2, padx=4)
        self.metrics_var = tk.StringVar(value="Metrics: -")
        ttk.Label(logging, textvariable=self.metrics_var).grid(row=2, column=0, columnspan=7, sticky="w", padx=4, pady=4)

        self._build_autotune_ui(main)
        self._build_bayesian_autotune_ui(main)

    def _build_autotune_ui(self, main: ttk.Frame) -> None:
        autotune = ttk.LabelFrame(main, text="Nelder-Mead AutoTune", padding=8)
        autotune.grid(row=5, column=0, sticky="nsew", padx=4, pady=4)
        main.rowconfigure(5, weight=1)
        buttons = [
            ("Start Nelder-Mead AutoTune", self.start_autotune),
            ("Continue AutoTune", self.continue_autotune),
            ("Stop AutoTune", self.stop_autotune),
            ("Apply Best PID", self.apply_best_pid),
            ("Export AutoTune History CSV", self.export_autotune_history_csv),
        ]
        for index, (text, command) in enumerate(buttons):
            ttk.Button(autotune, text=text, command=command).grid(row=0, column=index, padx=4, pady=3, sticky="ew")

        self.autotune_status_var = tk.StringVar(value="AutoTune status: idle")
        self.autotune_eval_var = tk.StringVar(value="Evaluating PID: -")
        self.autotune_score_var = tk.StringVar(value="Current score: -")
        self.autotune_best_var = tk.StringVar(value="Best PID: -")
        self.autotune_stop_var = tk.StringVar(value="Stop reason: -")
        ttk.Label(autotune, textvariable=self.autotune_status_var).grid(row=1, column=0, columnspan=5, sticky="w", padx=4)
        ttk.Label(autotune, textvariable=self.autotune_eval_var).grid(row=2, column=0, columnspan=5, sticky="w", padx=4)
        ttk.Label(autotune, textvariable=self.autotune_score_var).grid(row=3, column=0, columnspan=5, sticky="w", padx=4)
        ttk.Label(autotune, textvariable=self.autotune_best_var).grid(row=4, column=0, columnspan=5, sticky="w", padx=4)
        ttk.Label(autotune, textvariable=self.autotune_stop_var).grid(row=5, column=0, columnspan=5, sticky="w", padx=4)

        columns = ["iter", "step", "kp", "ki", "kd", "score", "mean", "max", "std", "sat", "safety", "status", "fault"]
        self.autotune_tree = ttk.Treeview(autotune, columns=columns, show="headings", height=8)
        widths = [45, 90, 65, 65, 65, 85, 75, 75, 75, 65, 65, 85, 170]
        for column, width in zip(columns, widths):
            self.autotune_tree.heading(column, text=column)
            self.autotune_tree.column(column, width=width, anchor="center")
        self.autotune_tree.grid(row=6, column=0, columnspan=5, sticky="nsew", padx=4, pady=4)
        autotune.rowconfigure(6, weight=1)
        autotune.columnconfigure(4, weight=1)

    def _build_bayesian_autotune_ui(self, main: ttk.Frame) -> None:
        bayes = ttk.LabelFrame(main, text="Bayesian AutoTune", padding=8)
        bayes.grid(row=6, column=0, sticky="nsew", padx=4, pady=4)
        main.rowconfigure(6, weight=1)

        buttons = [
            ("Start Bayesian AutoTune", self.start_bayesian_autotune),
            ("Stop AutoTune", self.stop_bayesian_autotune),
            ("Apply Best PID", self.apply_bayesian_best_pid),
            ("Export Results CSV", self.export_bayesian_results_csv),
        ]
        for index, (text, command) in enumerate(buttons):
            ttk.Button(bayes, text=text, command=command).grid(row=0, column=index, padx=4, pady=3, sticky="ew")

        self._entry(bayes, "Trials", self.bayes_trials_var, 1, 0)
        self._entry(bayes, "Trial duration s", self.bayes_trial_duration_var, 1, 2)
        self._entry(bayes, "Setpoint", self.bayes_setpoint_var, 1, 4)
        self._entry(bayes, "Max PWM", self.bayes_max_pwm_var, 2, 0)
        self._entry(bayes, "Min PWM", self.bayes_min_pwm_var, 2, 2)
        self._entry(bayes, "Safe angle limit", self.bayes_safe_angle_var, 2, 4)
        ttk.Checkbutton(bayes, text="Tune Ki", variable=self.bayes_tune_ki_var).grid(row=2, column=6, padx=4, sticky="w")

        self._entry(bayes, "Kp min", self.bayes_kp_min_var, 3, 0)
        self._entry(bayes, "Kp max", self.bayes_kp_max_var, 3, 2)
        self._entry(bayes, "Ki min", self.bayes_ki_min_var, 3, 4)
        self._entry(bayes, "Ki max", self.bayes_ki_max_var, 3, 6)
        self._entry(bayes, "Kd min", self.bayes_kd_min_var, 4, 0)
        self._entry(bayes, "Kd max", self.bayes_kd_max_var, 4, 2)

        self.bayes_status_var = tk.StringVar(value="Bayesian status: idle")
        self.bayes_trial_var = tk.StringVar(value="Trial: -")
        self.bayes_pid_var = tk.StringVar(value="PID in test: -")
        self.bayes_score_var = tk.StringVar(value="Current score: -")
        self.bayes_best_var = tk.StringVar(value="Best PID: -")
        self.bayes_safety_var = tk.StringVar(value="Safety: -")
        self.bayes_fault_var = tk.StringVar(value="Fault: -")
        ttk.Label(bayes, textvariable=self.bayes_status_var).grid(row=5, column=0, columnspan=4, sticky="w", padx=4)
        ttk.Label(bayes, textvariable=self.bayes_trial_var).grid(row=5, column=4, columnspan=4, sticky="w", padx=4)
        ttk.Label(bayes, textvariable=self.bayes_pid_var).grid(row=6, column=0, columnspan=4, sticky="w", padx=4)
        ttk.Label(bayes, textvariable=self.bayes_score_var).grid(row=6, column=4, columnspan=4, sticky="w", padx=4)
        ttk.Label(bayes, textvariable=self.bayes_best_var).grid(row=7, column=0, columnspan=4, sticky="w", padx=4)
        ttk.Label(bayes, textvariable=self.bayes_safety_var).grid(row=7, column=4, columnspan=2, sticky="w", padx=4)
        ttk.Label(bayes, textvariable=self.bayes_fault_var).grid(row=7, column=6, columnspan=2, sticky="w", padx=4)

        columns = ["trial", "Kp", "Ki", "Kd", "score", "mean", "max", "std", "sat", "safety", "fault"]
        self.bayes_tree = ttk.Treeview(bayes, columns=columns, show="headings", height=8)
        widths = [55, 70, 70, 70, 85, 75, 75, 75, 65, 70, 220]
        for column, width in zip(columns, widths):
            self.bayes_tree.heading(column, text=column)
            self.bayes_tree.column(column, width=width, anchor="center")
        self.bayes_tree.grid(row=8, column=0, columnspan=8, sticky="nsew", padx=4, pady=4)
        bayes.rowconfigure(8, weight=1)
        bayes.columnconfigure(7, weight=1)

    def _entry(self, parent: ttk.Frame, label: str, var: tk.StringVar, row: int, column: int, width: int = 10) -> None:
        ttk.Label(parent, text=label).grid(row=row, column=column, sticky="w", padx=4, pady=2)
        ttk.Entry(parent, textvariable=var, width=width).grid(row=row, column=column + 1, sticky="w", padx=4, pady=2)

    def connect(self) -> None:
        if self.worker is not None:
            return
        self.worker = WebSocketWorker(f"ws://{self.host_var.get().strip()}:{safe_int(self.port_var.get(), 81)}/", self.incoming)
        self.worker.start()
        self.status_var.set("Connecting...")

    def disconnect(self) -> None:
        if self.worker is not None:
            self.worker.close()
            self.worker = None
        self.status_var.set("Disconnected")

    def send_json(self, command: dict[str, Any], warn_if_disconnected: bool = True) -> bool:
        if self.worker is None:
            if warn_if_disconnected:
                messagebox.showwarning("Not connected", "Connect to the ESP32 first.")
            return False
        self.worker.send(command)
        return True

    def send_pid(self) -> None:
        try:
            kp, ki, kd = self._read_pid_values()
        except ValueError as exc:
            messagebox.showerror("Invalid parameter", str(exc))
            return
        self.send_json({"type": "set_pid", "kp": kp, "ki": ki, "kd": kd})

    def send_setpoint(self) -> None:
        try:
            setpoint = self._validated_float(self.setpoint_var, -15, 15, "Setpoint")
        except ValueError as exc:
            messagebox.showerror("Invalid parameter", str(exc))
            return
        self.send_json({"type": "set_setpoint", "setpoint": setpoint})

    def send_pwm_limit(self) -> None:
        try:
            max_pwm = self._validated_int(self.max_pwm_var, 0, 180, "Max PWM")
        except ValueError as exc:
            messagebox.showerror("Invalid parameter", str(exc))
            return
        self.send_json({"type": "set_pwm_limit", "maxPwm": max_pwm})

    def enable_motors(self) -> None:
        if messagebox.askyesno("Enable motors", "Enable motors now? Ensure the robot is safely supported."):
            self.send_json({"type": "enable_motors"})

    def stop_robot(self) -> None:
        self.send_json({"type": "stop"}, warn_if_disconnected=False)
        self.send_json({"type": "disable_motors"}, warn_if_disconnected=False)

    def _validated_float(self, var: tk.StringVar, minimum: float, maximum: float, name: str) -> float:
        value = safe_float(var.get(), float("nan"))
        if not minimum <= value <= maximum:
            raise ValueError(f"{name} must be between {minimum} and {maximum}")
        return value

    def _validated_int(self, var: tk.StringVar, minimum: int, maximum: int, name: str) -> int:
        value = safe_int(var.get(), minimum - 1)
        if not minimum <= value <= maximum:
            raise ValueError(f"{name} must be between {minimum} and {maximum}")
        return value

    def _read_pid_values(self) -> tuple[float, float, float]:
        return (
            clamp(self._validated_float(self.kp_var, 0, 100, "Kp"), 0, 100),
            clamp(self._validated_float(self.ki_var, 0, 5, "Ki"), 0, 5),
            clamp(self._validated_float(self.kd_var, 0, 20, "Kd"), 0, 20),
        )

    def _read_test_settings(self) -> dict[str, Any]:
        kp, ki, kd = self._read_pid_values()
        return {
            "kp": kp,
            "ki": ki,
            "kd": kd,
            "setpoint": self._validated_float(self.setpoint_var, -15, 15, "Setpoint"),
            "max_pwm": self._validated_int(self.max_pwm_var, 0, 180, "Max PWM"),
            "duration_s": self._validated_float(self.duration_var, 0.1, 120.0, "Duration"),
            "near_deg": self._validated_float(self.near_var, 0.1, 30.0, "Near threshold"),
            "safe_angle": self._validated_float(self.safe_angle_var, 1.0, 90.0, "Safe angle"),
            "max_iterations": self._validated_int(self.max_iterations_var, 1, 100, "Max iterations"),
            "tolerance": self._validated_float(self.tolerance_var, 0.0, 1000.0, "Tolerance"),
        }

    def _read_bayesian_settings(self) -> dict[str, Any]:
        kp_min = self._validated_float(self.bayes_kp_min_var, 0.0, 100.0, "Bayesian Kp min")
        kp_max = self._validated_float(self.bayes_kp_max_var, 0.0, 100.0, "Bayesian Kp max")
        ki_min = self._validated_float(self.bayes_ki_min_var, 0.0, 500.0, "Bayesian Ki min")
        ki_max = self._validated_float(self.bayes_ki_max_var, 0.0, 500.0, "Bayesian Ki max")
        kd_min = self._validated_float(self.bayes_kd_min_var, 0.0, 20.0, "Bayesian Kd min")
        kd_max = self._validated_float(self.bayes_kd_max_var, 0.0, 20.0, "Bayesian Kd max")
        if kp_min >= kp_max:
            raise ValueError("Bayesian Kp min must be lower than Kp max")
        if ki_min >= ki_max:
            raise ValueError("Bayesian Ki min must be lower than Ki max")
        if kd_min >= kd_max:
            raise ValueError("Bayesian Kd min must be lower than Kd max")
        current_ki = safe_float(self.latest_state.get("ki", self.ki_var.get()), 0.0)
        return {
            "trials": self._validated_int(self.bayes_trials_var, 1, 200, "Bayesian trials"),
            "duration_s": self._validated_float(self.bayes_trial_duration_var, 0.5, 30.0, "Bayesian trial duration"),
            "setpoint": self._validated_float(self.bayes_setpoint_var, -15.0, 15.0, "Bayesian setpoint"),
            "max_pwm": self._validated_int(self.bayes_max_pwm_var, 0, 255, "Bayesian max PWM"),
            "min_pwm": self._validated_int(self.bayes_min_pwm_var, 0, 80, "Bayesian min PWM"),
            "safe_angle": self._validated_float(self.bayes_safe_angle_var, 1.0, 90.0, "Bayesian safe angle"),
            "kp_min": kp_min,
            "kp_max": kp_max,
            "ki_min": ki_min,
            "ki_max": ki_max,
            "kd_min": kd_min,
            "kd_max": kd_max,
            "tune_ki": self.bayes_tune_ki_var.get(),
            "fixed_ki": current_ki,
            "near_deg": safe_float(self.near_var.get(), 2.0),
        }

    def process_queue(self) -> None:
        while True:
            try:
                kind, payload = self.incoming.get_nowait()
            except queue.Empty:
                break
            if kind == "state":
                self.update_state(payload)
            elif kind == "status":
                self.status_var.set(str(payload))
            elif kind == "message":
                self.status_var.set(str(payload))
            elif kind == "error":
                self.status_var.set(f"Error: {payload}")
            elif kind == "closed":
                self.status_var.set("Disconnected")
                self.worker = None
        self.root.after(50, self.process_queue)

    def update_state(self, state: dict[str, Any]) -> None:
        self.latest_state = state
        for key, var in self.state_vars.items():
            value = state.get(key, "-")
            var.set(f"{value:.3f}" if isinstance(value, float) else str(value) if value is not None else "-")

        if self.recording:
            if self.test_mode in {"autotune", "bayesian"} and not self.test_armed:
                self._handle_autotune_arming_state(state)
                return

            self.write_csv_row(state)
            setpoint = safe_float(self.active_test_params.get("setpoint", self.setpoint_var.get()), -7.5)
            safe_angle = safe_float(self.active_test_params.get("safe_angle", self.safe_angle_var.get()), 35.0)
            angle = angle_for_arming(state)
            if abs(angle - setpoint) > safe_angle:
                self.test_instability = True
                self.finish_test("Stopped: angle error exceeded safe limit")
            elif state.get("safetyStop", False):
                self.finish_test("Stopped: safetyStop from robot")
            elif time.monotonic() - self.test_started_monotonic >= self.test_duration_s:
                self.finish_test("Completed")

        if hasattr(self, "bayes_safety_var"):
            self.bayes_safety_var.set(f"Safety: {bool(state.get('safetyStop', False))}")
            self.bayes_fault_var.set(f"Fault: {state.get('faultMessage', '-')}")

    def _handle_autotune_arming_state(self, state: dict[str, Any]) -> None:
        setpoint = safe_float(self.active_test_params.get("setpoint", self.setpoint_var.get()), -7.5)
        safe_angle = safe_float(self.active_test_params.get("safe_angle", self.safe_angle_var.get()), 35.0)
        angle = angle_for_arming(state)
        angle_error = abs(angle - setpoint)
        fault = str(state.get("faultMessage", ""))
        now = time.monotonic()

        if angle_error >= ARMING_ANGLE_ERROR_LIMIT_DEG:
            self.arming_angle_ready_since = 0.0
            self.arming_enable_requested = False
            if bool(state.get("motorsEnabled", False)):
                self.stop_robot()
            self.status_var.set(
                f"AutoTune arming: waiting angle near setpoint "
                f"({angle_error:.2f} deg >= {ARMING_ANGLE_ERROR_LIMIT_DEG:.1f} deg)"
            )
            if self.test_mode == "bayesian":
                self.bayes_status_var.set("Bayesian status: waiting angle near setpoint")
            return

        if (
            self.arming_enable_requested
            and bool(state.get("motorsEnabled", False))
            and not bool(state.get("safetyStop", False))
        ):
            self.test_armed = True
            self.rows = []
            self.test_started_monotonic = now
            self.metrics_var.set(f"AutoTune test armed: {self.current_csv_path}")
            return

        if bool(state.get("safetyStop", False)) and fault not in {"Stop requested", "Motors disabled", "Boot", "OK"}:
            self.write_csv_row(state)
            self.finish_test(f"Stopped while arming: {fault}")
            return

        if self.arming_angle_ready_since <= 0.0:
            self.arming_angle_ready_since = now
            self.arming_enable_requested = False
            self.status_var.set("AutoTune arming: angle in range, waiting 2.0 s")
            if self.test_mode == "bayesian":
                self.bayes_status_var.set("Bayesian status: angle in range, waiting 2.0 s")
            return

        stable_elapsed = now - self.arming_angle_ready_since
        if stable_elapsed < ARMING_STABLE_DELAY_S:
            remaining = ARMING_STABLE_DELAY_S - stable_elapsed
            self.status_var.set(f"AutoTune arming: angle stable, enabling in {remaining:.1f} s")
            if self.test_mode == "bayesian":
                self.bayes_status_var.set(f"Bayesian status: angle stable, enabling in {remaining:.1f} s")
            return

        if now - self.last_enable_retry >= 0.25:
            self.last_enable_retry = now
            self.arming_enable_requested = True
            self.status_var.set("AutoTune arming: angle OK, waiting for motorsEnabled=True")
            self.send_json({"type": "enable_motors"}, warn_if_disconnected=False)

    def select_output_dir(self) -> None:
        selected = filedialog.askdirectory(initialdir=str(self.output_dir))
        if selected:
            self.output_dir = Path(selected)
            self.output_dir_var.set(str(self.output_dir))

    def start_test(self) -> None:
        if self.recording or self.autotune_active or self.bayesian_active:
            messagebox.showwarning("Busy", "Stop the current test or AutoTune first.")
            return
        try:
            settings = self._read_test_settings()
        except ValueError as exc:
            messagebox.showerror("Invalid parameter", str(exc))
            return
        self._start_recording_test("manual", settings, NMPoint((settings["kp"], settings["kd"]), "manual"), enable_motors=True)

    def stop_test(self) -> None:
        if self.recording:
            self.finish_test("Stopped by user")

    def _start_recording_test(self, mode: str, settings: dict[str, Any], point: NMPoint, enable_motors: bool) -> None:
        self.output_dir = Path(self.output_dir_var.get())
        self.output_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = self.output_dir / f"pid_{mode}_test_{timestamp}.csv"
        self.csv_file = csv_path.open("w", newline="", encoding="utf-8")
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=CSV_FIELDS)
        self.csv_writer.writeheader()
        self.rows = []
        self.active_test_params = settings
        self.active_nm_point = point
        self.current_csv_path = csv_path
        self.test_mode = mode
        self.test_duration_s = settings["duration_s"]
        self.test_started_monotonic = time.monotonic()
        self.test_instability = False
        self.test_armed = mode not in {"autotune", "bayesian"}
        self.arming_deadline = 0.0
        self.arming_angle_ready_since = 0.0
        self.arming_enable_requested = False
        self.last_enable_retry = 0.0
        self.recording = True
        self.metrics_var.set(f"Arming AutoTune test: {csv_path}" if mode == "autotune" else f"Recording: {csv_path}")
        self.send_json({"type": "set_pid", "kp": settings["kp"], "ki": settings["ki"], "kd": settings["kd"]})
        self.send_json({"type": "set_setpoint", "setpoint": settings["setpoint"]})
        self.send_json({"type": "set_pwm_limit", "maxPwm": settings["max_pwm"]})
        if "min_pwm" in settings:
            self.send_json({"type": "set_min_pwm", "minPwm": settings["min_pwm"]})
        if mode == "bayesian":
            self.send_json({"type": "set_angle_filter", "filter": "kalman"})
        if enable_motors and mode not in {"autotune", "bayesian"}:
            self.send_json({"type": "enable_motors"})

    def write_csv_row(self, state: dict[str, Any]) -> None:
        if self.csv_writer is None:
            return
        elapsed = time.monotonic() - self.test_started_monotonic
        params = self.active_test_params
        angle = safe_float(state.get("angle", state.get("filteredAngle", 0.0)))
        setpoint = safe_float(state.get("setpoint", params.get("setpoint", self.setpoint_var.get())))
        row = {
            "timestamp": time.time(),
            "elapsed_s": elapsed,
            "kp": safe_float(state.get("kp", params.get("kp", self.kp_var.get()))),
            "ki": safe_float(state.get("ki", params.get("ki", self.ki_var.get()))),
            "kd": safe_float(state.get("kd", params.get("kd", self.kd_var.get()))),
            "setpoint": setpoint,
            "maxPwm": safe_int(state.get("maxPwm", params.get("max_pwm", self.max_pwm_var.get()))),
            "angle": angle,
            "error": safe_float(state.get("pidError", setpoint - angle)),
            "pidOutput": safe_float(state.get("pidOutput", 0.0)),
            "leftPwm": safe_int(state.get("leftPwm", 0)),
            "rightPwm": safe_int(state.get("rightPwm", 0)),
            "motorsEnabled": bool(state.get("motorsEnabled", False)),
            "safetyStop": bool(state.get("safetyStop", False)),
            "faultMessage": str(state.get("faultMessage", "")),
        }
        self.rows.append(row)
        self.csv_writer.writerow(row)

    def finish_test(self, reason: str) -> None:
        if not self.recording:
            return
        mode = self.test_mode
        point = self.active_nm_point
        csv_path = self.current_csv_path
        self.recording = False
        self.stop_robot()
        if self.csv_file is not None:
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None
        metrics = self.compute_metrics()
        self.metrics_var.set(self.format_metrics(reason, metrics))
        if mode == "autotune" and point is not None:
            self._complete_autotune_trial(point, metrics, csv_path, reason)
        elif mode == "bayesian":
            self._complete_bayesian_trial(metrics, reason)

    def compute_metrics(self) -> TestMetrics:
        if not self.rows:
            return TestMetrics(0, 999.0, 999.0, 999.0, 100.0, 0.0, True, 12000.0, "No samples")
        errors = [abs(safe_float(row["error"])) for row in self.rows]
        angles = [safe_float(row["angle"]) for row in self.rows]
        outputs = [abs(safe_float(row["pidOutput"])) for row in self.rows]
        max_pwm = max(1, safe_int(self.active_test_params.get("max_pwm", self.max_pwm_var.get()), 1))
        near = safe_float(self.active_test_params.get("near_deg", self.near_var.get()), 2.0)
        safe_angle = safe_float(self.active_test_params.get("safe_angle", self.safe_angle_var.get()), 35.0)
        safety_stop = any(bool(row["safetyStop"]) for row in self.rows)
        saturated = sum(1 for output in outputs if output >= 0.95 * max_pwm)
        near_count = sum(1 for error in errors if error <= near)
        mean_abs_error = statistics.fmean(errors)
        max_abs_error = max(errors)
        angle_stddev = statistics.pstdev(angles) if len(angles) > 1 else 0.0
        saturation_percent = 100.0 * saturated / len(self.rows)
        near_setpoint_percent = 100.0 * near_count / len(self.rows)
        safety_penalty = 10000.0 if safety_stop else 0.0
        instability_penalty = 1000.0 if self.test_instability or max_abs_error > safe_angle else 0.0
        score = mean_abs_error + 0.6 * angle_stddev + 0.05 * saturation_percent + 0.2 * max_abs_error + safety_penalty + instability_penalty
        fault_message = str(self.rows[-1].get("faultMessage", ""))
        return TestMetrics(len(self.rows), mean_abs_error, max_abs_error, angle_stddev, saturation_percent, near_setpoint_percent, safety_stop, score, fault_message)

    def format_metrics(self, reason: str, metrics: TestMetrics) -> str:
        return (
            f"{reason}. samples={metrics.samples} mean={metrics.mean_abs_error:.3f} "
            f"max={metrics.max_abs_error:.3f} std={metrics.angle_stddev:.3f} "
            f"sat={metrics.saturation_percent:.1f}% safety={metrics.safety_stop} score={metrics.score:.3f}"
        )

    def start_autotune(self) -> None:
        if self.worker is None:
            messagebox.showwarning("Not connected", "Connect before starting AutoTune.")
            return
        if self.recording or self.bayesian_active:
            messagebox.showwarning("Busy", "Stop the current test first.")
            return
        try:
            settings = self._read_test_settings()
        except ValueError as exc:
            messagebox.showerror("Invalid parameter", str(exc))
            return
        if not messagebox.askyesno("Start AutoTune", "Start Nelder-Mead AutoTune? Each iteration will ask before enabling motors."):
            return

        self.autotune_active = True
        self.autotune_confirmed = True
        self.autotune_stop_requested = False
        self.autotune_stop_reason = "-"
        self.autotune_iteration = 0
        self.autotune_history = []
        self.autotune_best_trial = None
        self.autotune_optimizer = NelderMeadOptimizer(settings["kp"], settings["ki"], settings["kd"], self.tune_ki_var.get(), settings["tolerance"])
        self.pending_candidate = None
        for item in self.autotune_tree.get_children():
            self.autotune_tree.delete(item)
        self.autotune_phase = "pre_stop"
        self.autotune_deadline = time.monotonic() + 1.0
        self.stop_robot()
        self.autotune_status_var.set("AutoTune status: pre-stop before first test")

    def stop_autotune(self) -> None:
        self.autotune_stop_requested = True
        self.autotune_stop_reason = "stopped by user"
        if self.recording:
            self.finish_test("AutoTune stopped by user")
        self.stop_robot()
        self.autotune_active = False
        self.autotune_phase = "idle"
        self.autotune_status_var.set("AutoTune status: stopped")
        self.autotune_stop_var.set("Stop reason: stopped by user")

    def autotune_tick(self) -> None:
        try:
            self._autotune_tick_impl()
        finally:
            self.root.after(100, self.autotune_tick)

    def _autotune_tick_impl(self) -> None:
        if not self.autotune_active or self.recording:
            return
        if self.autotune_waiting_confirmation:
            return
        if self.autotune_phase == "waiting_start":
            return
        if self.autotune_stop_requested:
            self.stop_robot()
            self.autotune_active = False
            return
        if time.monotonic() < self.autotune_deadline:
            return
        if self.autotune_iteration >= safe_int(self.max_iterations_var.get(), 10):
            self._finish_autotune("maximum iterations reached")
            return
        if self.autotune_optimizer is None:
            self._finish_autotune("optimizer not initialized")
            return

        point = self.pending_candidate or self.autotune_optimizer.ask()
        self.pending_candidate = None
        if point is None:
            reason = self.autotune_optimizer.stop_reason or "optimizer converged"
            self._finish_autotune(reason)
            return

        key = self.autotune_optimizer.key(point.values)
        if key in self.autotune_optimizer.cache:
            cached_score = self.autotune_optimizer.cache[key]
            extra = self.autotune_optimizer.tell(point, cached_score)
            self.pending_candidate = extra
            return

        kp, ki, kd = self.autotune_optimizer.unpack(point.values)
        try:
            settings = self._read_test_settings()
        except ValueError as exc:
            self._finish_autotune(str(exc))
            return
        settings.update({"kp": kp, "ki": ki, "kd": kd})
        next_iteration = self.autotune_iteration + 1
        if not self._confirm_autotune_iteration(next_iteration, point.step_type, settings):
            self.pending_candidate = point
            self.autotune_waiting_confirmation = True
            self.autotune_phase = "waiting_confirmation"
            self.autotune_status_var.set("AutoTune status: waiting for user confirmation")
            self.autotune_eval_var.set(f"Pending PID: Kp={kp:.3f} Ki={ki:.3f} Kd={kd:.3f}")
            return
        self.autotune_waiting_confirmation = False
        self.autotune_iteration += 1
        self.autotune_status_var.set(f"AutoTune status: running iteration {self.autotune_iteration} ({point.step_type})")
        self.autotune_eval_var.set(f"Evaluating PID: Kp={kp:.3f} Ki={ki:.3f} Kd={kd:.3f}")
        self.stop_robot()
        self.autotune_phase = "waiting_start"
        self.root.after(1000, lambda: self._start_autotune_test(settings, point))

    def _confirm_autotune_iteration(self, iteration: int, step_type: str, settings: dict[str, Any]) -> bool:
        return messagebox.askyesno(
            "Confirm AutoTune iteration",
            "Start this AutoTune test with motors enabled?\n\n"
            f"Iteration: {iteration}\n"
            f"Step: {step_type}\n"
            f"Kp: {settings['kp']:.3f}\n"
            f"Ki: {settings['ki']:.3f}\n"
            f"Kd: {settings['kd']:.3f}\n"
            f"Setpoint: {settings['setpoint']:.3f}\n"
            f"Max PWM: {settings['max_pwm']}\n"
            f"Duration: {settings['duration_s']:.2f} s",
        )

    def continue_autotune(self) -> None:
        if not self.autotune_active or not self.autotune_waiting_confirmation:
            return
        self.autotune_waiting_confirmation = False
        self.autotune_phase = "wait_after_test"
        self.autotune_deadline = time.monotonic()
        self.autotune_status_var.set("AutoTune status: continuing")

    def _start_autotune_test(self, settings: dict[str, Any], point: NMPoint) -> None:
        if not self.autotune_active or self.autotune_stop_requested:
            return
        self.autotune_phase = "running_test"
        self._start_recording_test("autotune", settings, point, enable_motors=self.autotune_confirmed)

    def _complete_autotune_trial(self, point: NMPoint, metrics: TestMetrics, csv_path: Path | None, reason: str) -> None:
        if self.autotune_optimizer is None:
            return
        point.score = metrics.score
        kp, ki, kd = self.autotune_optimizer.unpack(point.values)
        status = "rejected"
        if self.autotune_best_trial is None or metrics.score < self.autotune_best_trial.score:
            status = "accepted"
        trial = AutoTuneTrial(self.autotune_iteration, point.step_type, kp, ki, kd, metrics.score, metrics, status, metrics.fault_message or reason, csv_path)
        self.autotune_history.append(trial)
        if self.autotune_best_trial is None or trial.score < self.autotune_best_trial.score:
            self.autotune_best_trial = trial
        self._insert_autotune_row(trial)
        self._refresh_autotune_labels(trial)
        next_point = self.autotune_optimizer.tell(point, metrics.score)
        self.pending_candidate = next_point
        self.stop_robot()
        self.autotune_phase = "wait_after_test"
        self.autotune_deadline = time.monotonic() + 1.0

    def _insert_autotune_row(self, trial: AutoTuneTrial) -> None:
        metrics = trial.metrics
        self.autotune_tree.insert("", "end", values=(
            trial.iteration,
            trial.step_type,
            f"{trial.kp:.3f}",
            f"{trial.ki:.3f}",
            f"{trial.kd:.3f}",
            f"{trial.score:.3f}",
            f"{metrics.mean_abs_error:.3f}",
            f"{metrics.max_abs_error:.3f}",
            f"{metrics.angle_stddev:.3f}",
            f"{metrics.saturation_percent:.1f}",
            str(metrics.safety_stop),
            trial.status,
            trial.fault_message,
        ))

    def _refresh_autotune_labels(self, trial: AutoTuneTrial) -> None:
        self.autotune_score_var.set(f"Current score: {trial.score:.3f}")
        if self.autotune_best_trial is not None:
            best = self.autotune_best_trial
            self.autotune_best_var.set(f"Best PID: score={best.score:.3f} Kp={best.kp:.3f} Ki={best.ki:.3f} Kd={best.kd:.3f}")

    def _finish_autotune(self, reason: str) -> None:
        self.stop_robot()
        self.autotune_active = False
        self.autotune_phase = "idle"
        self.autotune_stop_reason = reason
        self.autotune_status_var.set("AutoTune status: finished")
        self.autotune_stop_var.set(f"Stop reason: {reason}")

    def apply_best_pid(self) -> None:
        if self.autotune_best_trial is None:
            messagebox.showwarning("No best PID", "Run AutoTune first.")
            return
        best = self.autotune_best_trial
        self.kp_var.set(f"{best.kp:.3f}")
        self.ki_var.set(f"{best.ki:.3f}")
        self.kd_var.set(f"{best.kd:.3f}")
        self.send_json({"type": "set_pid", "kp": best.kp, "ki": best.ki, "kd": best.kd})
        self.send_setpoint()
        self.send_pwm_limit()

    def export_autotune_history_csv(self) -> None:
        if not self.autotune_history:
            messagebox.showwarning("No history", "There is no AutoTune history to export.")
            return
        path = filedialog.asksaveasfilename(
            initialdir=str(self.output_dir),
            initialfile=f"pid_autotune_history_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
        )
        if not path:
            return
        with Path(path).open("w", newline="", encoding="utf-8") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=AUTOTUNE_HISTORY_FIELDS)
            writer.writeheader()
            for trial in self.autotune_history:
                metrics = trial.metrics
                writer.writerow({
                    "iteration": trial.iteration,
                    "step_type": trial.step_type,
                    "kp": trial.kp,
                    "ki": trial.ki,
                    "kd": trial.kd,
                    "score": trial.score,
                    "mean_abs_error": metrics.mean_abs_error,
                    "max_abs_error": metrics.max_abs_error,
                    "angle_stddev": metrics.angle_stddev,
                    "saturation_percent": metrics.saturation_percent,
                    "safetyStop": metrics.safety_stop,
                    "status": trial.status,
                    "faultMessage": trial.fault_message,
                    "csv_path": str(trial.csv_path or ""),
                })
        self.autotune_status_var.set(f"AutoTune status: exported {path}")

    def start_bayesian_autotune(self) -> None:
        if self.worker is None:
            messagebox.showwarning("Not connected", "Connect before starting Bayesian AutoTune.")
            return
        if self.recording or self.autotune_active or self.bayesian_active:
            messagebox.showwarning("Busy", "Stop the current test or AutoTune first.")
            return
        try:
            settings = self._read_bayesian_settings()
        except ValueError as exc:
            messagebox.showerror("Invalid parameter", str(exc))
            return
        if not messagebox.askyesno(
            "Start Bayesian AutoTune",
            "Start Bayesian AutoTune?\n\n"
            "The robot will enable motors automatically for each trial.\n"
            "Keep the robot supported and be ready to stop it.\n"
            "The GUI will stop and disable motors after every trial.",
        ):
            return

        self.bayesian_active = True
        self.bayesian_stop_requested = False
        self.bayesian_phase = "pre_stop"
        self.bayesian_deadline = time.monotonic() + 1.0
        self.bayesian_iteration = 0
        self.bayesian_settings = settings
        self.bayesian_trial = None
        self.bayesian_history = []
        self.bayesian_best_trial = None
        self.bayesian_study = optuna.create_study(direction="minimize", sampler=optuna.samplers.TPESampler())
        for item in self.bayes_tree.get_children():
            self.bayes_tree.delete(item)
        self.stop_robot()
        self.send_json({"type": "set_angle_filter", "filter": "kalman"}, warn_if_disconnected=False)
        self.bayes_status_var.set("Bayesian status: pre-stop before first trial")
        self.bayes_trial_var.set(f"Trial: 0 / {settings['trials']}")
        self.bayes_pid_var.set("PID in test: -")
        self.bayes_score_var.set("Current score: -")
        self.bayes_best_var.set("Best PID: -")

    def stop_bayesian_autotune(self) -> None:
        self.bayesian_stop_requested = True
        if self.recording and self.test_mode == "bayesian":
            self.finish_test("Bayesian AutoTune stopped by user")
        self.stop_robot()
        self.bayesian_active = False
        self.bayesian_phase = "idle"
        self.bayes_status_var.set("Bayesian status: stopped")

    def bayesian_tick(self) -> None:
        try:
            self._bayesian_tick_impl()
        finally:
            self.root.after(100, self.bayesian_tick)

    def _bayesian_tick_impl(self) -> None:
        if not self.bayesian_active or self.recording:
            return
        if self.bayesian_phase == "waiting_start":
            return
        if self.bayesian_stop_requested:
            self.stop_robot()
            self.bayesian_active = False
            self.bayesian_phase = "idle"
            return
        if time.monotonic() < self.bayesian_deadline:
            return
        if self.bayesian_study is None:
            self._finish_bayesian_autotune("Optuna study not initialized")
            return
        if self.bayesian_iteration >= safe_int(self.bayesian_settings.get("trials", 0), 0):
            self._finish_bayesian_autotune("maximum trials reached")
            return

        optuna_trial = self.bayesian_study.ask()
        settings = dict(self.bayesian_settings)
        kp = optuna_trial.suggest_float("kp", settings["kp_min"], settings["kp_max"])
        if settings["tune_ki"]:
            ki = optuna_trial.suggest_float("ki", settings["ki_min"], settings["ki_max"])
        else:
            ki = safe_float(settings.get("fixed_ki", 0.0), 0.0)
        kd = optuna_trial.suggest_float("kd", settings["kd_min"], settings["kd_max"])
        settings.update({"kp": kp, "ki": ki, "kd": kd})
        self.bayesian_iteration += 1
        self.bayesian_trial = optuna_trial
        self.bayesian_settings.update(settings)
        self.bayes_status_var.set("Bayesian status: preparing trial")
        self.bayes_trial_var.set(f"Trial: {self.bayesian_iteration} / {settings['trials']}")
        self.bayes_pid_var.set(f"PID in test: Kp={kp:.3f} Ki={ki:.3f} Kd={kd:.3f}")
        self.stop_robot()
        self.bayesian_phase = "waiting_start"
        self.root.after(1000, lambda: self._start_bayesian_trial(settings))

    def _start_bayesian_trial(self, settings: dict[str, Any]) -> None:
        if not self.bayesian_active or self.bayesian_stop_requested:
            return
        self.bayesian_phase = "running_trial"
        self.bayes_status_var.set("Bayesian status: running trial")
        point = NMPoint((settings["kp"], settings["kd"]), "bayesian")
        self._start_recording_test("bayesian", settings, point, enable_motors=True)

    def _complete_bayesian_trial(self, metrics: TestMetrics, reason: str) -> None:
        if self.bayesian_study is None or self.bayesian_trial is None:
            return
        settings = self.active_test_params
        kp = safe_float(settings.get("kp", 0.0))
        ki = safe_float(settings.get("ki", 0.0))
        kd = safe_float(settings.get("kd", 0.0))
        self.bayesian_study.tell(self.bayesian_trial, metrics.score)
        self.bayesian_trial = None
        status = "accepted"
        if self.bayesian_best_trial is not None and metrics.score >= self.bayesian_best_trial.score:
            status = "rejected"
        trial = AutoTuneTrial(
            self.bayesian_iteration,
            "bayesian",
            kp,
            ki,
            kd,
            metrics.score,
            metrics,
            status,
            metrics.fault_message or reason,
            self.current_csv_path,
            datetime.now().isoformat(timespec="seconds"),
        )
        self.bayesian_history.append(trial)
        if self.bayesian_best_trial is None or trial.score < self.bayesian_best_trial.score:
            self.bayesian_best_trial = trial
        self._insert_bayesian_row(trial)
        self._refresh_bayesian_labels(trial)
        self.stop_robot()
        self.bayesian_phase = "wait_after_trial"
        self.bayesian_deadline = time.monotonic() + 1.0

    def _insert_bayesian_row(self, trial: AutoTuneTrial) -> None:
        metrics = trial.metrics
        self.bayes_tree.insert("", "end", values=(
            trial.iteration,
            f"{trial.kp:.3f}",
            f"{trial.ki:.3f}",
            f"{trial.kd:.3f}",
            f"{trial.score:.3f}",
            f"{metrics.mean_abs_error:.3f}",
            f"{metrics.max_abs_error:.3f}",
            f"{metrics.angle_stddev:.3f}",
            f"{metrics.saturation_percent:.1f}",
            str(metrics.safety_stop),
            trial.fault_message,
        ))

    def _refresh_bayesian_labels(self, trial: AutoTuneTrial) -> None:
        self.bayes_score_var.set(f"Current score: {trial.score:.3f}")
        self.bayes_safety_var.set(f"Safety: {trial.metrics.safety_stop}")
        self.bayes_fault_var.set(f"Fault: {trial.fault_message}")
        if self.bayesian_best_trial is not None:
            best = self.bayesian_best_trial
            self.bayes_best_var.set(f"Best PID: score={best.score:.3f} Kp={best.kp:.3f} Ki={best.ki:.3f} Kd={best.kd:.3f}")

    def _finish_bayesian_autotune(self, reason: str) -> None:
        self.stop_robot()
        self.bayesian_active = False
        self.bayesian_phase = "idle"
        self.bayes_status_var.set(f"Bayesian status: finished ({reason})")

    def apply_bayesian_best_pid(self) -> None:
        if self.bayesian_best_trial is None:
            messagebox.showwarning("No best PID", "Run Bayesian AutoTune first.")
            return
        best = self.bayesian_best_trial
        self.kp_var.set(f"{best.kp:.3f}")
        self.ki_var.set(f"{best.ki:.3f}")
        self.kd_var.set(f"{best.kd:.3f}")
        self.send_json({"type": "set_pid", "kp": best.kp, "ki": best.ki, "kd": best.kd})

    def export_bayesian_results_csv(self) -> None:
        if not self.bayesian_history:
            messagebox.showwarning("No history", "There are no Bayesian AutoTune results to export.")
            return
        path = filedialog.asksaveasfilename(
            initialdir=str(self.output_dir),
            initialfile=f"pid_bayesian_results_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
        )
        if not path:
            return
        with Path(path).open("w", newline="", encoding="utf-8") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=BAYESIAN_HISTORY_FIELDS)
            writer.writeheader()
            for trial in self.bayesian_history:
                metrics = trial.metrics
                writer.writerow({
                    "timestamp": trial.timestamp or datetime.now().isoformat(timespec="seconds"),
                    "trial": trial.iteration,
                    "Kp": trial.kp,
                    "Ki": trial.ki,
                    "Kd": trial.kd,
                    "score": trial.score,
                    "mean_abs_error": metrics.mean_abs_error,
                    "max_abs_error": metrics.max_abs_error,
                    "angle_stddev": metrics.angle_stddev,
                    "saturation_percent": metrics.saturation_percent,
                    "safetyStop": metrics.safety_stop,
                    "faultMessage": trial.fault_message,
                })
        self.bayes_status_var.set(f"Bayesian status: exported {path}")

    def on_close(self) -> None:
        if self.worker is not None:
            try:
                self.worker.send({"type": "stop"})
                self.worker.send({"type": "disable_motors"})
            except Exception:
                pass
        self.disconnect()
        if self.csv_file is not None:
            self.csv_file.close()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    PidTunerGui(root)
    root.mainloop()


if __name__ == "__main__":
    main()
