#!/usr/bin/env python3
"""Bayesian autotuning for the balancing robot over WebSocket.

The script only sends the same JSON commands as the web UI. Motor writes,
hard safety and OTA remain in the ESP32 firmware.

Install dependencies with:
    pip install -r tools/requirements.txt
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

import optuna
import websocket


TRIAL_FIELDS = [
    "timestamp",
    "trial",
    "elapsed_s",
    "kp",
    "ki",
    "kd",
    "setpoint",
    "maxPwm",
    "integralLimit",
    "iTermLimit",
    "integralEnabled",
    "motorDeadzonePwm",
    "encoderSyncEnabled",
    "encoderSyncKp",
    "encoderSyncDeadband",
    "encoderSyncMaxCorrection",
    "encoderSyncTargetDifference",
    "angle",
    "pidError",
    "gyroRate",
    "pidOutput",
    "outputBeforeLimit",
    "outputAfterLimit",
    "pTerm",
    "iTerm",
    "dTerm",
    "integral",
    "leftPwm",
    "rightPwm",
    "leftSpeed",
    "rightSpeed",
    "speedDifference",
    "encoderSyncError",
    "encoderSyncCorrection",
    "motorsEnabled",
    "safetyStop",
    "faultMessage",
]

SUMMARY_FIELDS = [
    "trial",
    "score",
    "samples",
    "mean_abs_error",
    "max_abs_error",
    "angle_stddev",
    "saturation_percent",
    "mean_abs_speed_difference",
    "mean_abs_encoder_sync_error",
    "mean_abs_pwm",
    "safetyStop",
    "faultMessage",
    "csv_path",
    "kp",
    "ki",
    "kd",
    "setpoint",
    "maxPwm",
    "integralLimit",
    "iTermLimit",
    "integralEnabled",
    "motorDeadzonePwm",
    "encoderSyncEnabled",
    "encoderSyncKp",
    "encoderSyncDeadband",
    "encoderSyncMaxCorrection",
    "encoderSyncTargetDifference",
]


@dataclass
class TrialMetrics:
    score: float
    samples: int
    mean_abs_error: float
    max_abs_error: float
    angle_stddev: float
    saturation_percent: float
    mean_abs_speed_difference: float
    mean_abs_encoder_sync_error: float
    mean_abs_pwm: float
    safety_stop: bool
    fault_message: str


def safe_float(value: Any, default: float = 0.0) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    if not math.isfinite(result):
        return default
    return result


def safe_int(value: Any, default: int = 0) -> int:
    try:
        return int(float(value))
    except (TypeError, ValueError):
        return default


def safe_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "y", "on"}


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Bayesian autotuning for robot balance parameters")
    parser.add_argument("--host", required=True, help="ESP32 IP address")
    parser.add_argument("--port", type=int, default=81, help="WebSocket port")
    parser.add_argument("--trials", type=int, default=40, help="Number of Bayesian trials")
    parser.add_argument("--duration", type=float, default=4.0, help="Seconds per trial")
    parser.add_argument("--output-dir", default="tools/tuning_logs", help="Directory for logs")
    parser.add_argument("--timeout", type=float, default=2.0, help="WebSocket receive timeout")
    parser.add_argument("--yes", action="store_true", help="Skip confirmation prompt")
    parser.add_argument("--dry-run", action="store_true", help="Do not enable motors")
    parser.add_argument("--apply-best", action="store_true", help="Apply best parameters at the end, motors disabled")
    parser.add_argument("--safe-angle", type=float, default=25.0, help="Stop trial if abs(angle-setpoint) exceeds this")
    parser.add_argument("--arming-angle", type=float, default=5.0, help="Angle error required before enabling motors")
    parser.add_argument("--arming-stable", type=float, default=2.0, help="Seconds angle must remain near setpoint")
    parser.add_argument("--startup-wait", type=float, default=0.25, help="Seconds to wait after applying params")
    parser.add_argument("--sampler-seed", type=int, default=None, help="Optional Optuna sampler seed")
    parser.add_argument("--disable-integral-search", action="store_true", help="Keep integral enabled and fixed")
    parser.add_argument("--disable-encoder-sync-search", action="store_true", help="Disable encoder sync for all trials")
    parser.add_argument("--fixed-setpoint", type=float, default=None, help="Use fixed setpoint instead of optimizing")
    parser.add_argument("--fixed-max-pwm", type=int, default=None, help="Use fixed max PWM instead of optimizing")

    parser.add_argument("--kp-min", type=float, default=5.0)
    parser.add_argument("--kp-max", type=float, default=60.0)
    parser.add_argument("--ki-min", type=float, default=0.0)
    parser.add_argument("--ki-max", type=float, default=400.0)
    parser.add_argument("--kd-min", type=float, default=0.0)
    parser.add_argument("--kd-max", type=float, default=8.0)
    parser.add_argument("--setpoint-min", type=float, default=-3.0)
    parser.add_argument("--setpoint-max", type=float, default=3.0)
    parser.add_argument("--max-pwm-min", type=int, default=80)
    parser.add_argument("--max-pwm-max", type=int, default=220)
    parser.add_argument("--integral-limit-min", type=float, default=0.0)
    parser.add_argument("--integral-limit-max", type=float, default=0.8)
    parser.add_argument("--i-term-limit-min", type=float, default=0.0)
    parser.add_argument("--i-term-limit-max", type=float, default=80.0)
    parser.add_argument("--deadzone-min", type=int, default=0)
    parser.add_argument("--deadzone-max", type=int, default=120)
    parser.add_argument("--sync-kp-min", type=float, default=0.0)
    parser.add_argument("--sync-kp-max", type=float, default=0.2)
    parser.add_argument("--sync-deadband-min", type=float, default=0.0)
    parser.add_argument("--sync-deadband-max", type=float, default=50.0)
    parser.add_argument("--sync-max-correction-min", type=int, default=0)
    parser.add_argument("--sync-max-correction-max", type=int, default=40)
    parser.add_argument("--sync-target-min", type=float, default=-80.0)
    parser.add_argument("--sync-target-max", type=float, default=80.0)
    return parser


def validate_args(args: argparse.Namespace) -> None:
    if args.trials <= 0:
        raise ValueError("--trials must be positive")
    if args.duration <= 0:
        raise ValueError("--duration must be positive")
    if args.kp_min >= args.kp_max or args.ki_min >= args.ki_max or args.kd_min >= args.kd_max:
        raise ValueError("PID min values must be lower than max values")
    if args.setpoint_min >= args.setpoint_max:
        raise ValueError("setpoint min must be lower than max")
    if args.max_pwm_min > args.max_pwm_max:
        raise ValueError("max PWM min must be <= max")
    if args.integral_limit_min > args.integral_limit_max:
        raise ValueError("integral limit min must be <= max")
    if args.i_term_limit_min > args.i_term_limit_max:
        raise ValueError("I term limit min must be <= max")
    if args.deadzone_min > args.deadzone_max:
        raise ValueError("deadzone min must be <= max")
    if args.sync_target_min > args.sync_target_max:
        raise ValueError("sync target min must be <= max")


def send_command(ws: websocket.WebSocket, command: dict[str, Any]) -> None:
    ws.send(json.dumps(command, separators=(",", ":")))


def receive_state(ws: websocket.WebSocket, timeout_s: float) -> dict[str, Any] | None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            message = ws.recv()
        except websocket.WebSocketTimeoutException:
            continue
        data: dict[str, Any]
        try:
            data = json.loads(message)
        except json.JSONDecodeError:
            continue
        if data.get("type") in {"ack", "error"}:
            print(f"Robot: {data}")
            continue
        return data
    return None


def connect_ws(args: argparse.Namespace) -> websocket.WebSocket:
    url = f"ws://{args.host}:{args.port}/"
    print(f"Connecting to {url}")
    ws = websocket.create_connection(url, timeout=args.timeout)
    ws.settimeout(0.1)
    return ws


def suggest_params(trial: optuna.trial.Trial, args: argparse.Namespace) -> dict[str, Any]:
    params: dict[str, Any] = {
        "kp": trial.suggest_float("kp", args.kp_min, args.kp_max),
        "ki": trial.suggest_float("ki", args.ki_min, args.ki_max),
        "kd": trial.suggest_float("kd", args.kd_min, args.kd_max),
        "setpoint": args.fixed_setpoint
        if args.fixed_setpoint is not None
        else trial.suggest_float("setpoint", args.setpoint_min, args.setpoint_max),
        "maxPwm": args.fixed_max_pwm
        if args.fixed_max_pwm is not None
        else trial.suggest_int("maxPwm", args.max_pwm_min, args.max_pwm_max),
        "motorDeadzonePwm": trial.suggest_int("motorDeadzonePwm", args.deadzone_min, args.deadzone_max),
    }

    if args.disable_integral_search:
        params.update({"integralEnabled": True, "integralLimit": 0.25, "iTermLimit": 40.0})
    else:
        params.update(
            {
                "integralEnabled": trial.suggest_categorical("integralEnabled", [True, False]),
                "integralLimit": trial.suggest_float("integralLimit", args.integral_limit_min, args.integral_limit_max),
                "iTermLimit": trial.suggest_float("iTermLimit", args.i_term_limit_min, args.i_term_limit_max),
            }
        )

    if args.disable_encoder_sync_search:
        params.update(
            {
                "encoderSyncEnabled": False,
                "encoderSyncKp": 0.0,
                "encoderSyncDeadband": 5.0,
                "encoderSyncMaxCorrection": 0,
                "encoderSyncTargetDifference": 0.0,
            }
        )
    else:
        params.update(
            {
                "encoderSyncEnabled": trial.suggest_categorical("encoderSyncEnabled", [True, False]),
                "encoderSyncKp": trial.suggest_float("encoderSyncKp", args.sync_kp_min, args.sync_kp_max),
                "encoderSyncDeadband": trial.suggest_float("encoderSyncDeadband", args.sync_deadband_min, args.sync_deadband_max),
                "encoderSyncMaxCorrection": trial.suggest_int(
                    "encoderSyncMaxCorrection", args.sync_max_correction_min, args.sync_max_correction_max
                ),
                "encoderSyncTargetDifference": trial.suggest_float(
                    "encoderSyncTargetDifference", args.sync_target_min, args.sync_target_max
                ),
            }
        )
    return params


def apply_params(ws: websocket.WebSocket, params: dict[str, Any]) -> None:
    send_command(ws, {"type": "disable_motors"})
    send_command(ws, {"type": "reset_integral"})
    send_command(ws, {"type": "set_pid", "kp": params["kp"], "ki": params["ki"], "kd": params["kd"]})
    send_command(ws, {"type": "set_setpoint", "setpoint": params["setpoint"]})
    send_command(ws, {"type": "set_pwm_limit", "maxPwm": params["maxPwm"]})
    send_command(ws, {"type": "set_integral_limit", "integralLimit": params["integralLimit"]})
    send_command(ws, {"type": "set_i_term_limit", "iTermLimit": params["iTermLimit"]})
    send_command(ws, {"type": "enable_integral", "enabled": params["integralEnabled"]})
    send_command(ws, {"type": "set_motor_deadzone", "deadzonePwm": params["motorDeadzonePwm"]})
    send_command(
        ws,
        {
            "type": "set_encoder_sync",
            "kp": params["encoderSyncKp"],
            "deadband": params["encoderSyncDeadband"],
            "maxCorrection": params["encoderSyncMaxCorrection"],
        },
    )
    send_command(ws, {"type": "set_encoder_sync_target", "targetDifference": params["encoderSyncTargetDifference"]})
    send_command(ws, {"type": "enable_encoder_sync", "enabled": params["encoderSyncEnabled"]})


def wait_until_armable(ws: websocket.WebSocket, args: argparse.Namespace, setpoint: float) -> bool:
    stable_since: float | None = None
    deadline = time.monotonic() + max(10.0, args.arming_stable + 5.0)
    while time.monotonic() < deadline:
        state = receive_state(ws, args.timeout)
        if state is None:
            continue
        angle = safe_float(state.get("selectedAngle", state.get("angle", 999.0)), 999.0)
        if bool(state.get("otaUpdating", False)):
            return False
        if abs(angle - setpoint) <= args.arming_angle and not bool(state.get("safetyStop", False)):
            stable_since = stable_since or time.monotonic()
            if time.monotonic() - stable_since >= args.arming_stable:
                return True
        else:
            stable_since = None
    return False


def row_from_state(trial_number: int, elapsed_s: float, params: dict[str, Any], state: dict[str, Any]) -> dict[str, Any]:
    angle = safe_float(state.get("selectedAngle", state.get("angle", 0.0)))
    error = safe_float(state.get("pidError", params["setpoint"] - angle))
    return {
        "timestamp": time.time(),
        "trial": trial_number,
        "elapsed_s": elapsed_s,
        **params,
        "angle": angle,
        "pidError": error,
        "gyroRate": safe_float(state.get("gyroRate")),
        "pidOutput": safe_float(state.get("pidOutput")),
        "outputBeforeLimit": safe_float(state.get("outputBeforeLimit")),
        "outputAfterLimit": safe_float(state.get("outputAfterLimit")),
        "pTerm": safe_float(state.get("pTerm")),
        "iTerm": safe_float(state.get("iTerm")),
        "dTerm": safe_float(state.get("dTerm")),
        "integral": safe_float(state.get("integral")),
        "leftPwm": safe_int(state.get("leftPwm")),
        "rightPwm": safe_int(state.get("rightPwm")),
        "leftSpeed": safe_float(state.get("leftSpeed")),
        "rightSpeed": safe_float(state.get("rightSpeed")),
        "speedDifference": safe_float(state.get("speedDifference")),
        "encoderSyncError": safe_float(state.get("encoderSyncError")),
        "encoderSyncCorrection": safe_int(state.get("encoderSyncCorrection")),
        "motorsEnabled": bool(state.get("motorsEnabled", False)),
        "safetyStop": bool(state.get("safetyStop", True)),
        "faultMessage": str(state.get("faultMessage", "")),
    }


def compute_metrics(rows: list[dict[str, Any]], params: dict[str, Any], args: argparse.Namespace) -> TrialMetrics:
    if len(rows) < 3:
        return TrialMetrics(12000.0, len(rows), 999.0, 999.0, 999.0, 100.0, 999.0, 999.0, 999.0, True, "too few samples")

    errors = [abs(safe_float(row["pidError"])) for row in rows]
    angles = [safe_float(row["angle"]) for row in rows]
    outputs = [abs(safe_float(row["pidOutput"])) for row in rows]
    speed_diffs = [abs(safe_float(row["speedDifference"])) for row in rows]
    sync_errors = [abs(safe_float(row["encoderSyncError"])) for row in rows]
    pwms = [(abs(safe_float(row["leftPwm"])) + abs(safe_float(row["rightPwm"]))) * 0.5 for row in rows]
    safety_stop = any(bool(row["safetyStop"]) for row in rows)
    fault_messages = [str(row["faultMessage"]) for row in rows if str(row["faultMessage"])]
    max_pwm = max(1.0, safe_float(params["maxPwm"], 1.0))
    saturated = sum(1 for output in outputs if output >= 0.95 * max_pwm)

    mean_abs_error = statistics.fmean(errors)
    max_abs_error = max(errors)
    angle_stddev = statistics.pstdev(angles) if len(angles) > 1 else 0.0
    saturation_percent = 100.0 * saturated / len(rows)
    mean_abs_speed_difference = statistics.fmean(speed_diffs)
    mean_abs_encoder_sync_error = statistics.fmean(sync_errors)
    mean_abs_pwm = statistics.fmean(pwms)

    score = (
        mean_abs_error
        + 0.6 * angle_stddev
        + 0.2 * max_abs_error
        + 0.05 * saturation_percent
        + 0.03 * mean_abs_speed_difference
        + 0.05 * mean_abs_encoder_sync_error
        + 0.02 * mean_abs_pwm
    )
    if safety_stop:
        score += 10000.0
    if max_abs_error > args.safe_angle:
        score += 5000.0
    return TrialMetrics(
        score,
        len(rows),
        mean_abs_error,
        max_abs_error,
        angle_stddev,
        saturation_percent,
        mean_abs_speed_difference,
        mean_abs_encoder_sync_error,
        mean_abs_pwm,
        safety_stop,
        fault_messages[-1] if fault_messages else "",
    )


def write_summary_row(path: Path, row: dict[str, Any]) -> None:
    exists = path.exists()
    with path.open("a", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=SUMMARY_FIELDS)
        if not exists:
            writer.writeheader()
        writer.writerow(row)


def run_trial(ws: websocket.WebSocket, trial_number: int, params: dict[str, Any], args: argparse.Namespace, output_dir: Path) -> tuple[TrialMetrics, Path]:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = output_dir / f"robot_bayes_trial_{timestamp}_{trial_number:03d}.csv"
    apply_params(ws, params)
    time.sleep(args.startup_wait)

    if not args.dry_run:
        armable = wait_until_armable(ws, args, safe_float(params["setpoint"]))
        if not armable:
            send_command(ws, {"type": "stop"})
            send_command(ws, {"type": "disable_motors"})
            metrics = TrialMetrics(15000.0, 0, 999.0, 999.0, 999.0, 100.0, 999.0, 999.0, 999.0, True, "arming failed")
            return metrics, csv_path
        send_command(ws, {"type": "enable_motors"})

    rows: list[dict[str, Any]] = []
    started = time.monotonic()
    with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=TRIAL_FIELDS)
        writer.writeheader()
        while time.monotonic() - started < args.duration:
            state = receive_state(ws, args.timeout)
            if state is None:
                continue
            elapsed_s = time.monotonic() - started
            row = row_from_state(trial_number, elapsed_s, params, state)
            rows.append(row)
            writer.writerow(row)
            if abs(row["pidError"]) > args.safe_angle or row["safetyStop"]:
                break

    send_command(ws, {"type": "stop"})
    send_command(ws, {"type": "disable_motors"})
    metrics = compute_metrics(rows, params, args)
    return metrics, csv_path


def summary_row(trial_number: int, params: dict[str, Any], metrics: TrialMetrics, csv_path: Path) -> dict[str, Any]:
    return {
        "trial": trial_number,
        "score": metrics.score,
        "samples": metrics.samples,
        "mean_abs_error": metrics.mean_abs_error,
        "max_abs_error": metrics.max_abs_error,
        "angle_stddev": metrics.angle_stddev,
        "saturation_percent": metrics.saturation_percent,
        "mean_abs_speed_difference": metrics.mean_abs_speed_difference,
        "mean_abs_encoder_sync_error": metrics.mean_abs_encoder_sync_error,
        "mean_abs_pwm": metrics.mean_abs_pwm,
        "safetyStop": metrics.safety_stop,
        "faultMessage": metrics.fault_message,
        "csv_path": str(csv_path),
        **params,
    }


def confirm(args: argparse.Namespace) -> None:
    if args.yes:
        return
    print("This autotuner can enable motors for multiple trials.")
    print("Use a safe support, keep hands clear, and be ready to cut power.")
    answer = input("Type YES to continue: ").strip()
    if answer != "YES":
        raise SystemExit("Cancelled")


def save_best(path: Path, params: dict[str, Any], metrics: TrialMetrics) -> None:
    payload = {"params": params, "metrics": metrics.__dict__}
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def main() -> int:
    args = build_arg_parser().parse_args()
    validate_args(args)
    confirm(args)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    summary_path = output_dir / f"robot_bayes_summary_{timestamp}.csv"
    best_path = output_dir / f"robot_bayes_best_{timestamp}.json"

    sampler = optuna.samplers.TPESampler(seed=args.sampler_seed)
    study = optuna.create_study(direction="minimize", sampler=sampler)
    ws = connect_ws(args)
    best_params: dict[str, Any] | None = None
    best_metrics: TrialMetrics | None = None

    try:
        def objective(trial: optuna.trial.Trial) -> float:
            nonlocal best_params, best_metrics
            params = suggest_params(trial, args)
            trial_number = trial.number + 1
            print(f"Trial {trial_number}/{args.trials}: {params}")
            metrics, csv_path = run_trial(ws, trial_number, params, args, output_dir)
            row = summary_row(trial_number, params, metrics, csv_path)
            write_summary_row(summary_path, row)
            if best_metrics is None or metrics.score < best_metrics.score:
                best_params = dict(params)
                best_metrics = metrics
                save_best(best_path, best_params, best_metrics)
            print(
                f"score={metrics.score:.3f} mean={metrics.mean_abs_error:.3f} "
                f"max={metrics.max_abs_error:.3f} std={metrics.angle_stddev:.3f} "
                f"safety={metrics.safety_stop} fault={metrics.fault_message}"
            )
            return metrics.score

        study.optimize(objective, n_trials=args.trials)
    finally:
        try:
            send_command(ws, {"type": "stop"})
            send_command(ws, {"type": "disable_motors"})
        finally:
            ws.close()

    print(f"Summary: {summary_path}")
    if best_params is not None and best_metrics is not None:
        print(f"Best JSON: {best_path}")
        print(f"Best score: {best_metrics.score:.3f}")
        print(json.dumps(best_params, indent=2))
        if args.apply_best:
            ws = connect_ws(args)
            try:
                apply_params(ws, best_params)
                send_command(ws, {"type": "disable_motors"})
            finally:
                ws.close()
            print("Best parameters applied. Motors left disabled.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("Interrupted", file=sys.stderr)
        raise SystemExit(130)
