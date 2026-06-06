#!/usr/bin/env python3
"""Assisted PID tuning over the ESP32 debug WebSocket.

The script only sends the same JSON commands as the web UI. Motor control and
safety decisions remain in the ESP32 control task.
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

import websocket


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


@dataclass
class TestResult:
    kp: float
    ki: float
    kd: float
    csv_path: Path
    samples: int
    mean_abs_error: float
    max_abs_error: float
    angle_stddev: float
    saturation_percent: float
    near_setpoint_percent: float
    safety_stop: bool
    score: float


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Tune balance PID over WebSocket")
    parser.add_argument("--host", required=True, help="ESP32 IP address, e.g. 192.168.1.11")
    parser.add_argument("--port", type=int, default=81, help="WebSocket port")
    parser.add_argument("--duration", type=float, default=5.0, help="Seconds per test")
    parser.add_argument("--kp", type=float, required=True, help="Initial Kp")
    parser.add_argument("--ki", type=float, default=0.0, help="Initial Ki, kept fixed by default")
    parser.add_argument("--kd", type=float, required=True, help="Initial Kd")
    parser.add_argument("--kp-step", type=float, default=2.0, help="Small Kp step for sweep")
    parser.add_argument("--kd-step", type=float, default=0.2, help="Small Kd step for sweep")
    parser.add_argument("--max-pwm", type=int, default=120, help="PID PWM limit")
    parser.add_argument("--setpoint", type=float, default=0.0, help="Angle setpoint in degrees")
    parser.add_argument("--output-dir", default="tools/pid_logs", help="Directory for CSV logs")
    parser.add_argument("--near-deg", type=float, default=2.0, help="Near-setpoint error threshold")
    parser.add_argument("--timeout", type=float, default=2.0, help="WebSocket receive timeout")
    parser.add_argument("--no-enable", action="store_true", help="Do not enable motors; record PID response only")
    parser.add_argument("--single", action="store_true", help="Run only the provided Kp/Ki/Kd")
    return parser


def validate_args(args: argparse.Namespace) -> None:
    if args.duration <= 0:
        raise ValueError("--duration must be positive")
    if not 0.0 <= args.kp <= 100.0:
        raise ValueError("--kp must be between 0 and 100")
    if not 0.0 <= args.ki <= 20.0:
        raise ValueError("--ki must be between 0 and 20")
    if not 0.0 <= args.kd <= 20.0:
        raise ValueError("--kd must be between 0 and 20")
    if not 0 <= args.max_pwm <= 180:
        raise ValueError("--max-pwm must be between 0 and 180 for assisted tuning")
    if not -10.0 <= args.setpoint <= 10.0:
        raise ValueError("--setpoint must be between -10 and 10 degrees")
    if args.kp_step < 0 or args.kd_step < 0:
        raise ValueError("PID sweep steps must be non-negative")


def make_combinations(args: argparse.Namespace) -> list[tuple[float, float, float]]:
    if args.single:
        return [(args.kp, args.ki, args.kd)]

    kp_values = sorted({clamp(args.kp - args.kp_step, 0.0, 100.0), args.kp, clamp(args.kp + args.kp_step, 0.0, 100.0)})
    kd_values = sorted({clamp(args.kd - args.kd_step, 0.0, 20.0), args.kd, clamp(args.kd + args.kd_step, 0.0, 20.0)})
    return [(kp, args.ki, kd) for kp in kp_values for kd in kd_values]


def connect_ws(host: str, port: int, timeout: float) -> websocket.WebSocket:
    url = f"ws://{host}:{port}/"
    print(f"Connecting to {url}")
    ws = websocket.create_connection(url, timeout=timeout)
    ws.settimeout(timeout)
    return ws


def send_command(ws: websocket.WebSocket, command: dict[str, Any]) -> None:
    ws.send(json.dumps(command, separators=(",", ":")))


def receive_state(ws: websocket.WebSocket, deadline: float) -> dict[str, Any] | None:
    while time.monotonic() < deadline:
        try:
            message = ws.recv()
        except websocket.WebSocketTimeoutException:
            continue

        try:
            data = json.loads(message)
        except json.JSONDecodeError:
            continue

        if data.get("type") in {"ack", "error"}:
            print(f"Robot: {data}")
            continue

        return data
    return None


def require_confirmation(kp: float, ki: float, kd: float, max_pwm: int, no_enable: bool) -> bool:
    if no_enable:
        print("Motors will remain disabled for this test.")
        return False

    print()
    print(f"Next test: Kp={kp:.3f} Ki={ki:.3f} Kd={kd:.3f} maxPwm={max_pwm}")
    print("Place the robot on a safe support or hold it securely.")
    answer = input("Type YES to enable motors for this test: ").strip()
    return answer == "YES"


def row_from_state(state: dict[str, Any], elapsed_s: float, kp: float, ki: float, kd: float, setpoint: float, max_pwm: int) -> dict[str, Any]:
    angle = float(state.get("angle", state.get("filteredAngle", 0.0)))
    error = float(state.get("pidError", setpoint - angle))
    return {
        "timestamp": time.time(),
        "elapsed_s": elapsed_s,
        "kp": kp,
        "ki": ki,
        "kd": kd,
        "setpoint": setpoint,
        "maxPwm": max_pwm,
        "angle": angle,
        "error": error,
        "pidOutput": float(state.get("pidOutput", 0.0)),
        "leftPwm": int(state.get("leftPwm", 0)),
        "rightPwm": int(state.get("rightPwm", 0)),
        "motorsEnabled": bool(state.get("motorsEnabled", False)),
        "safetyStop": bool(state.get("safetyStop", True)),
        "faultMessage": str(state.get("faultMessage", "")),
    }


def compute_metrics(rows: list[dict[str, Any]], max_pwm: int, near_deg: float) -> dict[str, float | bool]:
    if not rows:
        return {
            "mean_abs_error": math.inf,
            "max_abs_error": math.inf,
            "angle_stddev": math.inf,
            "saturation_percent": 100.0,
            "near_setpoint_percent": 0.0,
            "safety_stop": True,
            "score": math.inf,
        }

    errors = [abs(float(row["error"])) for row in rows]
    angles = [float(row["angle"]) for row in rows]
    outputs = [abs(float(row["pidOutput"])) for row in rows]
    safety_stop = any(bool(row["safetyStop"]) for row in rows)
    saturation_threshold = 0.95 * max_pwm
    saturated = sum(1 for output in outputs if max_pwm > 0 and output >= saturation_threshold)
    near = sum(1 for error in errors if error <= near_deg)

    mean_abs_error = statistics.fmean(errors)
    max_abs_error = max(errors)
    angle_stddev = statistics.pstdev(angles) if len(angles) > 1 else 0.0
    saturation_percent = 100.0 * saturated / len(rows)
    near_setpoint_percent = 100.0 * near / len(rows)
    score = mean_abs_error + 0.5 * angle_stddev + 0.05 * saturation_percent
    if safety_stop:
        score += 1000.0

    return {
        "mean_abs_error": mean_abs_error,
        "max_abs_error": max_abs_error,
        "angle_stddev": angle_stddev,
        "saturation_percent": saturation_percent,
        "near_setpoint_percent": near_setpoint_percent,
        "safety_stop": safety_stop,
        "score": score,
    }


def run_test(ws: websocket.WebSocket, args: argparse.Namespace, kp: float, ki: float, kd: float, output_dir: Path) -> TestResult:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = output_dir / f"pid_test_{timestamp}_kp{kp:.2f}_ki{ki:.2f}_kd{kd:.2f}.csv"

    send_command(ws, {"type": "set_pid", "kp": kp, "ki": ki, "kd": kd})
    send_command(ws, {"type": "set_setpoint", "setpoint": args.setpoint})
    send_command(ws, {"type": "set_pwm_limit", "maxPwm": args.max_pwm})
    time.sleep(0.2)

    enable = require_confirmation(kp, ki, kd, args.max_pwm, args.no_enable)
    if enable:
        send_command(ws, {"type": "enable_motors"})

    rows: list[dict[str, Any]] = []
    started = time.monotonic()

    with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)
        writer.writeheader()

        while time.monotonic() - started < args.duration:
            state = receive_state(ws, time.monotonic() + args.timeout)
            if state is None:
                print("No state received before timeout")
                continue

            elapsed_s = time.monotonic() - started
            row = row_from_state(state, elapsed_s, kp, ki, kd, args.setpoint, args.max_pwm)
            rows.append(row)
            writer.writerow(row)

            if row["safetyStop"]:
                print(f"Safety stop detected: {row['faultMessage']}")
                send_command(ws, {"type": "stop"})
                send_command(ws, {"type": "disable_motors"})
                break

    send_command(ws, {"type": "disable_motors"})
    metrics = compute_metrics(rows, args.max_pwm, args.near_deg)
    return TestResult(
        kp=kp,
        ki=ki,
        kd=kd,
        csv_path=csv_path,
        samples=len(rows),
        mean_abs_error=float(metrics["mean_abs_error"]),
        max_abs_error=float(metrics["max_abs_error"]),
        angle_stddev=float(metrics["angle_stddev"]),
        saturation_percent=float(metrics["saturation_percent"]),
        near_setpoint_percent=float(metrics["near_setpoint_percent"]),
        safety_stop=bool(metrics["safety_stop"]),
        score=float(metrics["score"]),
    )


def print_summary(results: list[TestResult]) -> None:
    if not results:
        print("No tests completed")
        return

    results = sorted(results, key=lambda result: result.score)
    header = (
        "rank kp     ki     kd     samples mean_err max_err  angle_sd sat_%  near_% safety score   csv"
    )
    print()
    print(header)
    print("-" * len(header))
    for index, result in enumerate(results, start=1):
        print(
            f"{index:<4} {result.kp:<6.2f} {result.ki:<6.2f} {result.kd:<6.2f} "
            f"{result.samples:<7} {result.mean_abs_error:<8.3f} {result.max_abs_error:<8.3f} "
            f"{result.angle_stddev:<8.3f} {result.saturation_percent:<6.1f} "
            f"{result.near_setpoint_percent:<6.1f} {str(result.safety_stop):<6} "
            f"{result.score:<7.3f} {result.csv_path}"
        )

    best = results[0]
    print()
    print("Recommended initial values:")
    print(f"Kp={best.kp:.3f} Ki={best.ki:.3f} Kd={best.kd:.3f}")


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()

    try:
        validate_args(args)
    except ValueError as exc:
        parser.error(str(exc))

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    combinations = make_combinations(args)
    print(f"Planned tests: {len(combinations)}")

    ws: websocket.WebSocket | None = None
    results: list[TestResult] = []
    try:
        ws = connect_ws(args.host, args.port, args.timeout)
        for kp, ki, kd in combinations:
            result = run_test(ws, args, kp, ki, kd, output_dir)
            results.append(result)
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        if ws is not None:
            try:
                send_command(ws, {"type": "stop"})
                send_command(ws, {"type": "disable_motors"})
                ws.close()
            except Exception as exc:  # noqa: BLE001 - best effort safety shutdown
                print(f"Warning: could not send final stop: {exc}", file=sys.stderr)

    print_summary(results)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
