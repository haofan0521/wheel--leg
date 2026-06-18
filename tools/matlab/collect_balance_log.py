#!/usr/bin/env python3
"""
从轮腿机器人 WiFi 调试接口采集平衡日志，生成 MATLAB 可读的 balance_log.csv。

用法示例：
  python tools/matlab/collect_balance_log.py --host 192.168.4.1 --duration 20 --rate 20
  python tools/matlab/collect_balance_log.py --url http://wheel-leg-debug.local/api/status --duration 30
"""

import argparse
import csv
import json
import time
import urllib.error
import urllib.request


DEFAULT_COLUMNS = [
    "time_ms",
    "pc_time_s",
    "sample_index",
    "pitch_deg",
    "target_pitch_deg",
    "pitch_rate_dps",
    "wheel_velocity",
    "output_velocity",
    "balance_active",
    "balance_enabled",
    "emergency_stopped",
    "balance_fault",
    "kp",
    "kd",
    "kv",
    "direction",
    "max_velocity",
    "start_angle_deg",
    "max_angle_deg",
    "left_target_velocity",
    "left_measured_velocity",
    "right_target_velocity",
    "right_measured_velocity",
]


def parse_args():
    parser = argparse.ArgumentParser(
        description="Poll /api/status and write balance_log.csv."
    )
    parser.add_argument(
        "--host",
        default="wheel-leg-debug.local",
        help="Robot host or IP, for example 192.168.4.1.",
    )
    parser.add_argument(
        "--url",
        default="",
        help="Full status URL. Overrides --host when provided.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=20.0,
        help="Capture duration in seconds.",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=20.0,
        help="Polling rate in Hz. 10-30 Hz is usually enough for identification.",
    )
    parser.add_argument(
        "--output",
        default="balance_log.csv",
        help="Output CSV path.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=0.5,
        help="HTTP timeout in seconds.",
    )
    return parser.parse_args()


def status_url(args):
    if args.url:
        return args.url
    host = args.host
    if host.startswith("http://") or host.startswith("https://"):
        return host.rstrip("/") + "/api/status"
    return "http://" + host.rstrip("/") + "/api/status"


def fetch_json(url, timeout):
    with urllib.request.urlopen(url, timeout=timeout) as response:
        return json.loads(response.read().decode("utf-8"))


def fget(data, key, default=""):
    value = data.get(key, default)
    if isinstance(value, bool):
        return int(value)
    return value


def build_row(sample_index, start_time, data):
    balance = data.get("balance", {})
    left = data.get("leftMotor", {})
    right = data.get("rightMotor", {})
    now = time.monotonic()

    return {
        "time_ms": int(round((now - start_time) * 1000.0)),
        "pc_time_s": f"{time.time():.6f}",
        "sample_index": sample_index,
        "pitch_deg": fget(balance, "pitch"),
        "target_pitch_deg": fget(balance, "targetPitch"),
        "pitch_rate_dps": fget(balance, "pitchRate"),
        "wheel_velocity": fget(balance, "wheelVelocity"),
        "output_velocity": fget(balance, "outputVelocity"),
        "balance_active": fget(balance, "active"),
        "balance_enabled": fget(balance, "enabled"),
        "emergency_stopped": fget(balance, "emergencyStopped"),
        "balance_fault": fget(balance, "fault"),
        "kp": fget(balance, "kp"),
        "kd": fget(balance, "kd"),
        "kv": fget(balance, "kv"),
        "direction": fget(balance, "direction"),
        "max_velocity": fget(balance, "maxVelocity"),
        "start_angle_deg": fget(balance, "startAngle"),
        "max_angle_deg": fget(balance, "maxAngle"),
        "left_target_velocity": fget(left, "targetVelocity"),
        "left_measured_velocity": fget(left, "measuredVelocity"),
        "right_target_velocity": fget(right, "targetVelocity"),
        "right_measured_velocity": fget(right, "measuredVelocity"),
    }


def main():
    args = parse_args()
    url = status_url(args)
    period = 1.0 / max(args.rate, 0.1)
    deadline = time.monotonic() + max(args.duration, 0.0)
    start_time = time.monotonic()
    next_tick = start_time
    sample_index = 0
    error_count = 0

    print(f"status url: {url}")
    print(f"writing: {args.output}")
    print("press Ctrl+C to stop early")

    with open(args.output, "w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=DEFAULT_COLUMNS)
        writer.writeheader()

        try:
            while time.monotonic() < deadline:
                now = time.monotonic()
                if now < next_tick:
                    time.sleep(next_tick - now)
                next_tick += period

                try:
                    data = fetch_json(url, args.timeout)
                    writer.writerow(build_row(sample_index, start_time, data))
                    csv_file.flush()
                    sample_index += 1
                except (urllib.error.URLError, TimeoutError, json.JSONDecodeError) as exc:
                    error_count += 1
                    print(f"sample error {error_count}: {exc}")
        except KeyboardInterrupt:
            print("stopped by user")

    print(f"done: {sample_index} samples, {error_count} errors")


if __name__ == "__main__":
    main()
