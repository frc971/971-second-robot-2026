#!/usr/bin/env python3
"""Report every auto-enable-to-motion delay in WPILOG files. Usage: auto-start-delay.py LOG..."""

import argparse
import math
import struct
from pathlib import Path

SPEED_THRESHOLD = 0.20  # m/s; reject small odometry noise
REQUIRED_SAMPLES = 3    # require sustained movement
MAX_SAMPLE_GAP_US = 100_000


def log_events(path):
    """Read only the Driver Station and drivetrain signals needed for this benchmark."""
    with path.open("rb") as log:
        header = log.read(12)
        if len(header) != 12 or header[:6] != b"WPILOG":
            raise ValueError(f"Not a WPILOG file: {path}")
        log.seek(struct.unpack_from("<I", header, 8)[0], 1)
        entries = {}
        while descriptor := log.read(1):
            bits = descriptor[0]
            entry = int.from_bytes(log.read((bits & 3) + 1), "little")
            size = int.from_bytes(log.read(((bits >> 2) & 3) + 1), "little")
            time_us = int.from_bytes(log.read(((bits >> 4) & 7) + 1), "little")
            if entry == 0:
                data = log.read(size)
                if len(data) < 5:
                    continue
                entry_id = struct.unpack_from("<I", data, 1)[0]
                if data[0] == 1:
                    entries.pop(entry_id, None)
                elif data[0] == 0:
                    name_size = struct.unpack_from("<I", data, 5)[0]
                    name = data[9:9 + name_size].decode("utf-8", "replace")
                    for suffix, signal in (
                        ("/DriverStation/Enabled", "enabled"),
                        ("/DriverStation/Autonomous", "autonomous"),
                        ("/DriveState/Speeds", "odometry"),
                        ("/Drive/MeasuredSpeeds", "periodic"),
                    ):
                        if name.endswith(suffix):
                            entries[entry_id] = signal
                            break
                continue
            if entry in entries:
                yield entries[entry], time_us, log.read(size)
            else:
                log.seek(size, 1)


def motion_start(samples):
    streak_start = streak_end = None
    count = 0
    for time_us, speed in samples:
        if speed < SPEED_THRESHOLD:
            count = 0
        elif count == 0 or time_us - streak_end > MAX_SAMPLE_GAP_US:
            streak_start, count = time_us, 1
        else:
            count += 1
        if speed >= SPEED_THRESHOLD:
            streak_end = time_us
        if count == REQUIRED_SAMPLES:
            return streak_start
    return None


def auto_runs(path):
    enabled = autonomous = False
    latest_speed = {}
    current = None
    runs = []
    for signal, time_us, data in log_events(path):
        was_auto = enabled and autonomous
        if signal in ("enabled", "autonomous") and len(data) == 1:
            if signal == "enabled":
                enabled = data[0] != 0
            else:
                autonomous = data[0] != 0
        elif signal in ("odometry", "periodic") and len(data) == 24:
            vx, vy = struct.unpack_from("<dd", data)
            speed = math.hypot(vx, vy)
            latest_speed[signal] = (time_us, speed)
            if current is not None:
                samples = current["samples"][signal]
                if not samples or time_us > samples[-1][0]:
                    samples.append((time_us, speed))
        is_auto = enabled and autonomous
        if is_auto and not was_auto:
            current = {"enabled_us": time_us, "before": latest_speed.copy(),
                       "samples": {"odometry": [], "periodic": []}}
        elif was_auto and not is_auto and current is not None:
            runs.append(current)
            current = None
    if current is not None:
        runs.append(current)
    return runs


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs", nargs="+", type=Path)
    args = parser.parse_args()
    print(f"Motion threshold: {SPEED_THRESHOLD:.2f} m/s for {REQUIRED_SAMPLES} consecutive readings")
    found = False
    for path in args.logs:
        runs = auto_runs(path)
        print(f"\n{path.name}")
        if not runs:
            print("  No auto-enable transitions found.")
            continue
        found = True
        for number, run in enumerate(runs, 1):
            source = ("odometry" if len(run["samples"]["odometry"]) >= REQUIRED_SAMPLES
                      or not run["samples"]["periodic"] else "periodic")
            samples = run["samples"][source]
            prior = run["before"].get(source)
            if prior and 0 <= run["enabled_us"] - prior[0] <= MAX_SAMPLE_GAP_US and prior[1] >= SPEED_THRESHOLD:
                result = "already moving at enable"
            elif not samples:
                result = "no drivetrain speed data"
            elif (start := motion_start(samples)) is None:
                result = "no sustained movement detected"
            else:
                result = f"{(start - run['enabled_us']) / 1e6:.3f} s"
            print(f"  Auto {number}: {result}  ({source} speed)")
    return 0 if found else 1


if __name__ == "__main__":
    raise SystemExit(main())