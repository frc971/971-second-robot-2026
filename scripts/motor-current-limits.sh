#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

python3 - "$REPO_ROOT" <<'PY'
import re
import sys
from pathlib import Path

repo_root = Path(sys.argv[1])
src_root = repo_root / "src" / "main" / "java"
superstructure_dir = src_root / "frc" / "robot" / "subsystems" / "superstructure"
tuner_constants_path = src_root / "frc" / "robot" / "generated" / "TunerConstants.java"
motor_config_path = src_root / "frc" / "robot" / "lib" / "superstructure" / "MotorConfig.java"

rows = []

default_foc = True
motor_config_text = motor_config_path.read_text()
default_foc_match = re.search(r"@Builder\.Default\s+private\s+boolean\s+FOC\s*=\s*(true|false)\s*;", motor_config_text)
if default_foc_match:
    default_foc = default_foc_match.group(1) == "true"


def parse_builder_value(text: str, key: str):
    m = re.search(rf"\.{re.escape(key)}\(([^)]+)\)", text)
    if not m:
        return None
    return m.group(1).strip()


def parse_motor_config_from_file(path: Path):
    text = path.read_text()
    rel = path.relative_to(repo_root).as_posix()

    if "public static MotorConfig getMotorConfig()" not in text:
        return

    if "tc.CurrentLimits.SupplyCurrentLimit" not in text or "tc.CurrentLimits.StatorCurrentLimit" not in text:
        return

    supply_match = re.search(r"tc\.CurrentLimits\.SupplyCurrentLimit\s*=\s*([0-9.]+)\s*;", text)
    stator_match = re.search(r"tc\.CurrentLimits\.StatorCurrentLimit\s*=\s*([0-9.]+)\s*;", text)
    if not supply_match or not stator_match:
        return

    get_motor_block = re.search(
        r"public static MotorConfig getMotorConfig\(\)\s*\{.*?return MotorConfig\.builder\(\)(.*?)\.build\(\);",
        text,
        re.S,
    )
    if not get_motor_block:
        return

    block = get_motor_block.group(1)
    name = parse_builder_value(block, "NAME")
    motor_id = parse_builder_value(block, "ID")
    foc = parse_builder_value(block, "FOC")

    if name is None or motor_id is None:
        return

    name = name.strip('"')
    if foc is None:
        foc = str(default_foc).lower()
    else:
        foc = foc.lower()

    supply = supply_match.group(1)
    stator = stator_match.group(1)

    rows.append((name, int(motor_id), supply, stator, foc, rel))

    follower_block = re.search(
        r"public static MotorConfig getFollowerConfig\(\)\s*\{(.*?)\}",
        text,
        re.S,
    )
    if follower_block:
        fblock = follower_block.group(1)
        follower_name = parse_builder_value(fblock, "NAME")
        follower_id = parse_builder_value(fblock, "ID")
        follower_foc = parse_builder_value(fblock, "FOC")
        if follower_name and follower_id:
            follower_name = follower_name.strip('"')
            if follower_foc is None:
                follower_foc = foc
            rows.append((follower_name, int(follower_id), supply, stator, follower_foc.lower(), rel))


for file_path in sorted(superstructure_dir.glob("*.java")):
    parse_motor_config_from_file(file_path)

# Parse swerve drive/steer initial current limits + module IDs.
tuner_text = tuner_constants_path.read_text()

drive_cfg = re.search(
    r"driveInitialConfigs\s*=.*?withStatorCurrentLimit\(([^)]+)\).*?withSupplyCurrentLimit\(([^)]+)\)",
    tuner_text,
    re.S,
)
steer_cfg = re.search(
    r"steerInitialConfigs\s*=.*?withStatorCurrentLimit\(([^)]+)\).*?withSupplyCurrentLimit\(([^)]+)\)",
    tuner_text,
    re.S,
)

drive_stator, drive_supply = ("?", "?")
steer_stator, steer_supply = ("?", "?")
if drive_cfg:
    drive_stator = drive_cfg.group(1).strip()
    drive_supply = drive_cfg.group(2).strip()
if steer_cfg:
    steer_stator = steer_cfg.group(1).strip()
    steer_supply = steer_cfg.group(2).strip()

for module in ("FrontLeft", "FrontRight", "BackLeft", "BackRight"):
    drive_id_m = re.search(rf"k{module}DriveMotorId\s*=\s*([0-9]+)\s*;", tuner_text)
    steer_id_m = re.search(rf"k{module}SteerMotorId\s*=\s*([0-9]+)\s*;", tuner_text)
    if drive_id_m:
        rows.append(
            (
                f"Swerve {module} Drive",
                int(drive_id_m.group(1)),
                drive_supply,
                drive_stator,
                "n/a",
                tuner_constants_path.relative_to(repo_root).as_posix(),
            )
        )
    if steer_id_m:
        rows.append(
            (
                f"Swerve {module} Steer",
                int(steer_id_m.group(1)),
                steer_supply,
                steer_stator,
                "n/a",
                tuner_constants_path.relative_to(repo_root).as_posix(),
            )
        )

rows.sort(key=lambda r: (r[0].lower(), r[1]))

print("Motor current limits and FOC configuration")
print("=========================================")
print()
print(f"{'MOTOR':<34} {'SUPPLY(A)':<10} {'STATOR(A)':<10} {'FOC':<8} SOURCE FILE")
print(f"{'-----':<34} {'---------':<10} {'---------':<10} {'---':<8} -----------")
for name, motor_id, supply, stator, foc, src in rows:
    motor_label = f"{name} (ID {motor_id})"
    print(f"{motor_label:<34} {supply:<10} {stator:<10} {foc:<8} {src}")

print()
print("Notes:")
print(f" - MotorConfig default FOC: {str(default_foc).lower()} ({motor_config_path.relative_to(repo_root).as_posix()})")
print(" - Swerve FOC is reported as n/a by this script because it is not configured via MotorConfig.")
PY
