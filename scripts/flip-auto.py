"""
Notes:
* Must run the script from the root of the repository
* Auto align and other commands must be manually flipped (e.g. AutoAlignLeft -> AutoAlignRight)
* Example usage: python3 scripts/flip-auto.py 'Left Coral x3' 'Left Coral x3.auto'
"""

import json
import os
import glob
import sys

PATHS_DIR = "src/main/deploy/pathplanner/paths"
AUTOS_DIR = "src/main/deploy/pathplanner/autos"
SETTINGS_PATH = "src/main/deploy/pathplanner/settings.json"
FIELD_Y = 8.0519016


def load_json(path):
    with open(path) as f:
        return json.load(f)


def save_json(data, path):
    with open(path, "w") as f:
        json.dump(data, f, indent=2)


def flip_name(name):
    if "left" in name.lower():
        return name.replace("Left", "Right").replace("left", "right")
    elif "right" in name.lower():
        return name.replace("Right", "Left").replace("right", "left")
    else:
        return f"{name}Flipped"


def flip_y_coord(y, center_y=FIELD_Y / 2):
    return center_y + (center_y - y)


def flip_path_file(input_file, output_file):
    data = load_json(input_file)

    for wp in data["waypoints"]:
        wp["anchor"]["y"] = flip_y_coord(wp["anchor"]["y"])
        for ctrl in ["prevControl", "nextControl"]:
            if wp[ctrl]:
                wp[ctrl]["y"] = flip_y_coord(wp[ctrl]["y"])

    for target in data["rotationTargets"]:
        target["rotationDegrees"] *= -1

    for state in ["goalEndState", "idealStartingState"]:
        data[state]["rotation"] *= -1

    data["folder"] = flip_name(data["folder"])

    save_json(data, output_file)


def flip_matching_paths(target_folder):
    for path_file in glob.glob(os.path.join(PATHS_DIR, "*.path")):
        data = load_json(path_file)
        if data.get("folder") == target_folder:
            name, ext = os.path.splitext(os.path.basename(path_file))
            flipped_name = flip_name(name)
            output_file = os.path.join(PATHS_DIR, f"{flipped_name}{ext}")
            flip_path_file(path_file, output_file)
            print(f"Flipped {name}{ext} -> {flipped_name}{ext}")


def flip_auto_file(target_auto):
    data = load_json(os.path.join(AUTOS_DIR, target_auto))

    for cmd in data["command"]["data"]["commands"]:
        if cmd.get("type") == "path":
            cmd["data"]["pathName"] = flip_name(cmd["data"]["pathName"])

    output_name = flip_name(os.path.splitext(target_auto)[0]) + ".auto"
    save_json(data, os.path.join(AUTOS_DIR, output_name))
    print(f"Flipped auto {target_auto} -> {output_name}")


def update_settings_file(target_folder):
    settings = load_json(SETTINGS_PATH)

    flipped_folder = flip_name(target_folder)
    if flipped_folder not in settings["pathFolders"]:
        settings["pathFolders"].append(flipped_folder)
        save_json(settings, SETTINGS_PATH)
        print(f"Added '{flipped_folder}' to pathFolders")


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python flip.py <TARGET_FOLDER> <TARGET_AUTO>")
        sys.exit(1)

    target_folder = sys.argv[1]
    target_auto = sys.argv[2]

    update_settings_file(target_folder)
    flip_matching_paths(target_folder)
    flip_auto_file(target_auto)
