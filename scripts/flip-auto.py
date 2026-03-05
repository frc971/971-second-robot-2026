"""
Notes:
* Auto align and other commands must be manually flipped (e.g. AutoAlignLeft -> AutoAlignRight)
* Example usage: python3 scripts/flip-auto.py 'Left Coral x3' 'Left Coral x3.auto'
"""

import json
import os
import glob
import sys

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
PATHS_DIR = os.path.join(REPO_ROOT, "src", "main", "deploy", "pathplanner", "paths")
AUTOS_DIR = os.path.join(REPO_ROOT, "src", "main", "deploy", "pathplanner", "autos")
SETTINGS_PATH = os.path.join(REPO_ROOT, "src", "main", "deploy", "pathplanner", "settings.json")
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


def recursively_flip_path_data(node):
    if isinstance(node, dict):
        # Mirror any field coordinate object against field centerline.
        if (
            "x" in node
            and "y" in node
            and isinstance(node["x"], (int, float))
            and isinstance(node["y"], (int, float))
        ):
            node["y"] = flip_y_coord(node["y"])

        # Mirror heading-style angles that are defined in field reference frame.
        for angle_key in ("rotation", "rotationDegrees", "rotationOffset"):
            if angle_key in node and isinstance(node[angle_key], (int, float)):
                node[angle_key] *= -1

        for value in node.values():
            recursively_flip_path_data(value)
    elif isinstance(node, list):
        for item in node:
            recursively_flip_path_data(item)


def flip_path_file(input_file, output_file):
    data = load_json(input_file)

    recursively_flip_path_data(data)

    # Mirrored paths should not retain linked waypoint groups from the source side.
    # Keeping any link names can still cause unexpected cross-path snapping in PathPlanner.
    for wp in data.get("waypoints", []):
        if "linkedName" in wp:
            wp["linkedName"] = None

    folder = data.get("folder")
    if isinstance(folder, str) and folder:
        data["folder"] = flip_name(folder)

    save_json(data, output_file)


def flip_matching_paths(target_folder):
    flipped_any = False
    for path_file in glob.glob(os.path.join(PATHS_DIR, "*.path")):
        data = load_json(path_file)
        if data.get("folder") == target_folder:
            name, ext = os.path.splitext(os.path.basename(path_file))
            flipped_name = flip_name(name)
            output_file = os.path.join(PATHS_DIR, f"{flipped_name}{ext}")
            flip_path_file(path_file, output_file)
            print(f"Flipped {name}{ext} -> {flipped_name}{ext}")
            flipped_any = True
    return flipped_any


def collect_path_names(node, found):
    if isinstance(node, dict):
        path_name = node.get("pathName")
        if isinstance(path_name, str):
            found.add(path_name)
        for value in node.values():
            collect_path_names(value, found)
    elif isinstance(node, list):
        for item in node:
            collect_path_names(item, found)


def flip_paths_by_name(path_names):
    for path_name in sorted(path_names):
        input_file = os.path.join(PATHS_DIR, f"{path_name}.path")
        if not os.path.exists(input_file):
            print(f"WARNING: Path file not found for referenced path '{path_name}'")
            continue
        output_name = flip_name(path_name)
        output_file = os.path.join(PATHS_DIR, f"{output_name}.path")
        flip_path_file(input_file, output_file)
        print(f"Flipped {path_name}.path -> {output_name}.path")


def flip_auto_file(target_auto):
    auto_name = target_auto if target_auto.endswith(".auto") else f"{target_auto}.auto"
    data = load_json(os.path.join(AUTOS_DIR, auto_name))
    path_names = set()
    collect_path_names(data.get("command"), path_names)

    def recursively_flip_auto_commands(node):
        if isinstance(node, dict):
            if "pathName" in node and isinstance(node["pathName"], str):
                node["pathName"] = flip_name(node["pathName"])
            for value in node.values():
                recursively_flip_auto_commands(value)
        elif isinstance(node, list):
            for item in node:
                recursively_flip_auto_commands(item)

    recursively_flip_auto_commands(data.get("command"))

    folder = data.get("folder")
    if isinstance(folder, str) and folder:
        data["folder"] = flip_name(folder)

    output_name = flip_name(os.path.splitext(auto_name)[0]) + ".auto"
    save_json(data, os.path.join(AUTOS_DIR, output_name))
    print(f"Flipped auto {auto_name} -> {output_name}")
    return path_names


def update_settings_file(target_folder):
    settings = load_json(SETTINGS_PATH)

    flipped_folder = flip_name(target_folder)
    updated = False

    for list_key in ("pathFolders", "autoFolders"):
        if list_key in settings and isinstance(settings[list_key], list):
            if flipped_folder not in settings[list_key]:
                settings[list_key].append(flipped_folder)
                print(f"Added '{flipped_folder}' to {list_key}")
                updated = True

    if updated:
        save_json(settings, SETTINGS_PATH)


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python flip.py <TARGET_FOLDER> <TARGET_AUTO>")
        sys.exit(1)

    target_folder = sys.argv[1]
    target_auto = sys.argv[2]

    update_settings_file(target_folder)
    flipped_by_folder = flip_matching_paths(target_folder)
    referenced_path_names = flip_auto_file(target_auto)
    if not flipped_by_folder or referenced_path_names:
        flip_paths_by_name(referenced_path_names)
