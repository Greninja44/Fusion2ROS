#!/usr/bin/env python3
"""Standalone CLI: generate a ROS 2 package from a RobotModel JSON file,
with no Fusion 360 process involved at all.

This is the practical payoff of robot_model/serialization.py: anyone who
can produce a Robot -- by hand, from a script, from a future non-Fusion
CAD importer, or by exporting one from Fusion for later reuse -- can run
the exact same generation pipeline Fusion2ROS's Fusion add-in uses.

Usage:
    python3 scripts/generate_from_json.py robot.json --output-dir output/ \\
        --ros2-control --gazebo --moveit --nav2

Write a RobotModel JSON file with:
    from robot_model import save_robot_json
    save_robot_json(my_robot, "robot.json")
(or see examples/sample_arm.py / examples/sample_rover.py, then
`save_robot_json(build_sample_arm(), "robot.json")`, to produce one to try this on.)

Limitation: this CLI never has a live Fusion session to export mesh files
from, so it always passes an empty mesh_files dict -- if the loaded Robot's
links already reference mesh geometry (kind="mesh", from a prior Fusion
export saved into the JSON), those package://... URDF references are
preserved as-is, but the actual .stl files are NOT copied into the
generated package's meshes/ directory. Copy them in by hand afterward, or
use primitive (box/cylinder/sphere) geometry for a JSON-only robot.
"""

import argparse
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from robot_model import ValidationError, load_robot_json
from fusion_addin.app import PipelineError, generate_ros_package


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("robot_json", type=Path, help="Path to a RobotModel JSON file (see robot_model.save_robot_json)")
    parser.add_argument("--output-dir", type=Path, default=REPO_ROOT / "output", help="Directory to write the package into (default: ./output)")
    parser.add_argument("--ros2-control", action="store_true", help="Include ros2_control config")
    parser.add_argument("--gazebo", action="store_true", help="Include Gazebo Sim config")
    parser.add_argument("--moveit", action="store_true", help="Include MoveIt 2 config")
    parser.add_argument("--nav2", action="store_true", help="Include Nav2 config")
    parser.add_argument("--moveit-group", default="arm", help="MoveIt planning group name (default: arm)")
    args = parser.parse_args(argv)

    if not args.robot_json.is_file():
        print(f"error: {args.robot_json} does not exist", file=sys.stderr)
        return 1

    try:
        robot = load_robot_json(args.robot_json)
    except (ValueError, TypeError) as exc:
        print(f"error: {args.robot_json} is not a valid RobotModel JSON file:\n  {exc}", file=sys.stderr)
        return 1

    try:
        robot.validate()
    except ValidationError as exc:
        print(f"error: {robot.name!r} failed validation:", file=sys.stderr)
        for problem in exc.problems:
            print(f"  - {problem}", file=sys.stderr)
        return 1

    print(f"Loaded {robot.name!r}: {len(robot.links)} links, {len(robot.joints)} joints -- valid")

    try:
        package_dir = generate_ros_package(
            robot,
            mesh_files={},
            output_dir=args.output_dir,
            include_ros2_control=args.ros2_control,
            include_gazebo=args.gazebo,
            include_moveit=args.moveit,
            include_nav2=args.nav2,
            moveit_group_name=args.moveit_group,
        )
    except PipelineError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    print(f"Generated ROS 2 package at: {package_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
