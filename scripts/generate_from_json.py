#!/usr/bin/env python3
"""Standalone CLI: generate a ROS 2 package from one or more RobotModel JSON
files, with no Fusion 360 process involved at all.

This is the practical payoff of robot_model/serialization.py: anyone who
can produce a Robot -- by hand, from a script, from a future non-Fusion
CAD importer, or by exporting one from Fusion for later reuse -- can run
the exact same generation pipeline Fusion2ROS's Fusion add-in uses.

Usage:
    python3 scripts/generate_from_json.py robot.json --output-dir output/ \\
        --ros2-control --gazebo --moveit --nav2

    # batch mode: generate several robots in one invocation, e.g. a whole
    # fleet or every robot in a directory. Every robot gets the same set of
    # include_* flags; each is processed independently -- one robot's
    # failure is reported and skipped, it does not stop the others.
    python3 scripts/generate_from_json.py robots/*.json --output-dir output/

Write a RobotModel JSON file with:
    from robot_model import save_robot_json
    save_robot_json(my_robot, "robot.json")
(or see examples/sample_arm.py / examples/sample_rover.py, then
`save_robot_json(build_sample_arm(), "robot.json")`, to produce one to try this on.)

Limitation: this CLI never has a live Fusion session to export mesh files
from, so it always passes an empty mesh_files dict -- if a loaded Robot's
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


def generate_one(robot_json: Path, output_dir: Path, include_kwargs: dict) -> bool:
    """Returns True on success. All errors are printed to stderr and
    reported as a False return rather than raised, so a caller processing
    multiple robots (see main()'s batch mode) can continue past one
    robot's failure instead of aborting the whole batch."""
    if not robot_json.is_file():
        print(f"error: {robot_json} does not exist", file=sys.stderr)
        return False

    try:
        robot = load_robot_json(robot_json)
    except (ValueError, TypeError) as exc:
        print(f"error: {robot_json} is not a valid RobotModel JSON file:\n  {exc}", file=sys.stderr)
        return False

    try:
        robot.validate()
    except ValidationError as exc:
        print(f"error: {robot.name!r} (from {robot_json}) failed validation:", file=sys.stderr)
        for problem in exc.problems:
            print(f"  - {problem}", file=sys.stderr)
        return False

    print(f"Loaded {robot.name!r} (from {robot_json}): {len(robot.links)} links, {len(robot.joints)} joints -- valid")

    def _print_progress(stage_description: str, step: int, total: int) -> None:
        print(f"  [{step}/{total}] {stage_description}")

    try:
        package_dir = generate_ros_package(
            robot, mesh_files={}, output_dir=output_dir, progress_callback=_print_progress, **include_kwargs
        )
    except PipelineError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return False

    print(f"Generated ROS 2 package at: {package_dir}")
    return True


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "robot_json",
        type=Path,
        nargs="+",
        help="Path(s) to one or more RobotModel JSON files (see robot_model.save_robot_json). "
        "Pass more than one (or a shell glob) for batch mode.",
    )
    parser.add_argument("--output-dir", type=Path, default=REPO_ROOT / "output", help="Directory to write the package(s) into (default: ./output)")
    parser.add_argument("--ros2-control", action="store_true", help="Include ros2_control config")
    parser.add_argument("--gazebo", action="store_true", help="Include Gazebo Sim config")
    parser.add_argument("--moveit", action="store_true", help="Include MoveIt 2 config")
    parser.add_argument("--nav2", action="store_true", help="Include Nav2 config")
    parser.add_argument("--moveit-group", default="arm", help="MoveIt planning group name (default: arm)")
    args = parser.parse_args(argv)

    include_kwargs = dict(
        include_ros2_control=args.ros2_control,
        include_gazebo=args.gazebo,
        include_moveit=args.moveit,
        include_nav2=args.nav2,
        moveit_group_name=args.moveit_group,
    )

    results = []
    for robot_json in args.robot_json:
        if len(args.robot_json) > 1:
            print(f"\n=== {robot_json} ===")
        results.append(generate_one(robot_json, args.output_dir, include_kwargs))

    if len(args.robot_json) > 1:
        succeeded = sum(results)
        print(f"\n{succeeded}/{len(results)} robots generated successfully")

    return 0 if all(results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
