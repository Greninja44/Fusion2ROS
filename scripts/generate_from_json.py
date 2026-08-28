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

    # See what would be written without writing anything:
    python3 scripts/generate_from_json.py robot.json --dry-run --gazebo

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

Config file: repeated flags (which outputs to include, --output-dir,
maintainer/license metadata) can be set once in a config file instead of
passed on every invocation -- see fusion_addin/config.py's module
docstring for the file format and the CLI-overrides-config precedence
rule. Looked up automatically (./.fusion2ros.yaml, then
~/.fusion2ros.yaml) unless --config or --no-config is given.
"""

import argparse
import sys
from pathlib import Path
from typing import Dict, Optional

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from robot_model import ValidationError, load_robot_json
from fusion_addin.app import PipelineError, generate_ros_package
from fusion_addin.config import ConfigError, Fusion2ROSConfig, resolve_config, resolve_value
from fusion_addin.generators.package import PackageManifest


def _print_manifest(manifest: PackageManifest) -> None:
    print(f"Dry run -- would generate ROS 2 package at: {manifest.pkg_dir}")
    width = max((len(entry.path) for entry in manifest.entries), default=0)
    for entry in manifest.entries:
        print(f"  {entry.path.ljust(width)}   {entry.description}")
    print("No files were written.")


def generate_one(
    robot_json: Path,
    output_dir: Path,
    include_kwargs: dict,
    dry_run: bool = False,
    metadata_overrides: Optional[Dict[str, str]] = None,
) -> bool:
    """Returns True on success. All errors are printed to stderr and
    reported as a False return rather than raised, so a caller processing
    multiple robots (see main()'s batch mode) can continue past one
    robot's failure instead of aborting the whole batch."""
    if not robot_json.is_file():
        print(f"error: {robot_json} does not exist", file=sys.stderr)
        return False

    try:
        robot = load_robot_json(robot_json)
    except (ValueError, TypeError, KeyError) as exc:
        # ValueError/TypeError: e.g. malformed JSON syntax, or a field with
        # the wrong type reaching a dataclass constructor. KeyError: e.g. a
        # joint dict missing its required "type" key (robot_from_dict reads
        # that one via data["type"], not .get) -- without catching it here
        # too, that case fell straight through as a raw, unexplained
        # traceback instead of the same one-line "not a valid ... file"
        # message every other malformed-JSON case already gets.
        detail = f"missing required field {exc}" if isinstance(exc, KeyError) else str(exc)
        print(f"error: {robot_json} is not a valid RobotModel JSON file:\n  {detail}", file=sys.stderr)
        return False

    try:
        robot.validate()
    except ValidationError as exc:
        print(f"error: {robot.name!r} (from {robot_json}) failed validation:", file=sys.stderr)
        for problem in exc.problems:
            print(f"  - {problem}", file=sys.stderr)
        return False

    if metadata_overrides:
        # Only fills gaps: a value already present in the robot's own JSON
        # (the most specific, per-robot-authored source) is left alone --
        # config-file/--maintainer-*/--license values are defaults, not
        # forced overrides, of that.
        if robot.metadata is None:
            robot.metadata = {}
        for key, value in metadata_overrides.items():
            robot.metadata.setdefault(key, value)

    print(f"Loaded {robot.name!r} (from {robot_json}): {len(robot.links)} links, {len(robot.joints)} joints -- valid")

    def _print_progress(stage_description: str, step: int, total: int) -> None:
        print(f"  [{step}/{total}] {stage_description}")

    try:
        result = generate_ros_package(
            robot,
            mesh_files={},
            output_dir=output_dir,
            progress_callback=_print_progress,
            dry_run=dry_run,
            **include_kwargs,
        )
    except PipelineError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return False

    if dry_run:
        _print_manifest(result)
    else:
        print(f"Generated ROS 2 package at: {result}")
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
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Directory to write the package(s) into (default: ./output, or the config file's output_dir)",
    )
    parser.add_argument(
        "--ros2-control",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="Include ros2_control config (default: off, or the config file's include.ros2_control)",
    )
    parser.add_argument(
        "--gazebo",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="Include Gazebo Sim config (default: off, or the config file's include.gazebo)",
    )
    parser.add_argument(
        "--moveit",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="Include MoveIt 2 config (default: off, or the config file's include.moveit)",
    )
    parser.add_argument(
        "--nav2",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="Include Nav2 config (default: off, or the config file's include.nav2)",
    )
    parser.add_argument(
        "--moveit-group",
        default=None,
        help="MoveIt planning group name (default: arm, or the config file's moveit_group)",
    )
    parser.add_argument(
        "--maintainer-name",
        default=None,
        help="package.xml maintainer name, used only where the robot JSON doesn't already set one "
        "(default: the config file's maintainer.name, else 'TODO')",
    )
    parser.add_argument(
        "--maintainer-email",
        default=None,
        help="package.xml maintainer email, same fallback rule as --maintainer-name "
        "(default: the config file's maintainer.email, else 'TODO@TODO.TODO')",
    )
    parser.add_argument(
        "--license",
        dest="license_name",
        default=None,
        help="package.xml license, same fallback rule as --maintainer-name "
        "(default: the config file's license, else 'TODO: License declaration')",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=None,
        help="Path to a Fusion2ROS config file (default: look for ./.fusion2ros.yaml, then "
        "~/.fusion2ros.yaml; see fusion_addin/config.py)",
    )
    parser.add_argument(
        "--no-config",
        action="store_true",
        help="Ignore any config file, even one that would otherwise be found automatically",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Show what would be generated (files + short descriptions) without writing anything",
    )
    args = parser.parse_args(argv)

    try:
        config = Fusion2ROSConfig() if args.no_config else resolve_config(explicit_path=args.config)
    except ConfigError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    if config.source_path is not None:
        print(f"Using config file: {config.source_path}")

    output_dir = resolve_value(
        args.output_dir,
        Path(config.output_dir) if config.output_dir is not None else None,
        REPO_ROOT / "output",
    )
    include_kwargs = dict(
        include_ros2_control=resolve_value(args.ros2_control, config.include_ros2_control, False),
        include_gazebo=resolve_value(args.gazebo, config.include_gazebo, False),
        include_moveit=resolve_value(args.moveit, config.include_moveit, False),
        include_nav2=resolve_value(args.nav2, config.include_nav2, False),
        moveit_group_name=resolve_value(args.moveit_group, config.moveit_group_name, "arm"),
    )

    metadata_overrides = {
        key: value
        for key, value in (
            ("maintainer_name", resolve_value(args.maintainer_name, config.maintainer_name, None)),
            ("maintainer_email", resolve_value(args.maintainer_email, config.maintainer_email, None)),
            ("license", resolve_value(args.license_name, config.license, None)),
        )
        if value is not None
    }

    results = []
    for robot_json in args.robot_json:
        if len(args.robot_json) > 1:
            print(f"\n=== {robot_json} ===")
        results.append(
            generate_one(
                robot_json, output_dir, include_kwargs, dry_run=args.dry_run, metadata_overrides=metadata_overrides
            )
        )

    if len(args.robot_json) > 1:
        succeeded = sum(results)
        verb = "validated (dry run)" if args.dry_run else "generated"
        print(f"\n{succeeded}/{len(results)} robots {verb} successfully")

    return 0 if all(results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
