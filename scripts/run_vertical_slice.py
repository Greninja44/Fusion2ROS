#!/usr/bin/env python3
"""Runs the full Fusion2ROS pipeline for real, against the real ~/ros2_ws,
using two hand-authored Robots in place of a live Fusion extraction (no
Fusion process available in this environment -- see ARCHITECTURE.md's
milestone step 8, which calls exactly this out as the right way to verify
the non-Fusion half of the pipeline):

    examples/sample_arm.py (fixed-base 2-joint arm)
        -> generate_ros_package(..., include_ros2_control=True, include_moveit=True)

    examples/sample_rover.py (differential-drive mobile base)
        -> generate_ros_package(..., include_ros2_control=True, include_gazebo=True, include_nav2=True)

Both packages: generate -> validate -> copy into ~/ros2_ws/src -> colcon
build --packages-select. Never touches any other package under ~/ros2_ws.
"""

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from examples.sample_arm import build_sample_arm
from examples.sample_rover import build_sample_rover
from fusion_addin.app import generate_ros_package
from ros2_tools.validate.package import validate_package_structure
from ros2_tools.validate.urdf import validate_urdf_file
from bridge.wsl_side.build import colcon_build, copy_package_to_workspace


def run_one(robot, generate_kwargs, output_dir, ros_ws):
    print(f"\n{'=' * 60}\n{robot.name}\n{'=' * 60}")

    print(f"== Generate ROS 2 package ({', '.join(k for k, v in generate_kwargs.items() if v is True)}) ==")
    package_dir = generate_ros_package(robot, {}, output_dir, **generate_kwargs)
    print(f"   package written to {package_dir}")
    print("   files:", sorted(p.relative_to(package_dir).as_posix() for p in package_dir.rglob("*") if p.is_file()))

    print("== Validate generated URDF ==")
    urdf_path = package_dir / "urdf" / f"{robot.name}.urdf.xacro"
    urdf_problems = validate_urdf_file(urdf_path)
    if urdf_problems:
        print("   PROBLEMS:", urdf_problems)
        return False
    print("   clean")

    print("== Validate generated package structure ==")
    pkg_problems = validate_package_structure(package_dir)
    if pkg_problems:
        print("   PROBLEMS:", pkg_problems)
        return False
    print("   clean")

    ros_ws_src = ros_ws / "src"
    print(f"== Copy package into {ros_ws_src} ==")
    dest = copy_package_to_workspace(package_dir, ros_ws_src)
    print(f"   copied to {dest}")

    print(f"== colcon build --packages-select {robot.name} ==")
    result = colcon_build(ros_ws, robot.name)
    print(result.stdout)
    if not result.success:
        print("BUILD FAILED")
        print(result.stderr)
        return False
    print("BUILD SUCCESS")
    return True


def main() -> int:
    output_dir = REPO_ROOT / "output"
    ros_ws = Path.home() / "ros2_ws"

    arm_ok = run_one(
        build_sample_arm(),
        {"include_ros2_control": True, "include_moveit": True},
        output_dir,
        ros_ws,
    )
    rover_ok = run_one(
        build_sample_rover(),
        {"include_ros2_control": True, "include_gazebo": True, "include_nav2": True},
        output_dir,
        ros_ws,
    )

    return 0 if (arm_ok and rover_ok) else 1


if __name__ == "__main__":
    raise SystemExit(main())
