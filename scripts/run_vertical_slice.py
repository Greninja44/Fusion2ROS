#!/usr/bin/env python3
"""Runs the full Fusion2ROS vertical slice for real, against the real
~/ros2_ws, using a hand-authored Robot (examples/sample_arm.py) in place of
a live Fusion extraction (no Fusion process available in this environment --
see ARCHITECTURE.md's milestone step 8, which calls exactly this out as the
right way to verify the non-Fusion half of the pipeline).

    sample_arm.py (Robot)
        -> generate_urdf_xacro           (fusion_addin/generators/urdf.py)
        -> generate_package              (fusion_addin/generators/package.py)
        -> validate_package_structure    (ros2_tools/validate/package.py)
        -> copy_package_to_workspace     (bridge/wsl_side/build.py)
        -> colcon_build --packages-select (bridge/wsl_side/build.py)

Never touches any package under ~/ros2_ws other than the one this script
generates (--packages-select is always used, matching every agent's
instructions in this project).
"""

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from examples.sample_arm import build_sample_arm
from fusion_addin.generators.package import generate_package
from fusion_addin.generators.urdf import generate_urdf_xacro
from ros2_tools.validate.package import validate_package_structure
from ros2_tools.validate.urdf import validate_urdf_file
from bridge.wsl_side.build import colcon_build, copy_package_to_workspace


def main() -> int:
    output_dir = REPO_ROOT / "output"
    ros_ws = Path.home() / "ros2_ws"
    ros_ws_src = ros_ws / "src"

    print("== 1. Build RobotModel ==")
    robot = build_sample_arm()
    print(f"   {robot.name}: {len(robot.links)} links, {len(robot.joints)} joints -- validate() OK")

    print("== 2. Generate URDF/Xacro ==")
    urdf_xacro = generate_urdf_xacro(robot)
    print(f"   {len(urdf_xacro)} bytes of URDF/Xacro generated")

    print("== 3. Generate ROS 2 package ==")
    package_dir = generate_package(robot, urdf_xacro, mesh_files={}, output_dir=output_dir)
    print(f"   package written to {package_dir}")

    print("== 4. Validate generated URDF ==")
    urdf_path = package_dir / "urdf" / f"{robot.name}.urdf.xacro"
    urdf_problems = validate_urdf_file(urdf_path)
    if urdf_problems:
        print("   PROBLEMS:")
        for p in urdf_problems:
            print(f"     - {p}")
        return 1
    print("   clean")

    print("== 5. Validate generated package structure ==")
    pkg_problems = validate_package_structure(package_dir)
    if pkg_problems:
        print("   PROBLEMS:")
        for p in pkg_problems:
            print(f"     - {p}")
        return 1
    print("   clean")

    print(f"== 6. Copy package into {ros_ws_src} ==")
    dest = copy_package_to_workspace(package_dir, ros_ws_src)
    print(f"   copied to {dest}")

    print(f"== 7. colcon build --packages-select {robot.name} ==")
    result = colcon_build(ros_ws, robot.name)
    print(result.stdout)
    if not result.success:
        print("BUILD FAILED")
        print(result.stderr)
        return 1
    print("BUILD SUCCESS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
