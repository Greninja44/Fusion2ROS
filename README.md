# Fusion2ROS

[![CI](https://github.com/Greninja44/Fusion2ROS/actions/workflows/ci.yml/badge.svg)](https://github.com/Greninja44/Fusion2ROS/actions/workflows/ci.yml)

Converts a Fusion 360 robot CAD assembly into a complete, ready-to-build ROS 2
package: URDF/Xacro, meshes, launch files, `ros2_control` config, Gazebo Sim
scaffolding, MoveIt 2 motion planning, and Nav2 navigation — generated from
one canonical `RobotModel`, not five separate pipelines.

See `docs/ARCHITECTURE.md` for the full design (the two-side Windows Fusion
add-in / WSL ROS 2 tooling split, the `robot_model/` canonical schema both
sides share) and its "Status" sections for exactly what's been verified
against real, live ROS 2/Gazebo/MoveIt binaries versus what's written but
unverified against an actual Fusion 360 process.

## What it generates

Given a `robot_model.Robot` — extracted from a live Fusion 360 assembly, or
hand-authored/loaded from JSON (see `robot_model/serialization.py` and
`scripts/generate_from_json.py` — no Fusion required for either) —
`fusion_addin/app.py`'s `generate_ros_package` produces a single ROS 2
(`ament_cmake`) package containing:

- `urdf/<robot>.urdf.xacro` — always generated
- `meshes/`, `launch/display.launch.py`, `rviz/<robot>.rviz` — always generated
- `config/controllers.yaml`, `launch/control.launch.py`, and a spliced
  `<ros2_control>` block — with `include_ros2_control=True`. Supports
  arm/manipulator robots (`joint_trajectory_controller`) and mobile bases
  (`diff_drive_controller` or `mecanum_drive_controller`, selected via
  `robot.metadata["drivetrain"]`)
- `worlds/empty.sdf`, `launch/gazebo.launch.py`, and a spliced `<gazebo>`
  block (materials + the `gz_ros2_control` plugin) — with `include_gazebo=True`
- `config/<robot>.srdf`, `kinematics.yaml`, `joint_limits.yaml`,
  `moveit_controllers.yaml`, `ompl_planning.yaml`, and
  `launch/moveit_demo.launch.py` — with `include_moveit=True`. Supports a
  single planning-group chain, or an automatic arm+gripper two-group split
  for one branch point
- `config/nav2_params.yaml`, `map.yaml`, and `launch/nav2_bringup.launch.py`
  — with `include_nav2=True` (requires `robot.metadata["drivetrain"]`)

Every optional output is opt-in and independently toggleable, matching the
planned Fusion UI's output checkboxes.

## Layout

- `robot_model/` — canonical RobotModel schema + JSON serialization. Pure stdlib Python, shared unmodified by both sides.
- `fusion_addin/` — Windows side, runs inside Fusion 360's Python.
  - `extraction/` — Fusion API → RobotModel (via a Fusion-symbol-free abstraction, so it's testable without Fusion)
  - `generators/` — RobotModel → URDF/ros2_control/Gazebo/MoveIt/Nav2/sensors text, each a pure function
  - `app.py` — orchestration layer tying generators together; the one thing both `ui/` and the standalone CLI call into
  - `ui/` — thin Fusion command handlers (Generate, Validate, Build in WSL, Launch RViz)
- `ros2_tools/` — WSL side, pure Linux/ROS 2 (URDF/package structural validation).
- `bridge/` — Windows↔WSL glue: copy a generated package into a colcon workspace and build/launch it remotely via `wsl.exe`, plus `sync_addin.py` for syncing the add-in onto the Windows FusionAddins folder.
- `examples/` — hand-authored sample robots (`sample_arm.py`, `sample_rover.py`) used for testing and demos without Fusion.
- `scripts/` — standalone tools: `run_vertical_slice.py` (full generate→build demo), `generate_from_json.py` (Fusion-free CLI).
- `output/` — generated ROS 2 packages land here.
- `tests/` — must run with plain `python3 -m pytest`, no Fusion, no ROS (a few integration tests self-skip gracefully when ROS/colcon aren't installed).

## Running tests

```
python3 -m pytest tests/ -v
```

## Generating a package without Fusion

```
python3 -c "
from robot_model import save_robot_json
from examples.sample_arm import build_sample_arm
save_robot_json(build_sample_arm(), 'robot.json')
"
python3 scripts/generate_from_json.py robot.json --output-dir output/ --ros2-control --moveit
```

## Running the full demo (generate → colcon build, against a real ROS 2 workspace)

```
python3 scripts/run_vertical_slice.py
```
