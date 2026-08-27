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
- `worlds/empty.sdf`, `launch/gazebo.launch.py`, `config/ros_gz_bridge.yaml`,
  and a spliced `<gazebo>` block — with `include_gazebo=True`. A
  `"differential_drive"` robot gets gz-sim's **native** `DiffDrive` plugin
  (no `ros2_control` dependency, sidestepping a confirmed upstream
  `gz_ros2_control` SIGSEGV — see `docs/ARCHITECTURE.md`'s "Gazebo"
  section); any robot with a non-fixed joint gets a native
  `JointStatePublisher` plugin so `/joint_states` reflects real simulated
  motion. `launch/gazebo.launch.py` auto-starts `ros_gz_bridge`'s
  `parameter_bridge` whenever there's a bridge config to read.
- `config/<robot>.srdf`, `kinematics.yaml`, `joint_limits.yaml`,
  `moveit_controllers.yaml`, `ompl_planning.yaml`, and
  `launch/moveit_demo.launch.py` — with `include_moveit=True`. Supports a
  single planning-group chain, or an automatic arm+gripper two-group split
  for one branch point
- `config/nav2_params.yaml`, `map.yaml`, and `launch/nav2_bringup.launch.py`
  — with `include_nav2=True` (requires `robot.metadata["drivetrain"]`)
- `launch/bringup.launch.py` — with `include_nav2=True` or
  `include_moveit=True`: since neither Nav2 nor MoveIt's own launch file
  starts `robot_state_publisher`, this combines exactly one base
  (Gazebo > `ros2_control` > `display`, by priority) with whichever of
  Nav2/MoveIt were requested, so there's one launch file that actually works

Every optional output is opt-in and independently toggleable, matching the
Fusion UI's output checkboxes. Robot/package names are sanitized into valid
ROS 2 identifiers automatically (`fusion_addin/app.py`'s
`_sanitize_ros_package_name`) — Fusion's own default component name,
"Main Assembly", is not a legal package name on its own.

## Fusion UI automation

The "Generate ROS 2 Package" dialog does more than gather inputs for a
one-shot export:

- **Drivetrain auto-detection** (`extraction/drivetrain_detect.py`) — guesses
  a `"differential_drive"` `robot.metadata["drivetrain"]` from wheel joint
  names (containing "wheel") and forward-kinematics geometry, pre-filling
  the left/right joint + wheel separation/radius fields instead of
  requiring them typed in by hand. Conservative by design: returns no guess
  at all (leaving the fields for manual entry) whenever candidates are
  ambiguous, rather than pairing wheels that don't belong together.
- **Sensor UI** — 3 slots (type + parent link + optional name/update rate)
  to attach cameras/lidar/IMUs to links, wired straight into
  `generators/sensors.py`'s existing Gazebo XML + `ros_gz_bridge` generation.
- **Smart checkbox defaults** — selecting a drivetrain nudges "Gazebo" and
  "Nav2" checked; a suitable single-chain arm nudges "MoveIt 2" checked.
  Always one-directional (never un-checks anything the user already chose).
- **Chained validation** — every successful generation is immediately
  checked with the same structural/URDF validation "Validate ROS 2 Package"
  runs, before any WSL build step.
- **One-click Generate → Build → Launch** — two opt-in fields chain straight
  into `bridge/windows/invoke.py`'s `build_package_in_wsl`/
  `launch_ros2_in_wsl`, gated by `bridge/windows/doctor.py`'s pre-flight
  environment checks (WSL/distro reachable, ROS setup exists, colcon can
  actually build a real throwaway package — also exposed standalone as the
  "Check WSL Environment" command) so a broken environment produces one
  clear report instead of a mid-build failure.
- **Persisted dialog state** (`ui/state.py`) — every checkbox, drivetrain
  field, and sensor slot is saved to `~/Fusion2ROS/.generate_dialog_state.json`
  keyed by robot name, and restored the next time the dialog opens for that
  robot — even after a full Fusion restart.

## Layout

- `robot_model/` — canonical RobotModel schema + JSON serialization. Pure stdlib Python, shared unmodified by both sides.
- `fusion_addin/` — Windows side, runs inside Fusion 360's Python.
  - `extraction/` — Fusion API → RobotModel (via a Fusion-symbol-free abstraction, so it's testable without Fusion), plus `drivetrain_detect.py`'s geometry-only auto-detection
  - `generators/` — RobotModel → URDF/ros2_control/Gazebo/MoveIt/Nav2/sensors/bringup text, each a pure function
  - `app.py` — orchestration layer tying generators together; the one thing both `ui/` and the standalone CLI call into
  - `ui/` — Fusion command handlers: Generate (with the automation above), Validate, Build in WSL, Launch RViz, Check WSL Environment
- `ros2_tools/` — WSL side, pure Linux/ROS 2 (URDF/package structural validation).
- `bridge/` — Windows↔WSL glue: `windows/invoke.py` copies a generated package into a colcon workspace and builds/launches it remotely via `wsl.exe`; `windows/doctor.py` runs pre-flight environment checks; `windows/sync_addin.py` syncs the add-in onto the Windows FusionAddins folder.
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
