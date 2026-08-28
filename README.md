# Fusion2ROS

[![CI](https://github.com/Greninja44/Fusion2ROS/actions/workflows/ci.yml/badge.svg)](https://github.com/Greninja44/Fusion2ROS/actions/workflows/ci.yml)

Fusion2ROS turns a Fusion 360 robot assembly into a ready-to-build ROS 2
package: URDF/Xacro, meshes, `ros2_control`, Gazebo Sim, MoveIt 2, and Nav2 —
all generated from one shared `RobotModel`, not five separate pipelines. A
Fusion 360 command dialog drives the whole thing, including auto-detecting
your drivetrain, building the package inside WSL, and launching it, without
leaving Fusion.

## Why

Hand-writing a URDF, a `ros2_control` config, a Gazebo world, a MoveIt
config, and a Nav2 params file for the same robot — and keeping all five in
sync every time the CAD changes — is the tedious part of getting a robot
into simulation. Fusion2ROS extracts link/joint geometry, mass properties,
and mesh geometry directly from the CAD assembly you already built, so the
robot description stays a byproduct of the CAD, not a hand-maintained
duplicate of it.

## How it works

```
Fusion 360 assembly
      │  (fusion_addin/extraction — Fusion API calls only)
      ▼
  RobotModel                    ← robot_model/, pure-stdlib, shared schema
      │  (fusion_addin/generators — pure functions, no Fusion/ROS imports)
      ▼
ROS 2 package (output/<robot_name>/)
      │  (bridge/windows — shells out to wsl.exe)
      ▼
colcon build + ros2 launch, inside WSL
```

- **`robot_model/`** is the one package both sides import unmodified: a
  small, dependency-free schema (`Robot`, `Link`, `Joint`, `Sensor`,
  `Actuator`, all SI units) plus JSON (de)serialization. It runs equally
  inside Fusion's embedded Python, inside WSL, and under plain `pytest`.
- **`fusion_addin/`** is the Windows side, running inside Fusion 360's
  Python: `extraction/` walks the live assembly into a `RobotModel`;
  `generators/` turn a `RobotModel` into URDF/YAML/launch text (pure
  functions — testable with zero Fusion or ROS installed); `ui/` wires
  both into Fusion's command dialogs.
- **`ros2_tools/`** is the WSL/Linux side: structural and URDF validation
  that needs no live ROS environment.
- **`bridge/`** crosses the Windows↔WSL boundary — copying a generated
  package into a real colcon workspace and driving `colcon build` /
  `ros2 launch` remotely via `wsl.exe`.

See `docs/ARCHITECTURE.md` for the full design rationale and its "Status"
sections for exactly what's been verified against real, live ROS 2 / Gazebo
/ MoveIt binaries versus what's written but not yet run against a live
Fusion 360 process.

## Getting started

### 1. Install the add-in into Fusion 360

Fusion loads add-ins from `FusionAddins`, scanning for a folder containing a
`<FolderName>.py` matching the folder name. Two ways to get `fusion_addin/`
there:

- **Preferred, if `\\wsl.localhost\<distro>\...` is reachable from Windows
  on your machine:** make `FusionAddins/Fusion2ROS` a directory symlink into
  this repo's `fusion_addin/` folder over the WSL UNC path — edits made in
  WSL are then visible to Fusion instantly, no separate sync step.
- **Fallback (always works):** `python3 -m bridge.windows.sync_addin`
  (add `--watch` to keep mirroring on every save) copies `fusion_addin/`,
  plus the three repo-root packages it imports by absolute name
  (`robot_model/`, `bridge/windows/`, `ros2_tools/validate/`), into
  `FusionAddins/Fusion2ROS`. Re-run it after every edit unless you use
  `--watch`.

Then in Fusion: **Tools → Add-Ins → Scripts and Add-Ins → Add-Ins tab →
Fusion2ROS → Run**. Five new commands appear on the Solid tab's Create
panel: **Generate ROS 2 Package**, **Validate ROS 2 Package**,
**Build in WSL**, **Launch RViz**, **Check WSL Environment**.

### 2. Run the environment check

Before generating anything, run **Check WSL Environment** — it verifies WSL
and your distro are reachable, your ROS setup script exists, `colcon` is on
`PATH`, and (the check that actually matters) `colcon` can build a real
throwaway package end to end, not just that `catkin_pkg` imports. One clear
pass/fail report up front beats a build failing halfway through later.

### 3. Generate a package

Open your robot assembly, run **Generate ROS 2 Package**, pick which
outputs you want (`ros2_control`, Gazebo, MoveIt 2, Nav2, sensors — see
[What it generates](#what-it-generates)), and hit OK. The dialog
auto-detects a differential-drive chassis from wheel joint geometry,
nudges sensible checkboxes on, validates the result immediately, and can
optionally build it in WSL and launch it — all described in
[Fusion UI automation](#fusion-ui-automation) below.

### 4. Or skip Fusion entirely

Nothing above requires a live Fusion process to try the pipeline itself —
see [Generating a package without Fusion](#generating-a-package-without-fusion).

## What it generates

Given a `robot_model.Robot` — extracted from a live Fusion 360 assembly, or
hand-authored/loaded from JSON (see `robot_model/serialization.py` and
`scripts/generate_from_json.py`) — `fusion_addin/app.py`'s
`generate_ros_package` produces a single `ament_cmake` ROS 2 package:

| Output | Flag | Notes |
|---|---|---|
| `urdf/<robot>.urdf.xacro`, `meshes/`, `launch/display.launch.py`, `rviz/<robot>.rviz` | always | |
| `config/controllers.yaml`, `launch/control.launch.py`, spliced `<ros2_control>` | `include_ros2_control` | arm/manipulator (`joint_trajectory_controller`) or mobile base (`diff_drive_controller` / `mecanum_drive_controller`), from `robot.metadata["drivetrain"]` |
| `worlds/empty.sdf`, `launch/gazebo.launch.py`, `config/ros_gz_bridge.yaml`, spliced `<gazebo>` | `include_gazebo` | differential-drive robots get gz-sim's **native** `DiffDrive` plugin (no `ros2_control` dependency — sidesteps a confirmed upstream `gz_ros2_control` SIGSEGV, see `docs/ARCHITECTURE.md`); any robot with a non-fixed joint gets a native `JointStatePublisher` plugin; `ros_gz_bridge`'s `parameter_bridge` is auto-started whenever there's a bridge config to read |
| `config/<robot>.srdf`, `kinematics.yaml`, `joint_limits.yaml`, `moveit_controllers.yaml`, `ompl_planning.yaml`, `launch/moveit_demo.launch.py` | `include_moveit` | single planning-group chain, or an automatic arm+gripper split at one branch point |
| `config/nav2_params.yaml`, `map.yaml`, `launch/nav2_bringup.launch.py` | `include_nav2` | requires `robot.metadata["drivetrain"]` |
| `launch/bringup.launch.py` | `include_nav2` or `include_moveit` | neither Nav2 nor MoveIt's own launch file starts `robot_state_publisher`, so this combines exactly one base (Gazebo > `ros2_control` > `display`, by priority) with whichever stacks were requested, into one launch file that actually works |

Every optional output is independently toggleable, matching the Fusion UI's
checkboxes. Robot/package names are sanitized into valid ROS 2 identifiers
automatically (`_sanitize_ros_package_name` in `fusion_addin/app.py`) —
Fusion's own default component name, "Main Assembly", isn't a legal package
name as-is.

## Fusion UI automation

The **Generate ROS 2 Package** dialog does more than collect checkboxes for
a one-shot export:

- **Drivetrain auto-detection** (`extraction/drivetrain_detect.py`) —
  guesses a `"differential_drive"` `robot.metadata["drivetrain"]` from wheel
  joint names and forward-kinematics geometry, pre-filling the left/right
  joint and wheel separation/radius fields. Conservative by design: it
  returns no guess at all — leaving the fields for manual entry — whenever
  the candidates are ambiguous, rather than risk pairing wheels that don't
  belong together.
- **Sensor UI** — three slots (type + parent link + optional name/update
  rate) to attach cameras, depth cameras, lidar, IMUs, or GPS/NavSat units
  to links, feeding straight into `generators/sensors.py`'s existing Gazebo
  XML and `ros_gz_bridge` generation. (Force/torque sensors are
  generator-level only — they're joint-mounted, not link-mounted, so they
  don't fit this slot shape; attach one by hand-editing the generated
  `Robot`/URDF, see `generators/sensors.py`.)
- **Smart checkbox defaults** — picking a drivetrain nudges "Gazebo" and
  "Nav2" on; a suitable single-chain arm nudges "MoveIt 2" on. Always
  one-directional — it never unchecks something you already chose.
- **Chained validation** — every successful generation is immediately
  checked with the same structural/URDF validation "Validate ROS 2 Package"
  runs on its own, before any WSL build step.
- **One-click Generate → Build → Launch** — two opt-in fields chain
  straight into `bridge/windows/invoke.py`'s `build_package_in_wsl` /
  `launch_ros2_in_wsl`, gated by `bridge/windows/doctor.py`'s pre-flight
  checks (the same ones "Check WSL Environment" runs standalone), so a
  broken environment produces one clear report instead of a build that
  fails halfway through.
- **Persisted dialog state** (`ui/state.py`) — every checkbox, drivetrain
  field, and sensor slot is saved to
  `~/Fusion2ROS/.generate_dialog_state.json` keyed by robot name, and
  restored the next time the dialog opens for that robot, even after a full
  Fusion restart.

## Generating a package without Fusion

```
python3 -c "
from robot_model import save_robot_json
from examples.sample_arm import build_sample_arm
save_robot_json(build_sample_arm(), 'robot.json')
"
python3 scripts/generate_from_json.py robot.json --output-dir output/ --ros2-control --moveit
```

`examples/sample_arm.py` and `examples/sample_rover.py` are hand-authored
`RobotModel`s used for exactly this — testing and demos with no Fusion
process involved.

## Running the full demo (generate → colcon build, against a real ROS 2 workspace)

```
python3 scripts/run_vertical_slice.py
```

Generates and `colcon build`s both example robots (arm with
`ros2_control` + MoveIt, rover with `ros2_control` + Gazebo + Nav2) against
a real colcon workspace end to end.

## Testing in WSL

1. `python3 -m pytest tests/ -v` — the full suite, no Fusion, no ROS
   required (integration tests that need `colcon`/ROS installed self-skip
   gracefully if they're absent).
2. Point `bridge.windows.invoke.copy_package_to_workspace` (or just `cp -r`)
   at a generated `output/<robot_name>/` into your `~/ros2_ws/src/`, then
   `colcon build --packages-select <robot_name>` and source the workspace.
3. `ros2 launch <robot_name> display.launch.py` for a static RViz check, or
   `gazebo.launch.py` / `control.launch.py` / `moveit_demo.launch.py` /
   `nav2_bringup.launch.py` / `bringup.launch.py` for whichever stacks you
   generated.

The **Check WSL Environment** command (or `bridge/windows/doctor.py`
directly) runs the same checks non-interactively if you want a quick
pass/fail without opening Fusion at all.

## Layout

- `robot_model/` — canonical `RobotModel` schema + JSON serialization. Pure
  stdlib Python, shared unmodified by both sides.
- `fusion_addin/` — Windows side, runs inside Fusion 360's Python.
  - `extraction/` — Fusion API → `RobotModel`, via a Fusion-symbol-free
    abstraction so it's testable without Fusion, plus
    `drivetrain_detect.py`'s geometry-only auto-detection.
  - `generators/` — `RobotModel` → URDF / `ros2_control` / Gazebo / MoveIt /
    Nav2 / sensors / bringup text, each a pure function.
  - `app.py` — orchestration tying generators together; the one thing both
    `ui/` and the standalone CLI call into.
  - `ui/` — Fusion command handlers: Generate (with the automation above),
    Validate, Build in WSL, Launch RViz, Check WSL Environment.
- `ros2_tools/` — WSL side, pure Linux/ROS 2 (URDF/package structural
  validation).
- `bridge/` — Windows↔WSL glue: `windows/invoke.py` copies a generated
  package into a colcon workspace and builds/launches it remotely via
  `wsl.exe`; `windows/doctor.py` runs the pre-flight environment checks;
  `windows/sync_addin.py` mirrors the add-in onto the Windows
  `FusionAddins` folder.
- `examples/` — hand-authored sample robots (`sample_arm.py`,
  `sample_rover.py`) used for testing and demos without Fusion.
- `scripts/` — standalone tools: `run_vertical_slice.py` (full
  generate→build demo), `generate_from_json.py` (Fusion-free CLI).
- `output/` — generated ROS 2 packages land here.
- `tests/` — runs with plain `python3 -m pytest`, no Fusion, no ROS (a few
  integration tests self-skip gracefully when ROS/colcon aren't installed).
- `docs/ARCHITECTURE.md` — design rationale and a running log of what's
  been verified against real Fusion/ROS/Gazebo/MoveIt binaries.

## Status

Core generators (URDF, `ros2_control`, Gazebo, MoveIt 2, Nav2) and the
extraction/validation pipeline have all been exercised against real,
installed ROS 2 binaries — `colcon build`, `ros2 launch`, `move_group`,
`check_urdf`, and a live headless `gz sim` run — not just unit tests. Full
details, including exactly which pieces are still unverified against a live
Fusion 360 process, are in `docs/ARCHITECTURE.md`'s "Status" sections.

### Known limitations

- **`gz_ros2_control` SIGSEGV on non-differential-drive robots.** A robot
  with `metadata["drivetrain"]["type"] == "differential_drive"` gets
  gz-sim's native `DiffDrive` plugin, which is real-verified crash-free.
  Anything else — an arm, `"mecanum_drive"`, or no drivetrain at all — still
  falls back to `gz_ros2_control`, which has a confirmed upstream SIGSEGV
  on some gz-sim versions ([ros-controls/gz_ros2_control#848](https://github.com/ros-controls/gz_ros2_control/issues/848),
  filed against ROS Rolling's own CI, auto-closed stale/unfixed). This is
  an external bug, not something fixable from this repo. The Fusion UI
  warns about it in the generation report when it applies, rather than
  letting you discover it via a crash.
- **Nav2's `nav2_bringup` package itself was never exercised end-to-end**
  on the machine this was built on (only individually-installed Nav2
  servers were) — see `docs/ARCHITECTURE.md`'s Nav2 status note.
- **Install is a symlink or a copy-script**, not a packaged Fusion 360
  add-in you install from the Autodesk App Store — see
  [Getting started](#getting-started).
