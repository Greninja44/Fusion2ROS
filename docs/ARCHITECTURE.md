# Fusion2ROS — Architecture

## Environment (as investigated 2026-08-25)

| Layer | Finding |
|---|---|
| OS | WSL2, Ubuntu 26.04 "resolute", kernel 6.6.87.2-microsoft-standard-WSL2 |
| Windows user | `iamdu` |
| ROS 2 distro | **lyrical**, installed at `/opt/ros/lyrical` |
| Build tool | `colcon` present (`/usr/bin/colcon`) |
| Gazebo | Present via `ros-lyrical-ros-gz` + `gz-sim` 10.4.0 vendor packages. Binary: `gz` |
| MoveIt 2 | **Not installed.** Needed only for post-MVP MoveIt generator work. |
| Fusion 360 | Installed natively on Windows for user `iamdu`. Add-ins load from `C:\Users\iamdu\AppData\Roaming\Autodesk\FusionAddins` (empty at start of this project). |
| Windows↔WSL | Standard WSL2 interop. WSL sees Windows at `/mnt/c/...`. Windows sees WSL at `\\wsl.localhost\Ubuntu-26.04\...`. Distro name: `Ubuntu-26.04`. |
| Existing ROS workspace | `~/ros2_ws` already exists and builds an unrelated project (`field_rover`, an ag-rover sim — see project memory). This is a real, working colcon workspace we can use as the bridge's build target for early testing. |
| Repo location decision | Repo lives in WSL home (`~/Fusion2ROS`) for fast native git/python/colcon. A Windows-side directory symlink connects Fusion's add-in loader to `fusion_addin/` over `\\wsl.localhost\...`. See "Windows/WSL wiring" below. |

No prior Fusion2ROS code existed anywhere on the system before this session.

## Two-side architecture

```
Fusion2ROS/
├── robot_model/        # Canonical RobotModel schema + validation. Pure stdlib
│                        # Python, zero external deps. Importable unmodified from
│                        # BOTH Fusion's embedded interpreter (Windows) and WSL.
│                        # This is the one package both sides share directly.
│
├── fusion_addin/        # WINDOWS SIDE — runs inside Fusion 360's Python.
│   ├── extraction/       # Fusion API calls: components, joints, mass props, mesh export
│   ├── generators/       # RobotModel -> URDF/Xacro, ROS 2 package tree, meshes
│   ├── ui/                # Fusion command/palette UI (thin — calls into extraction/generators only)
│   └── Fusion2ROS.py      # add-in entry point (Fusion looks for <FolderName>/<FolderName>.py)
│
├── ros2_tools/           # WSL SIDE — pure Linux/ROS 2. Never imported by fusion_addin.
│   ├── validate/          # URDF/package/tree validation (works even without a live ROS env)
│   └── build/             # colcon build wrappers, launch helpers
│
├── bridge/               # Talks across the Windows/WSL boundary.
│   ├── windows/            # runs on Windows side: invokes wsl.exe, detects WSL, copies output/
│   └── wsl_side/            # optional WSL-side helper invoked by the Windows bridge
│
├── output/                # Generated ROS 2 packages land here before being copied into
│                            # a real colcon workspace (e.g. ~/ros2_ws/src/<robot>/)
│
├── docs/
└── tests/
    └── robot_model/        # RobotModel tests — must run with plain `python3`, no Fusion, no ROS
```

**Hard rule enforced by this layout:** nothing under `fusion_addin/` imports anything under `ros2_tools/` or `bridge/wsl_side/`, and vice versa. The only shared code is `robot_model/`, and it must stay dependency-free (Python 3 stdlib only) so it can run unmodified inside Fusion's embedded interpreter, inside WSL, and under plain `pytest` in CI with none of the above installed.

## Windows/WSL wiring

Fusion 360 loads add-ins by scanning `FusionAddins` for folders containing a `<FolderName>.py` matching the folder name. To develop the add-in as version-controlled code in WSL while Fusion (a native Windows process) loads it, we link rather than copy:

```
(Windows, one-time setup, run from an elevated or Developer-Mode-enabled prompt)
mklink /D "C:\Users\iamdu\AppData\Roaming\Autodesk\FusionAddins\Fusion2ROS" ^
           "\\wsl.localhost\Ubuntu-26.04\home\batman\Fusion2ROS\fusion_addin"
```

Editing files in `~/Fusion2ROS/fusion_addin/` from WSL (or any editor) is then immediately visible to Fusion. This requires either an elevated prompt or Windows Developer Mode enabled (for unprivileged `mklink /D`).

**Tried for real on this machine (SHINCHAN) and it didn't work.** `dir`/a direct UNC path/`net use`+drive-letter mapping to `\\wsl.localhost\Ubuntu-26.04\...` all failed ("path invalid" / "network name no longer available") from a genuine Windows cmd.exe process reached via WSL interop, even though `wsl.exe -l -v` correctly reports `Ubuntu-26.04` as the running default distro and basic process interop (launching cmd.exe/wsl.exe from WSL) works fine. Root cause not identified — the P9 network redirector backing `\\wsl.localhost` simply isn't answering in this session. **Fallback in use: `bridge/windows/sync_addin.py`**, a copy-based mirror (`python3 -m bridge.windows.sync_addin`, optionally `--watch`) — not live, must be re-run after editing `fusion_addin/`. If `\\wsl.localhost` starts working on a given machine (e.g. after a WSL/Windows update), try the symlink first; it's strictly better (no separate sync step).

## Bridge workflow (per project spec)

```
Fusion: [Generate ROS 2 Package]
   → Fusion2ROS/output/<robot_name>/            (written by fusion_addin/generators, Windows side)
   → bridge copies to ~/ros2_ws/src/<robot_name>/  (via wsl.exe from the Windows bridge module)
   → wsl.exe bash -lc "cd ~/ros2_ws && colcon build --packages-select <robot_name>"
   → result (BUILD SUCCESS / BUILD FAILED + captured stderr) reported back to Fusion UI
```

The Windows add-in never requires WSL to be present to extract a model, build RobotModel, or generate URDF/package files — only the final "Build in WSL" / "Launch RViz" actions need it. Bridge detects WSL availability (`wsl.exe --status` / `where wsl`) up front and disables those actions gracefully if absent.

## RobotModel

See `robot_model/schema.py` for the authoritative definition. Summary: `Robot { name, links[], joints[], sensors[], actuators[], metadata }`, `Link`, `Joint`, `Sensor`, `Actuator` — all SI units (meters, kilograms, radians, seconds) enforced at construction. Fusion's internal units (cm by default) must be converted at the extraction boundary, never inside RobotModel or downstream generators.

## First vertical-slice milestone

1. `robot_model/` — schema + validation + round-trip tests (no Fusion, no ROS). **Done in this session.**
2. `fusion_addin/extraction/` — walk a Fusion assembly's root occurrence, its rigid/revolute/prismatic joints, and physical properties → build a `Robot`. Needs a real Fusion session to test; build against the documented Fusion API only, no invented calls.
3. `fusion_addin/generators/mesh.py` — export visual/collision meshes (STL) per body/link.
4. `fusion_addin/generators/urdf.py` — `Robot` → URDF/Xacro text. Pure function of RobotModel; testable with a hand-built `Robot` fixture, no Fusion needed.
5. `fusion_addin/generators/package.py` — `Robot` + URDF + meshes → full ROS 2 package tree (`package.xml`, `CMakeLists.txt`, `urdf/`, `meshes/`, `launch/`, `rviz/`) under `output/<robot_name>/`.
6. `bridge/` — copy `output/<robot_name>/` into `~/ros2_ws/src/<robot_name>/`, shell out to `colcon build` via `wsl.exe`, capture pass/fail.
7. `ros2_tools/validate/` — URDF well-formedness + structural checks (no cycles, connected tree, no dangling mesh refs), runnable standalone in WSL against a generated package.
8. Manual milestone verification: a simple hand-authored `Robot` fixture (no Fusion needed yet) flows through steps 4–6 and appears in RViz via `ros2 launch <robot> display.launch.py`.

Steps 1, 4, 5, 7 have zero Fusion/ROS runtime dependency and are safe to build and test immediately in this WSL session. Steps 2, 3, 6 need the real Fusion process or a live WSL ROS env to fully verify and should be handed to focused agents/sessions once the schema (step 1) is stable, since they all consume it.

## Status (this session)

All of steps 1–7 are implemented and merged to `main` (103 tests, `python3 -m pytest tests/`). Step 8 was run for real, end to end, against this machine's actual `~/ros2_ws`:

```
examples/sample_arm.py (hand-authored Robot, no Fusion --
  see ARCHITECTURE.md step 8's own rationale for why this
  is the right way to verify the non-Fusion half)
   -> generate_urdf_xacro -> generate_package
   -> ros2_tools.validate (URDF + package structure): clean
   -> copy_package_to_workspace(~/ros2_ws/src/sample_arm)
   -> colcon build --packages-select sample_arm: BUILD SUCCESS
```
Run it yourself: `python3 scripts/run_vertical_slice.py`.

Live ROS verification beyond the build: `xacro`-processed the generated URDF and ran it through `check_urdf` (clean, correct `base_link -> upper_arm -> forearm` tree printed); launched real `robot_state_publisher` against it — `/robot_description`, `/tf`, `/tf_static`, `/joint_states` all appear on the live graph as expected, with one benign, well-known warning (`kdl_parser`: "root link has an inertia specified... KDL does not support a root link with an inertia" — a common, harmless KDL quirk, not a defect in the generated URDF).

**Two real environment gaps found, not code defects:**
- Neither `joint_state_publisher` nor `joint_state_publisher_gui` is installed on this ROS "lyrical" install, so the generated `display.launch.py` (which references `joint_state_publisher_gui`) fails to launch as-is here. `sudo apt install ros-lyrical-joint-state-publisher-gui` would fix it — not run automatically since installing system packages wasn't asked for.
- A live RViz2 pixel screenshot could not be captured in this session: WSLg's Wayland compositor doesn't support `wlr-screencopy` (breaks `grim`), and `scrot`/`xdotool` against the XWayland surface consistently produced all-black captures of RViz's GL-rendered window (tried both hardware and `LIBGL_ALWAYS_SOFTWARE=1`) — a known category of issue capturing GL front-buffers through XWayland, not something wrong with the render itself (RViz initialized cleanly at OpenGL 4.5, no render errors). `check_urdf`'s parsed tree + the live `/tf`/`/robot_description` graph are the verification that stood in for a pixel screenshot this session.

**Fusion-side pieces (`fusion_addin/extraction/fusion_adapter.py`, `fusion_addin/generators/mesh.py`, `fusion_addin/Fusion2ROS.py`, `fusion_addin/ui/command.py`) are written against documented Fusion API behavior but UNVERIFIED against a live Fusion process** — this sandbox has no `adsk.core`/`adsk.fusion`. They're synced to this machine's `FusionAddins/Fusion2ROS/` folder (via `bridge/windows/sync_addin.py`, see above) and ready to load in a real Fusion session; that load, plus running "Generate ROS 2 Package" against a real assembly, is the one remaining step that needs a human at the Fusion 360 UI.

One integration finding worth knowing before that test: `FusionDesignReaderAdapter` always returns `None` for a joint's `velocity_limit`/`effort_limit` (Fusion's CAD `Joint` object has no motor/velocity concept), so **every Fusion-extracted revolute/prismatic joint will fail generation** with a clear `PipelineError` (`fusion_addin/app.py`'s `check_missing_actuator_limits`) until something sets those limits — there's no `Actuator`-to-`Joint`-limit wiring yet. That's the natural next piece of work, not a bug to "fix" by inventing values.

## Status update: ros2_control / Gazebo / MoveIt 2 / Nav2 (later session)

Four more generators were added the same way as steps 4–5 (parallel agents, each producing plain XML/YAML/launch text with zero Fusion/ROS import-time dependency): `fusion_addin/generators/ros2_control.py`, `gazebo.py`, `moveit.py`, `nav2.py`. `fusion_addin/app.py`'s `generate_ros_package` gained four opt-in flags (`include_ros2_control`, `include_gazebo`, `include_moveit`, `include_nav2`) that splice each generator's `<ros2_control>`/`<gazebo>` XML into the URDF and drop its YAML/launch files into the package via `package.py`'s new `extra_files` mechanism. `robot.metadata["drivetrain"]` (`{"type": "differential_drive", "left_wheel_joint", "right_wheel_joint", "wheel_separation", "wheel_radius"}`) is the shared convention `ros2_control.py` and `nav2.py` both key off of to distinguish an arm from a mobile base — `moveit.py` refuses a drivetrain robot, `nav2.py` requires one.

The user asked to install `ros2_control`/`gz_ros2_control`/Nav2/MoveIt 2 for real verification (none were installed when the generators were authored — same "unverified" status as `fusion_adapter.py`). Once installed, running `python3 scripts/run_vertical_slice.py` (now generates and colcon-builds **two** robots — `examples/sample_arm.py` with `include_ros2_control + include_moveit`, `examples/sample_rover.py` — a hand-authored differential-drive rover — with `include_ros2_control + include_gazebo + include_nav2`) plus manual `ros2 launch`/`ros2 run` against both surfaced **five real bugs, all found and fixed by actually running the generated output, not by inspection**:

1. **ros2_control spawner needed `--param-file`.** This ros2_control version (6.9.0) doesn't propagate a controller's parameters from `ros2_control_node`'s own startup params — each `spawner` invocation needs `--param-file` directly, or the controller fails to initialize ("Length of parameter 'joints' is '0'"). Fixed in `generate_control_launch`.
2. **`joint_trajectory_controller`'s `state_interfaces` param rejects `effort`** (only accepts `{position, velocity, acceleration}` — a real, load-bearing constraint, not documented anywhere obvious upfront). `generate_controllers_yaml` was reusing the Actuator-command-interface validation set, which does allow effort, for this unrelated parameter. Fixed with a dedicated `_JTC_STATE_INTERFACES`.
3. **`ros2_control_node` subscribes to `/robot_description` as a topic**, not just a parameter — the generated `control.launch.py` never started `robot_state_publisher` and hung forever ("Waiting for data on 'robot_description' topic"). Fixed by adding a `robot_state_publisher` node to that launch file.
4. **`package.py`'s `CMakeLists.txt` install list was hardcoded** (`urdf meshes launch rviz config`) — `gazebo.py`'s generated `worlds/empty.sdf` sat in the package but was never installed, so `gz sim` reported "Unable to find or download file". This was the integrator's (not any generator agent's) bug: `extra_files` can introduce new top-level directories `package.py` didn't anticipate. Fixed by computing `install_dirs` from what's actually on disk after all files are written, not a fixed list.
5. **MoveIt's `generate_moveit_demo_launch` assumed a separate `<robot>_moveit_config` package** (the real `moveit_setup_assistant` convention), but this pipeline writes everything into one combined package — `move_group` couldn't find `sample_arm_moveit_config`. Fixed with a `moveit_config_package` override parameter, defaulting to the original real-world behavior; `app.py` passes `robot.name`.
6. **`move_group` cannot start at all with no planning pipeline registered** — throws and terminates immediately ("Planning plugin name is empty..."). The generator's own docstring had understated this as a "simplification" (missing tuned OMPL parameters); it's actually a hard requirement. Added `generate_ompl_planning_yaml()` — static, robot-independent, matching this machine's real installed `moveit_configs_utils/default_configs/ompl_planning.yaml` (deliberately omitting the ~130-line generic `planner_configs` presets file, confirmed not required for `move_group` to start or plan with the default planner).

**What's now real-verified, live, against actual installed binaries:**
- `ros2_control`: `ros2 launch sample_arm control.launch.py` → both `joint_state_broadcaster` and `joint_trajectory_controller` reach `active` state (`ros2 control list_controllers` confirms), hardware (`mock_components/GenericSystem`) loads with the exact joint limits from the generated `<ros2_control>` XML.
- `MoveIt 2`: `ros2 launch sample_arm moveit_demo.launch.py` → `move_group` prints its own readiness banner, **"You can start planning now!"**, with `MoveGroup context using pipeline ompl` confirming the fix; `/moveit_simple_controller_manager` bridges to ros2_control correctly.
- `Nav2`: no `nav2-bringup` package exists on this machine's apt mirror at all (only the individual servers do — `nav2_amcl`, `nav2_controller`, `nav2_planner`, `nav2_bt_navigator`, `nav2_lifecycle_manager`, etc., all installed and used for this check), so the generated `nav2_bringup.launch.py`'s `IncludeLaunchDescription` of `nav2_bringup`'s own launch file can't be exercised end-to-end here — a real environment gap, not a code defect. Instead: `ros2 run nav2_planner planner_server` and `ros2 run nav2_controller controller_server`, each pointed at the generated `nav2_params.yaml`, both start their lifecycle nodes and costmaps cleanly with zero parameter errors — real confirmation the params file itself (RegulatedPurePursuitController + NavfnPlanner plugin selection, footprint radius, costmap config) is structurally correct and loadable by real Nav2 binaries.
- `Gazebo`: **partially verified; one confirmed-external, upstream bug, root-caused precisely, not fixable from generator code.** World loading and entity spawning work correctly end-to-end (`ros2 launch sample_rover gazebo.launch.py` → gz-sim starts, "Entity creation successful"). A follow-up debugging session reproduced the failure headlessly (`gz sim -s -r` + a manual `robot_state_publisher` + `ros_gz_sim create`, avoiding the GUI entirely) and ran the real `gz-sim-main` binary directly under `gdb` (attaching post-hoc is blocked by this machine's `ptrace_scope=1`, so `gdb` must launch the process itself) to get a live, non-truncated backtrace. Finding: it is a genuine **SIGSEGV**, not a hang — it happens synchronously on gz-sim's own main simulation thread, inside `gz_ros2_control::GazeboSimROS2ControlPlugin::Configure()`, immediately after that function logs "Loading controller_manager" (`SimulationRunner::Step → UpdateSystems → SdfEntityCreator::CreateEntities → LoadModelPlugins → SystemManager::LoadPlugin → AddSystemImpl → Configure → crash`, with the faulting frame itself unsymbolized because `libgz_ros2_control-system.so` ships stripped). Two concrete hypotheses from the generator side were checked and ruled out with hard evidence rather than assumed away: (1) the generated `<parameters>$(find sample_rover)/config/controllers.yaml</parameters>` value — running the real `xacro` CLI on the generated URDF (as `robot_state_publisher` does at launch) confirms `xacro`'s generic per-text-node `$(...)` substitution (via `ament_index_python.packages.get_package_share_directory`, not a xacro-specific hack) resolves this to a real, existing, correctly-populated absolute path (`.../install/sample_rover/share/sample_rover/config/controllers.yaml`) well before `gz_ros2_control`'s C++ ever sees it — this is not the bug; (2) the plugin failing to load at all (`SystemLoader.cc: Could not find shared library`) — this only happens if `GZ_SIM_SYSTEM_PLUGIN_PATH` isn't seeded from `LD_LIBRARY_PATH`, which is exactly what `ros_gz_sim`'s own `gz_sim.launch.py` does (confirmed by reading it) — with that env var set to match, the plugin loads and crashes deterministically instead, so this isn't it either. This is an exact match for a bug already filed upstream: [`ros-controls/gz_ros2_control#848`](https://github.com/ros-controls/gz_ros2_control/issues/848) ("Segfault in `gz_ros2_control::GazeboSimROS2ControlPlugin::Configure()`"), filed 2026-05-12 by a `ros2_control` maintainer against **ROS Rolling's own scheduled CI** hitting the identical crash (identical function, identical "Loading controller_manager" → truncated stack-trace shape, same `gz-sim` 10.x lineage — their trace shows `libgz-sim.so.10.1.1`, this machine has `gz-sim` 10.4.0) — i.e. this is not specific to this project's generated files, this machine's packaging, or even the older `resolute`/"lyrical" distro; the bleeding-edge upstream hits the same crash. That issue was auto-closed 2026-08-11 as stale/"not_planned" — it was never actually fixed, and `gz_ros2_control_plugin.cpp`'s commit history since has no related change. There is also no alternate installable version to try: `apt-cache policy`/`apt list -a ros-lyrical-gz-ros2-control` show only the one installed candidate, `3.0.8-1resolute.20260812.033512`. Separately: with the GUI removed from the repro, CPU stays flat/idle after the crash — the previously-observed 450%+ CPU spike is specific to the separate `gz-sim` **GUI client** process (almost certainly spinning/retrying against a server left wedged by the crashed main thread), not the crash itself. Conclusion: a real, externally-confirmed upstream `gz_ros2_control`/`gz-sim` 10.x incompatibility, out of this project's control; nothing in `fusion_addin/generators/gazebo.py` was changed as a result.

Full test suite: 183 passed, 1 skipped (`tests/generators/test_ros2_control.py`'s deepest integration test skips because `ros2_control_node`/`spawner` aren't plain PATH binaries by ROS convention — confirmed legitimate, not a bug, via `ros2 pkg executables controller_manager`).
