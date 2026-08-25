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
