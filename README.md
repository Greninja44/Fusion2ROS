# Fusion2ROS

Converts a Fusion 360 robot CAD assembly into a complete ROS 2 package
(URDF/Xacro, meshes, launch files, and eventually ros2_control/Gazebo/MoveIt 2
scaffolding).

See `docs/ARCHITECTURE.md` for the full design: the two-side (Windows Fusion
add-in / WSL ROS 2 tooling) split, the `robot_model/` canonical schema both
sides share, and the vertical-slice milestone plan.

## Layout

- `robot_model/` — canonical RobotModel schema. Pure stdlib Python, shared unmodified by both sides.
- `fusion_addin/` — Windows side, runs inside Fusion 360's Python.
- `ros2_tools/` — WSL side, pure Linux/ROS 2.
- `bridge/` — Windows↔WSL glue (copy generated package, invoke colcon via wsl.exe).
- `output/` — generated ROS 2 packages land here.
- `tests/` — must run with plain `python3 -m pytest`, no Fusion, no ROS.

## Running tests

```
python3 -m pytest tests/ -v
```
