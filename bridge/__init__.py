"""Windows<->WSL bridge glue.

See docs/ARCHITECTURE.md ("Bridge workflow") for the full picture:

    Fusion: [Generate ROS 2 Package]
       -> Fusion2ROS/output/<robot_name>/            (Windows side)
       -> bridge copies to ~/ros2_ws/src/<robot_name>/  (via wsl.exe, Windows side)
       -> wsl.exe bash -lc "cd ~/ros2_ws && colcon build --packages-select <robot_name>"
       -> result (BUILD SUCCESS / BUILD FAILED + captured stderr) reported to Fusion UI

Sub-packages:
    bridge.wsl_side  -- runs as a normal Linux process inside WSL. Fully
                         testable in this environment.
    bridge.windows   -- runs under Windows Python inside the Fusion 360
                         add-in process, shelling out to wsl.exe. Cannot be
                         run or tested from inside WSL itself; see module
                         docstrings for the "unverified" caveat.

Hard rule (per docs/ARCHITECTURE.md): nothing under fusion_addin/ or
ros2_tools/ may be imported here, and this package must not import them
either.
"""
