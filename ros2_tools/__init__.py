"""ros2_tools — WSL-side, pure-Linux ROS 2 tooling for Fusion2ROS.

Never imported by fusion_addin/ (the Windows side). Submodules of this
package must stay importable with only the Python standard library —
no rclpy import at module scope, no third-party dependencies — so that
`ros2_tools.validate` in particular can run on a plain Linux box with no
ROS installation at all. Anything that genuinely needs a live ROS 2
environment (e.g. shelling out to `check_urdf`) must detect its absence
via `shutil.which` and degrade gracefully rather than erroring at import
or call time.
"""
