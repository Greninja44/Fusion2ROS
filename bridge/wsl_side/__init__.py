"""WSL-side half of the bridge.

Runs as a normal Linux process inside WSL. Fully implemented and covered by
real (non-mocked) tests -- see tests/bridge/test_wsl_side.py.
"""

from .build import BuildResult, colcon_build, copy_package_to_workspace

__all__ = ["BuildResult", "colcon_build", "copy_package_to_workspace"]
