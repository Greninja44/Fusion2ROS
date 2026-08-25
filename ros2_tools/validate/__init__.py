"""URDF / ROS 2 package structural validation.

Pure Python standard library only (see ros2_tools/__init__.py for why). The
optional `check_urdf` cross-check in `urdf.validate_urdf_file` is the one
place this package talks to an actual ROS 2 install, and it is entirely
opportunistic: if the binary isn't on PATH, that check is silently skipped.
"""

from .urdf import validate_urdf_file
from .package import validate_package_structure

__all__ = ["validate_urdf_file", "validate_package_structure"]
