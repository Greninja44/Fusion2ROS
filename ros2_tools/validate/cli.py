"""Command-line entry point for ros2_tools.validate.

Usage:
    python3 -m ros2_tools.validate <path>

If <path> is a directory it is validated as a ROS 2 package
(validate_package_structure); if it's a file it is validated as a URDF
(validate_urdf_file). Each problem is printed on its own line. Exit code is
1 if any problems were found, 0 if the target is clean.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import List, Optional

from .package import validate_package_structure
from .urdf import validate_urdf_file


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        prog="python3 -m ros2_tools.validate",
        description=(
            "Validate a URDF file or a ROS 2 package directory tree, "
            "without needing a live ROS 2 environment."
        ),
    )
    parser.add_argument(
        "path",
        type=Path,
        help="URDF file or ROS 2 package directory to validate",
    )
    args = parser.parse_args(argv)

    path: Path = args.path

    if not path.exists():
        print(f"error: path does not exist: '{path}'", file=sys.stderr)
        return 1

    if path.is_dir():
        problems = validate_package_structure(path)
    else:
        problems = validate_urdf_file(path)

    for problem in problems:
        print(problem)

    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
