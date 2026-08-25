"""Enables `python3 -m ros2_tools.validate <path>`."""

import sys

from .cli import main

if __name__ == "__main__":
    sys.exit(main())
