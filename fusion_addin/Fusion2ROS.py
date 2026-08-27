"""Fusion 360 add-in entry point. Fusion loads this by convention: the file
must be named <FolderName>.py, matching the FusionAddins subfolder it lives
in (Fusion2ROS.manifest sits alongside it for the same reason).

REAL BUG FOUND AND FIXED HERE (confirmed live, via a real Fusion 360
session -- the first genuine runtime error this project has hit):
build_command.py/launch_command.py do `from bridge.windows.detect import
...` / `from bridge.windows.invoke import ...` -- an ABSOLUTE import,
assuming `bridge` is importable as a top-level package. It never was: the
deployed add-in folder (FusionAddins/Fusion2ROS/, populated by
bridge/windows/sync_addin.py from this repo's fusion_addin/ directory) has
no idea the sibling `bridge/` directory back in the repo even exists --
nothing put the repo root on Fusion's Python's sys.path. The failure mode
was exactly as ugly as that sounds: an import-time ModuleNotFoundError
raised while Fusion's loader executes this very file, which happens BEFORE
run()'s own try/except below ever gets a chance to run -- so instead of a
clean "Fusion2ROS failed to start" messageBox, Fusion's own outer handling
kicked in, silently disabling the add-in.

Fix: sync_addin.py now ALSO mirrors bridge/windows/ (the only bridge
subpackage build_command.py/launch_command.py actually need -- see
bridge/windows/__init__.py's own docstring: it never imports
bridge/wsl_side/) into a `bridge/windows/` subfolder placed right next to
this file, so the deployed add-in folder is fully self-contained. The
sys.path insertion immediately below makes that self-contained `bridge/`
importable as an absolute package from here, before anything that needs it
is imported.

*** UNVERIFIED against a live Fusion process BEYOND THIS FIX *** --
run(context)/stop(context) is the standard, universally-documented Fusion
add-in lifecycle contract. Deliberately minimal: it does nothing but call
each ui/*_command.py sibling module's register()/unregister() -- see those
files' docstrings for why the actual logic isn't here. All four (Generate,
Validate, Build in WSL, Launch RViz) share the same register(ui)/
unregister(ui) shape and the same panel (SolidCreatePanel), so they appear
together.
"""

import sys
import traceback
from pathlib import Path

# Must happen before importing .ui.{build_command,launch_command}, which
# import the co-deployed bridge/windows/ package by absolute name -- see
# this module's docstring for the real bug this fixes.
sys.path.insert(0, str(Path(__file__).resolve().parent))

import adsk.core

from .ui import build_command, command, doctor_command, launch_command, validate_command

_MODULES = (command, validate_command, build_command, launch_command, doctor_command)


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        for module in _MODULES:
            module.register(ui)
    except Exception:
        if ui:
            ui.messageBox(f"Fusion2ROS failed to start:\n{traceback.format_exc()}")


def stop(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        for module in _MODULES:
            module.unregister(ui)
    except Exception:
        if ui:
            ui.messageBox(f"Fusion2ROS failed to stop cleanly:\n{traceback.format_exc()}")
