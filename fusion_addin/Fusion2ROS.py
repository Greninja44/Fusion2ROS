"""Fusion 360 add-in entry point. Fusion loads this by convention: the file
must be named <FolderName>.py, matching the FusionAddins subfolder it lives
in (Fusion2ROS.manifest sits alongside it for the same reason).

*** UNVERIFIED against a live Fusion process *** -- run(context)/stop(context)
is the standard, universally-documented Fusion add-in lifecycle contract, but
this exact file has never been loaded by a real Fusion session (this sandbox
has no adsk.core). Deliberately minimal: it does nothing but call each
ui/*_command.py sibling module's register()/unregister() -- see those files'
docstrings for why the actual logic isn't here. All four (Generate, Validate,
Build in WSL, Launch RViz) share the same register(ui)/unregister(ui) shape
and the same panel (SolidCreatePanel), so they appear together.
"""

import traceback

import adsk.core

from .ui import build_command, command, launch_command, validate_command

_MODULES = (command, validate_command, build_command, launch_command)


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
