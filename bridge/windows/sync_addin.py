"""Mirrors fusion_addin/ into Fusion 360's FusionAddins folder by copying.

ARCHITECTURE.md originally proposed a live directory symlink/junction from
FusionAddins into the WSL repo over \\\\wsl.localhost\\..., so edits made in
WSL would be visible to Fusion instantly with no separate sync step. That was
tried for real on this machine (SHINCHAN) and the \\\\wsl.localhost network
share turned out to be unreachable from the Windows side in this session --
`dir`, a direct UNC path, and `net use`/drive-letter mapping all failed
("path invalid" / "network name no longer available"), even though `wsl.exe
-l -v` correctly reports the Ubuntu-26.04 distro as running and default, and
basic Windows<->WSL process interop (launching cmd.exe, wsl.exe from WSL)
works fine. Root cause not identified (the P9 network redirector that backs
\\\\wsl.localhost simply isn't answering here) -- if it starts working on a
given machine (e.g. after a WSL/Windows update), the symlink approach in
ARCHITECTURE.md is still the better, no-sync-step option; try that first.

Until then, this is the fallback: an explicit, idempotent copy-tree sync.
Not live -- you must re-run it (or use --watch) after editing fusion_addin/
for Fusion to see the change. Only touches *.py files it manages; anything
Fusion itself writes into the destination (e.g. its own __pycache__) is left
alone except pycache, which is always pruned since it's derived, not source.

Also deploys three repo-root packages that fusion_addin/ imports by absolute
name (bridge/windows/, robot_model/, ros2_tools/validate/) into the same
destination folder, since none of them are otherwise on Fusion's Python's
sys.path -- see each sync_*_into() function below for which real bug it
fixes (all three were found the same way: a ModuleNotFoundError live in
Fusion, reproduced by simulating Fusion's sys.path against the deployed
copy).
"""

from __future__ import annotations

import argparse
import shutil
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SOURCE = REPO_ROOT / "fusion_addin"
# bridge/windows/ itself is needed inside the deployed add-in folder:
# fusion_addin/ui/build_command.py and launch_command.py do
# `from bridge.windows.detect import ...` / `from bridge.windows.invoke
# import ...` -- an ABSOLUTE import, which only resolves if a `bridge/`
# package sits somewhere on Fusion's Python's sys.path. It never did before
# this was added (a real bug, found live in Fusion: "ModuleNotFoundError:
# No module named 'bridge'", raised from build_command.py at add-in load
# time -- see fusion_addin/Fusion2ROS.py's docstring for the full story and
# the matching sys.path fix on that side). bridge/wsl_side/ is NOT deployed
# here -- bridge/windows/__init__.py's own docstring confirms
# bridge/windows/ never imports it, and it wouldn't work under Windows
# Python anyway (it shells out to `colcon`/`bash`, Linux-only tools).
BRIDGE_WINDOWS_SOURCE = REPO_ROOT / "bridge" / "windows"
BRIDGE_INIT_SOURCE = REPO_ROOT / "bridge" / "__init__.py"
# Same underlying bug, same fix shape: fusion_addin/app.py and every
# fusion_addin/generators/*.py and fusion_addin/extraction/converter.py do
# `from robot_model import ...` -- also an ABSOLUTE import of a repo-root
# package that was never deployed alongside fusion_addin/. Confirmed live
# by re-running the exact sys.path-simulation repro used to find the
# `bridge` bug against the deployed copy AFTER that first fix: it got
# past the bridge import and immediately hit
# "ModuleNotFoundError: No module named 'robot_model'" instead.
ROBOT_MODEL_SOURCE = REPO_ROOT / "robot_model"
# fusion_addin/ui/validate_command.py does `from ros2_tools.validate.package
# import ...` / `from ros2_tools.validate.urdf import ...` -- same bug again.
# validate_command.py's own docstring already flags this as a deliberate,
# one-off exception to the "fusion_addin never imports ros2_tools" layering
# rule specifically because ros2_tools.validate is pure-stdlib (no adsk, no
# live ROS env needed), so it's safe to deploy; only the validate/
# subpackage is needed, not all of ros2_tools (e.g. nothing else under
# ros2_tools is pure-stdlib/Windows-safe).
ROS2_TOOLS_VALIDATE_SOURCE = REPO_ROOT / "ros2_tools" / "validate"
ROS2_TOOLS_INIT_SOURCE = REPO_ROOT / "ros2_tools" / "__init__.py"


def sync_addin_to_fusion(source_dir: Path, dest_dir: Path) -> Path:
    """Copies source_dir's contents into dest_dir, removing files under
    dest_dir that no longer exist in source_dir (a real mirror, not just an
    overlay) so stale/deleted modules don't linger and get imported by
    mistake. dest_dir is created if it doesn't exist. __pycache__ is never
    copied. Returns dest_dir.
    """
    source_dir = Path(source_dir)
    dest_dir = Path(dest_dir)
    if not source_dir.is_dir():
        raise FileNotFoundError(f"source_dir does not exist or is not a directory: {source_dir}")

    dest_dir.mkdir(parents=True, exist_ok=True)

    source_files = {
        p.relative_to(source_dir)
        for p in source_dir.rglob("*")
        if p.is_file() and "__pycache__" not in p.parts
    }
    dest_files = {
        p.relative_to(dest_dir)
        for p in dest_dir.rglob("*")
        if p.is_file() and "__pycache__" not in p.parts
    }

    for stale in dest_files - source_files:
        (dest_dir / stale).unlink()

    for rel in source_files:
        src_path = source_dir / rel
        dst_path = dest_dir / rel
        dst_path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(src_path, dst_path)

    for cache_dir in list(dest_dir.rglob("__pycache__")):
        shutil.rmtree(cache_dir, ignore_errors=True)

    # Prune now-empty directories left behind by removed stale files.
    for d in sorted((p for p in dest_dir.rglob("*") if p.is_dir()), key=lambda p: -len(p.parts)):
        try:
            d.rmdir()
        except OSError:
            pass  # not empty

    return dest_dir


def sync_bridge_windows_into(dest_dir: Path) -> None:
    """Deploys the self-contained `bridge/windows/` subpackage (plus a
    `bridge/__init__.py`) into `dest_dir/bridge/windows/` and
    `dest_dir/bridge/__init__.py`, so the add-in folder can `import
    bridge.windows.detect` / `import bridge.windows.invoke` on its own --
    see this module's BRIDGE_WINDOWS_SOURCE comment for why this exists.
    Reuses sync_addin_to_fusion for the directory-mirror semantics (removes
    stale files, prunes __pycache__) rather than a bespoke copy.
    """
    dest_dir = Path(dest_dir)
    sync_addin_to_fusion(BRIDGE_WINDOWS_SOURCE, dest_dir / "bridge" / "windows")
    (dest_dir / "bridge").mkdir(parents=True, exist_ok=True)
    shutil.copyfile(BRIDGE_INIT_SOURCE, dest_dir / "bridge" / "__init__.py")


def sync_robot_model_into(dest_dir: Path) -> None:
    """Deploys the pure-stdlib `robot_model/` package into
    `dest_dir/robot_model/`, so `app.py` and `generators/*.py`'s absolute
    `from robot_model import ...` resolve -- see ROBOT_MODEL_SOURCE's
    comment for why this exists."""
    sync_addin_to_fusion(ROBOT_MODEL_SOURCE, Path(dest_dir) / "robot_model")


def sync_ros2_tools_validate_into(dest_dir: Path) -> None:
    """Deploys `ros2_tools/validate/` (plus a `ros2_tools/__init__.py`) into
    `dest_dir/ros2_tools/validate/`, so validate_command.py's absolute
    `from ros2_tools.validate.{package,urdf} import ...` resolve -- see
    ROS2_TOOLS_VALIDATE_SOURCE's comment for why only this subpackage is
    deployed rather than all of ros2_tools."""
    dest_dir = Path(dest_dir)
    sync_addin_to_fusion(ROS2_TOOLS_VALIDATE_SOURCE, dest_dir / "ros2_tools" / "validate")
    (dest_dir / "ros2_tools").mkdir(parents=True, exist_ok=True)
    shutil.copyfile(ROS2_TOOLS_INIT_SOURCE, dest_dir / "ros2_tools" / "__init__.py")


def _default_windows_addins_dir() -> Path:
    """Best-effort default: /mnt/c/Users/<user>/AppData/Roaming/Autodesk/FusionAddins.
    Only reliable when run from WSL against a drvfs-mounted C:; pass --dest
    explicitly (e.g. a real Windows path) when running under Windows Python."""
    mnt_c_users = Path("/mnt/c/Users")
    if not mnt_c_users.is_dir():
        raise FileNotFoundError(
            "Could not find /mnt/c/Users -- pass --dest explicitly with the target "
            "FusionAddins directory."
        )
    candidates = [
        d / "AppData" / "Roaming" / "Autodesk" / "FusionAddins"
        for d in mnt_c_users.iterdir()
        if d.is_dir() and (d / "AppData" / "Roaming" / "Autodesk").is_dir()
    ]
    if len(candidates) != 1:
        raise FileNotFoundError(
            f"Expected exactly one Windows user with a FusionAddins-capable profile under "
            f"{mnt_c_users}, found {len(candidates)} -- pass --dest explicitly."
        )
    return candidates[0]


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE, help="fusion_addin/ directory to sync from")
    parser.add_argument(
        "--dest",
        type=Path,
        default=None,
        help="FusionAddins/Fusion2ROS destination directory (default: auto-detect under /mnt/c/Users/<you>/...)",
    )
    parser.add_argument("--watch", action="store_true", help="Re-sync every --interval seconds until Ctrl-C")
    parser.add_argument("--interval", type=float, default=2.0, help="Seconds between syncs in --watch mode")
    args = parser.parse_args(argv)

    dest = args.dest
    if dest is None:
        addins_dir = _default_windows_addins_dir()
        dest = addins_dir / "Fusion2ROS"

    def _sync_all() -> None:
        sync_addin_to_fusion(args.source, dest)
        sync_bridge_windows_into(dest)
        sync_robot_model_into(dest)
        sync_ros2_tools_validate_into(dest)

    _sync_all()
    print(f"Synced {args.source} -> {dest}")
    print(f"Synced {BRIDGE_WINDOWS_SOURCE} -> {dest / 'bridge' / 'windows'}")
    print(f"Synced {ROBOT_MODEL_SOURCE} -> {dest / 'robot_model'}")
    print(f"Synced {ROS2_TOOLS_VALIDATE_SOURCE} -> {dest / 'ros2_tools' / 'validate'}")

    if args.watch:
        print(f"Watching for changes every {args.interval}s (Ctrl-C to stop)...")
        try:
            while True:
                time.sleep(args.interval)
                _sync_all()
        except KeyboardInterrupt:
            print("Stopped.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
