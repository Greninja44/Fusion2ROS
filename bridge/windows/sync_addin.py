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

`main()` also validates the destination up front (writable, and the
source has the `<FolderName>.py` entry point Fusion actually requires) so a
doomed sync fails fast with one actionable message instead of a raw
traceback, and -- unless `--skip-env-check` is passed -- runs a fast subset
of `bridge.windows.doctor`'s WSL/ROS 2 environment checks right after a
successful sync and folds the result into this command's own exit code.
That collapses "install the add-in" and "confirm the environment is
healthy" (previously two separate steps: run this script, then remember to
also run "Check WSL Environment" inside Fusion) into the one command a user
actually runs from WSL; the only step that still can't be collapsed away is
enabling the add-in inside Fusion's own UI (Tools -> Add-Ins -> ... -> Run),
since nothing on this side of the boundary can drive that.
"""

from __future__ import annotations

import argparse
import os
import shutil
import sys
import time
from pathlib import Path
from typing import Optional

from .doctor import all_critical_passed, format_report, run_environment_checks
from .invoke import DEFAULT_DISTRO, DEFAULT_ROS_SETUP, DEFAULT_WSL_ROS_WS_SRC

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


def _verify_expected_addin_entry_point(source_dir: Path, dest_dir: Path) -> Optional[str]:
    """Fusion only lists an add-in whose folder contains a `<FolderName>.py`
    matching the folder's own name (see this module's top-of-file
    docstring) -- e.g. `FusionAddins/Fusion2ROS/Fusion2ROS.py`. Returns an
    actionable error string if `source_dir` doesn't have the file
    `dest_dir`'s name requires, else None. This is normally only hit with a
    customized `--dest` whose folder name doesn't match the add-in's own
    entry-point file (the default `--dest` always ends in `Fusion2ROS`,
    matching `fusion_addin/Fusion2ROS.py`, which is always present in a
    checked-out repo) -- but when it does happen, the sync would otherwise
    "succeed" while Fusion silently never shows the add-in at all, which is
    a much worse failure mode than catching it here up front.
    """
    expected = f"{dest_dir.name}.py"
    if not (source_dir / expected).is_file():
        return (
            f"Sync aborted: {source_dir} has no {expected!r}. Fusion 360 only loads an add-in "
            f"whose folder name has a matching <FolderName>.py inside it, so {dest_dir.name!r} "
            f"would never show up in Fusion's Add-Ins list after this sync. If --dest was "
            f"customized, point it at a folder named after the add-in's own entry-point file "
            f"(normally 'Fusion2ROS', matching fusion_addin/Fusion2ROS.py)."
        )
    return None


def _check_dest_writable(dest_dir: Path) -> Optional[str]:
    """Best-effort pre-check that `dest_dir` can actually be written to, so
    a doomed sync fails fast with one clear, actionable message instead of
    a raw PermissionError/OSError traceback partway through copying files.
    Walks up to the nearest existing ancestor of dest_dir (which itself may
    not exist yet) and checks that. Returns an error string if not
    writable, else None.
    """
    probe = dest_dir
    while not probe.exists():
        parent = probe.parent
        if parent == probe:
            break
        probe = parent
    if not os.access(probe, os.W_OK):
        return (
            f"Sync aborted: destination is not writable: {dest_dir} (checked {probe}). Check "
            f"folder permissions, and that it isn't locked by Explorer, Fusion, or a sync tool "
            f"like OneDrive."
        )
    return None


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
    parser.add_argument(
        "--skip-env-check",
        action="store_true",
        help="Don't run the WSL/ROS 2 environment doctor checks after syncing (e.g. for fast --watch loops)",
    )
    parser.add_argument(
        "--full-env-check",
        action="store_true",
        help="Include the slow real `colcon build` probe in the post-sync environment check "
        "(default: a fast subset -- WSL/distro/ROS-setup/colcon-on-PATH/workspace only)",
    )
    parser.add_argument(
        "--check-gazebo", action="store_true", help="Also check that `gz` (Gazebo Sim) is on PATH"
    )
    parser.add_argument("--distro", default=DEFAULT_DISTRO, help="WSL distro name for the environment check")
    parser.add_argument("--ros-setup", default=DEFAULT_ROS_SETUP, help="ROS 2 setup script path inside the distro")
    parser.add_argument(
        "--ros-ws-src", default=DEFAULT_WSL_ROS_WS_SRC, help="Colcon workspace src/ dir inside the distro"
    )
    args = parser.parse_args(argv)

    try:
        dest = args.dest
        if dest is None:
            addins_dir = _default_windows_addins_dir()
            dest = addins_dir / "Fusion2ROS"
    except FileNotFoundError as exc:
        print(f"Could not determine the FusionAddins destination: {exc}", file=sys.stderr)
        return 1

    if not args.source.is_dir():
        print(f"Sync aborted: source directory does not exist: {args.source}", file=sys.stderr)
        return 1

    entry_point_error = _verify_expected_addin_entry_point(args.source, dest)
    if entry_point_error:
        print(entry_point_error, file=sys.stderr)
        return 1

    writable_error = _check_dest_writable(dest)
    if writable_error:
        print(writable_error, file=sys.stderr)
        return 1

    def _sync_all() -> None:
        sync_addin_to_fusion(args.source, dest)
        sync_bridge_windows_into(dest)
        sync_robot_model_into(dest)
        sync_ros2_tools_validate_into(dest)

    try:
        _sync_all()
    except PermissionError as exc:
        print(
            f"Sync failed: destination is not writable ({exc}). Check that {dest} isn't open in "
            f"Explorer/Fusion or locked by a sync tool like OneDrive, and that you have write "
            f"permission there.",
            file=sys.stderr,
        )
        return 1
    except OSError as exc:
        print(f"Sync failed: {exc}", file=sys.stderr)
        return 1

    print(f"Synced {args.source} -> {dest}")
    print(f"Synced {BRIDGE_WINDOWS_SOURCE} -> {dest / 'bridge' / 'windows'}")
    print(f"Synced {ROBOT_MODEL_SOURCE} -> {dest / 'robot_model'}")
    print(f"Synced {ROS2_TOOLS_VALIDATE_SOURCE} -> {dest / 'ros2_tools' / 'validate'}")

    env_ok = True
    if not args.skip_env_check:
        print()
        print('Running WSL/ROS 2 environment checks (same as Fusion\'s "Check WSL Environment" command)...')
        checks = run_environment_checks(
            distro=args.distro,
            ros_setup=args.ros_setup,
            wsl_ros_ws_src=args.ros_ws_src,
            check_gazebo=args.check_gazebo,
            include_build_probe=args.full_env_check,
        )
        print(format_report(checks))
        env_ok = all_critical_passed(checks)
        print()
        if env_ok:
            subset_note = "" if args.full_env_check else " (fast subset -- pass --full-env-check for the real colcon build probe too)"
            print(f"Install + environment check both OK{subset_note}.")
        else:
            print(
                "Files are synced, but the environment check above found problems -- fix those "
                "before building/launching from Fusion (Generate/Validate will still work).",
                file=sys.stderr,
            )

    print()
    print(
        "Remaining manual step: in Fusion, go to Tools -> Add-Ins -> Scripts and Add-Ins -> "
        "Add-Ins tab -> Fusion2ROS -> Run."
    )

    if args.watch:
        print(f"Watching for changes every {args.interval}s (Ctrl-C to stop)...")
        try:
            while True:
                time.sleep(args.interval)
                try:
                    _sync_all()
                except OSError as exc:
                    print(f"Resync failed (will retry): {exc}", file=sys.stderr)
        except KeyboardInterrupt:
            print("Stopped.")

    return 0 if env_ok else 1


if __name__ == "__main__":
    sys.exit(main())
