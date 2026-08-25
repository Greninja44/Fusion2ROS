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
"""

from __future__ import annotations

import argparse
import shutil
import sys
import time
from pathlib import Path

DEFAULT_SOURCE = Path(__file__).resolve().parents[2] / "fusion_addin"


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

    sync_addin_to_fusion(args.source, dest)
    print(f"Synced {args.source} -> {dest}")

    if args.watch:
        print(f"Watching for changes every {args.interval}s (Ctrl-C to stop)...")
        try:
            while True:
                time.sleep(args.interval)
                sync_addin_to_fusion(args.source, dest)
        except KeyboardInterrupt:
            print("Stopped.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
