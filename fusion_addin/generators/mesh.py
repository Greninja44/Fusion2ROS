"""Exports Fusion body geometry to STL mesh files, one per RobotModel link.

*** UNVERIFIED against a live Fusion process, same as fusion_addin/extraction/
fusion_adapter.py -- written from documented adsk.fusion.ExportManager /
STLExportOptions API (createSTLExportOptions / execute is one of the most
common, stable patterns in third-party Fusion API STL-export samples), but
this sandbox has no adsk.fusion to actually run it against. Review carefully
against a real Fusion session before trusting it in production.

Design choice (documented, not hidden): one STL per Link, exporting the
occurrence's Fusion bodies. If an occurrence has multiple bodies, Fusion's
occurrence-level export combines them into a single mesh file -- simplest
thing that works for MVP; per-body-separate meshes (e.g. for independently
colored/collidable sub-parts) is a documented future enhancement, not
attempted here since not every occurrence maps cleanly to one intended
"part" and guessing that split would be inventing behavior, not observing it.
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict

try:
    import adsk.core
    import adsk.fusion

    _ADSK_AVAILABLE = True
except ImportError:
    _ADSK_AVAILABLE = False

from robot_model import Robot


def _require_adsk() -> None:
    if not _ADSK_AVAILABLE:
        raise RuntimeError(
            "fusion_addin.generators.mesh requires the adsk.core/adsk.fusion modules, "
            "which are only available inside a running Fusion 360 process."
        )


def export_link_meshes(design, robot: Robot, output_dir: Path) -> Dict[str, Path]:
    """Exports one STL per link in `robot` to `output_dir/<link.name>.stl`,
    matching each link back to its live Fusion occurrence via the
    `fusion_occurrence` key that fusion_addin.extraction.converter stashes in
    Link.metadata. Links with no `fusion_occurrence` metadata (e.g. hand-built
    or previously-generated Robots not sourced from this design) are skipped,
    not errored -- mesh export is best-effort per link.

    Returns {link_name: exported_file_path} for the links that were exported.

    design: adsk.fusion.Design (typically app.activeProduct)
    """
    _require_adsk()
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    occurrences_by_name = {occ.name: occ for occ in design.rootComponent.allOccurrences}
    export_mgr = design.exportManager

    exported: Dict[str, Path] = {}
    for link in robot.links:
        fusion_name = link.metadata.get("fusion_occurrence")
        if fusion_name is None:
            continue
        occ = occurrences_by_name.get(fusion_name)
        if occ is None:
            continue
        if not list(occ.bRepBodies):
            continue  # nothing to export (e.g. a pure-reference/empty occurrence)

        dest = output_dir / f"{link.name}.stl"
        # Link.name has no character restrictions in robot_model.schema (the
        # normal Fusion pipeline sanitizes it upstream in
        # fusion_addin.extraction.converter.sanitize_link_name, but this
        # function accepts any Robot, e.g. one hand-built or loaded from
        # JSON). Since Path("/out") / "/etc/passwd" == Path("/etc/passwd"),
        # an absolute-looking or ".."-containing link name would otherwise
        # make the Fusion STL exporter write outside output_dir entirely.
        if dest.resolve().parent != output_dir.resolve():
            raise ValueError(
                f"Link.name {link.name!r} would export outside {output_dir} "
                f"(resolves to {dest.resolve()})"
            )
        stl_options = export_mgr.createSTLExportOptions(occ, str(dest))
        export_mgr.execute(stl_options)
        exported[link.name] = dest

    return exported
