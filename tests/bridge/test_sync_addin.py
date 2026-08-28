import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import bridge.windows.sync_addin as sync_addin_module
from bridge.windows.doctor import DoctorCheck
from bridge.windows.sync_addin import (
    _check_dest_writable,
    _verify_expected_addin_entry_point,
    sync_addin_to_fusion,
)


def test_sync_copies_files(tmp_path):
    source = tmp_path / "src"
    (source / "sub").mkdir(parents=True)
    (source / "a.py").write_text("A")
    (source / "sub" / "b.py").write_text("B")

    dest = tmp_path / "dest"
    sync_addin_to_fusion(source, dest)

    assert (dest / "a.py").read_text() == "A"
    assert (dest / "sub" / "b.py").read_text() == "B"


def test_sync_is_idempotent_and_prunes_stale_files(tmp_path):
    source = tmp_path / "src"
    source.mkdir()
    (source / "keep.py").write_text("keep")
    (source / "remove_me.py").write_text("stale")

    dest = tmp_path / "dest"
    sync_addin_to_fusion(source, dest)
    assert (dest / "remove_me.py").exists()

    (source / "remove_me.py").unlink()
    sync_addin_to_fusion(source, dest)

    assert (dest / "keep.py").read_text() == "keep"
    assert not (dest / "remove_me.py").exists()


def test_sync_prunes_pycache(tmp_path):
    source = tmp_path / "src"
    (source / "__pycache__").mkdir(parents=True)
    (source / "__pycache__" / "a.cpython-314.pyc").write_bytes(b"junk")
    (source / "real.py").write_text("real")

    dest = tmp_path / "dest"
    sync_addin_to_fusion(source, dest)

    assert not (dest / "__pycache__").exists()
    assert (dest / "real.py").read_text() == "real"


def test_sync_missing_source_raises(tmp_path):
    with pytest.raises(FileNotFoundError):
        sync_addin_to_fusion(tmp_path / "does_not_exist", tmp_path / "dest")


# --- _verify_expected_addin_entry_point -------------------------------------


def test_verify_entry_point_ok_when_matching_py_file_present(tmp_path):
    source = tmp_path / "src"
    source.mkdir()
    (source / "Fusion2ROS.py").write_text("# addin")
    dest = tmp_path / "FusionAddins" / "Fusion2ROS"

    assert _verify_expected_addin_entry_point(source, dest) is None


def test_verify_entry_point_errors_when_matching_py_file_missing(tmp_path):
    source = tmp_path / "src"
    source.mkdir()
    (source / "app.py").write_text("# app")
    dest = tmp_path / "FusionAddins" / "Fusion2ROS"

    error = _verify_expected_addin_entry_point(source, dest)
    assert error is not None
    assert "Fusion2ROS.py" in error
    assert str(source) in error


# --- _check_dest_writable -----------------------------------------------


def test_check_dest_writable_ok_for_writable_new_dir(tmp_path):
    dest = tmp_path / "FusionAddins" / "Fusion2ROS"
    assert _check_dest_writable(dest) is None


def test_check_dest_writable_errors_for_readonly_ancestor(tmp_path):
    import os

    readonly_parent = tmp_path / "readonly"
    readonly_parent.mkdir()
    os.chmod(readonly_parent, 0o500)
    dest = readonly_parent / "Fusion2ROS"
    try:
        error = _check_dest_writable(dest)
    finally:
        os.chmod(readonly_parent, 0o700)  # so tmp_path cleanup can remove it

    assert error is not None
    assert "not writable" in error


# --- main() ---------------------------------------------------------------


def _fake_source(tmp_path: Path) -> Path:
    source = tmp_path / "fusion_addin"
    source.mkdir()
    (source / "Fusion2ROS.py").write_text("# addin")
    return source


def test_main_fails_fast_when_source_missing_entry_point(tmp_path, capsys):
    source = tmp_path / "fusion_addin"
    source.mkdir()
    (source / "app.py").write_text("# app, no Fusion2ROS.py")
    dest = tmp_path / "FusionAddins" / "Fusion2ROS"

    rc = sync_addin_module.main(["--source", str(source), "--dest", str(dest), "--skip-env-check"])

    assert rc == 1
    assert not dest.exists()
    assert "Fusion2ROS.py" in capsys.readouterr().err


def test_main_fails_fast_when_dest_not_writable(tmp_path, capsys):
    import os

    source = _fake_source(tmp_path)
    readonly_parent = tmp_path / "readonly"
    readonly_parent.mkdir()
    os.chmod(readonly_parent, 0o500)
    dest = readonly_parent / "Fusion2ROS"

    try:
        rc = sync_addin_module.main(["--source", str(source), "--dest", str(dest), "--skip-env-check"])
    finally:
        os.chmod(readonly_parent, 0o700)

    assert rc == 1
    assert "not writable" in capsys.readouterr().err


def test_main_succeeds_and_copies_files_with_env_check_skipped(tmp_path):
    source = _fake_source(tmp_path)
    dest = tmp_path / "FusionAddins" / "Fusion2ROS"

    rc = sync_addin_module.main(["--source", str(source), "--dest", str(dest), "--skip-env-check"])

    assert rc == 0
    assert (dest / "Fusion2ROS.py").read_text() == "# addin"
    # The three absolute-import packages fusion_addin needs are deployed too.
    assert (dest / "bridge" / "windows" / "detect.py").exists()
    assert (dest / "robot_model").is_dir()
    assert (dest / "ros2_tools" / "validate").is_dir()


def test_main_exit_code_reflects_failed_env_check(tmp_path, monkeypatch, capsys):
    source = _fake_source(tmp_path)
    dest = tmp_path / "FusionAddins" / "Fusion2ROS"

    monkeypatch.setattr(
        sync_addin_module,
        "run_environment_checks",
        lambda *a, **kw: [DoctorCheck("WSL installed", False, "wsl.exe not found")],
    )

    rc = sync_addin_module.main(["--source", str(source), "--dest", str(dest)])

    assert rc == 1
    # Files were still copied -- a bad environment doesn't undo a good sync.
    assert (dest / "Fusion2ROS.py").exists()
    err = capsys.readouterr().err
    assert "environment check above found problems" in err


def test_main_exit_code_zero_when_env_check_passes(tmp_path, monkeypatch):
    source = _fake_source(tmp_path)
    dest = tmp_path / "FusionAddins" / "Fusion2ROS"

    monkeypatch.setattr(
        sync_addin_module,
        "run_environment_checks",
        lambda *a, **kw: [DoctorCheck("WSL installed", True, "OK")],
    )

    rc = sync_addin_module.main(["--source", str(source), "--dest", str(dest)])

    assert rc == 0


def test_main_env_check_defaults_to_fast_subset(tmp_path, monkeypatch):
    """The post-sync check should default to include_build_probe=False (a
    fast subset) -- the real colcon-build probe is slow and shouldn't run
    on every install/sync unless the user opts in with --full-env-check."""
    source = _fake_source(tmp_path)
    dest = tmp_path / "FusionAddins" / "Fusion2ROS"

    captured_kwargs = {}

    def fake_checks(*args, **kwargs):
        captured_kwargs.update(kwargs)
        return [DoctorCheck("WSL installed", True, "OK")]

    monkeypatch.setattr(sync_addin_module, "run_environment_checks", fake_checks)

    sync_addin_module.main(["--source", str(source), "--dest", str(dest)])
    assert captured_kwargs["include_build_probe"] is False

    captured_kwargs.clear()
    sync_addin_module.main(["--source", str(source), "--dest", str(dest), "--full-env-check"])
    assert captured_kwargs["include_build_probe"] is True


def test_main_skip_env_check_never_calls_doctor(tmp_path, monkeypatch):
    source = _fake_source(tmp_path)
    dest = tmp_path / "FusionAddins" / "Fusion2ROS"

    def fail_if_called(*args, **kwargs):
        raise AssertionError("run_environment_checks should not be called with --skip-env-check")

    monkeypatch.setattr(sync_addin_module, "run_environment_checks", fail_if_called)

    rc = sync_addin_module.main(["--source", str(source), "--dest", str(dest), "--skip-env-check"])
    assert rc == 0


def test_main_missing_source_dir_reports_clean_error_not_traceback(tmp_path, capsys):
    dest = tmp_path / "FusionAddins" / "Fusion2ROS"
    rc = sync_addin_module.main(
        ["--source", str(tmp_path / "does_not_exist"), "--dest", str(dest), "--skip-env-check"]
    )
    assert rc == 1
    assert "does not exist" in capsys.readouterr().err
