import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from bridge.windows.sync_addin import sync_addin_to_fusion


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
    import pytest

    with pytest.raises(FileNotFoundError):
        sync_addin_to_fusion(tmp_path / "does_not_exist", tmp_path / "dest")
