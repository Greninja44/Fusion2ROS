"""Tests for fusion_addin.extraction.fusion_adapter using mocked adsk.core/
adsk.fusion module objects (not a live Fusion process -- none of this file's
functions have ever been run against real Fusion, see fusion_adapter.py's own
module docstring). This is the same "mock adsk in sys.modules before import"
technique used to reproduce and fix the real ModuleNotFoundError bug found
earlier in this project, applied here to give this previously fully-untested
file at least basic coverage of its pure translation logic.

Focus: the real bug this file's As-Built Joint support fixes -- an assembly
whose joints were all authored as As-Built Joints (a separate Fusion
collection from regular Joints) reported zero joints detected. These tests
mock just enough of the adsk object shapes (per AsBuiltJoint.htm/Joint.htm,
fetched live during that fix) to exercise the real conversion code paths.
"""

import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

# fusion_adapter.py gates its real logic behind `import adsk.core` /
# `import adsk.fusion` succeeding -- mock both so _ADSK_AVAILABLE is True,
# matching how a real Fusion process would present them.
sys.modules.setdefault("adsk", MagicMock())
sys.modules.setdefault("adsk.core", MagicMock())
sys.modules.setdefault("adsk.fusion", MagicMock())

from fusion_addin.extraction import fusion_adapter as fa  # noqa: E402


def _point3d(x, y, z):
    return SimpleNamespace(x=x, y=y, z=z)


def _identity_matrix3d(origin=(0.0, 0.0, 0.0)):
    """A fake adsk.core.Matrix3D exposing just getAsCoordinateSystem(),
    matching _matrix3d_to_pose's documented usage -- identity rotation at
    the given origin."""
    matrix = MagicMock()
    matrix.getAsCoordinateSystem.return_value = (
        _point3d(*origin),
        _point3d(1.0, 0.0, 0.0),
        _point3d(0.0, 1.0, 0.0),
        _point3d(0.0, 0.0, 1.0),
    )
    return matrix


def _fake_occurrence(name):
    return SimpleNamespace(name=name)


def _fake_revolute_motion(axis=(0.0, 0.0, 1.0), lower=-1.0, upper=1.0):
    motion = MagicMock()
    motion.jointType.name = "RevoluteJointType"
    motion.rotationAxisVector = _point3d(*axis)
    motion.rotationLimits = SimpleNamespace(
        isMinimumValueEnabled=lower is not None,
        isMaximumValueEnabled=upper is not None,
        minimumValue=lower,
        maximumValue=upper,
    )
    return motion


def _fake_rigid_motion():
    motion = MagicMock()
    motion.jointType.name = "RigidJointType"
    return motion


# ---------------------------------------------------------------------------
# As-Built Joint conversion -- the real bug fix.
# ---------------------------------------------------------------------------


def test_as_built_joint_uses_transform_directly_for_origin():
    joint = MagicMock()
    joint.name = "as_built_1"
    joint.occurrenceOne = _fake_occurrence("base:1")
    joint.occurrenceTwo = _fake_occurrence("arm1:1")
    joint.jointMotion = _fake_revolute_motion(axis=(0.0, 0.0, 1.0), lower=-2.0, upper=2.0)
    joint.transform = _identity_matrix3d(origin=(5.0, 0.0, 0.0))

    info = fa._as_built_joint_to_fusion_joint_info(joint)

    assert info.name == "as_built_1"
    assert info.occurrence_one == "base:1"
    assert info.occurrence_two == "arm1:1"
    assert info.joint_type == "RevoluteJointType"
    assert info.axis == pytest.approx((0.0, 0.0, 1.0))
    assert info.lower_limit == pytest.approx(-2.0)
    assert info.upper_limit == pytest.approx(2.0)
    assert info.origin.xyz == pytest.approx((5.0, 0.0, 0.0))


def test_as_built_rigid_joint_has_no_axis_or_limits():
    joint = MagicMock()
    joint.name = "as_built_rigid"
    joint.occurrenceOne = _fake_occurrence("base:1")
    joint.occurrenceTwo = _fake_occurrence("bracket:1")
    joint.jointMotion = _fake_rigid_motion()
    joint.transform = _identity_matrix3d()

    info = fa._as_built_joint_to_fusion_joint_info(joint)

    assert info.joint_type == "RigidJointType"
    assert info.axis is None
    assert info.lower_limit is None
    assert info.upper_limit is None


# ---------------------------------------------------------------------------
# list_joints() combines BOTH collections -- this is exactly the real bug:
# an assembly with only As-Built Joints used to report zero joints at all.
# ---------------------------------------------------------------------------


def test_list_joints_combines_regular_and_as_built_joints():
    regular_joint = MagicMock()
    regular_joint.name = "regular_1"
    regular_joint.occurrenceOne = _fake_occurrence("base:1")
    regular_joint.occurrenceTwo = _fake_occurrence("link1:1")
    regular_joint.jointMotion = _fake_rigid_motion()
    regular_joint.geometryOrOriginOne = SimpleNamespace(transform=_identity_matrix3d())

    as_built_joint = MagicMock()
    as_built_joint.name = "as_built_1"
    as_built_joint.occurrenceOne = _fake_occurrence("link1:1")
    as_built_joint.occurrenceTwo = _fake_occurrence("link2:1")
    as_built_joint.jointMotion = _fake_revolute_motion()
    as_built_joint.transform = _identity_matrix3d(origin=(1.0, 2.0, 3.0))

    fake_root = SimpleNamespace(allJoints=[regular_joint], allAsBuiltJoints=[as_built_joint])

    adapter = fa.FusionDesignReaderAdapter.__new__(fa.FusionDesignReaderAdapter)
    adapter._root = fake_root

    joints = adapter.list_joints()

    assert {j.name for j in joints} == {"regular_1", "as_built_1"}
    as_built_result = next(j for j in joints if j.name == "as_built_1")
    assert as_built_result.origin.xyz == pytest.approx((1.0, 2.0, 3.0))


def test_list_joints_with_no_as_built_joints_still_works():
    regular_joint = MagicMock()
    regular_joint.name = "regular_only"
    regular_joint.occurrenceOne = _fake_occurrence("base:1")
    regular_joint.occurrenceTwo = _fake_occurrence("link1:1")
    regular_joint.jointMotion = _fake_rigid_motion()
    regular_joint.geometryOrOriginOne = SimpleNamespace(transform=_identity_matrix3d())

    fake_root = SimpleNamespace(allJoints=[regular_joint], allAsBuiltJoints=[])

    adapter = fa.FusionDesignReaderAdapter.__new__(fa.FusionDesignReaderAdapter)
    adapter._root = fake_root

    joints = adapter.list_joints()
    assert len(joints) == 1
    assert joints[0].name == "regular_only"
