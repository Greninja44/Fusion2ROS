"""Fusion design -> robot_model.Robot extraction.

`interface.py` and `converter.py` are Fusion-symbol-free and fully unit
testable. `fusion_adapter.py` implements the interface against real
`adsk.core`/`adsk.fusion` calls and is only usable inside a running Fusion
360 process; importing this package outside Fusion works fine (the `adsk`
import in fusion_adapter.py is guarded), but instantiating
`FusionDesignReaderAdapter` outside Fusion raises a clear RuntimeError.
"""

from .converter import ExtractionError, UnsupportedJointTypeError, build_robot_model
from .interface import FusionDesignReader, FusionInertia, FusionJointInfo, FusionOccurrence, FusionPose

# Importing fusion_adapter never raises outside Fusion -- its `import adsk...`
# is guarded internally (see fusion_adapter.py). Only *instantiating*
# FusionDesignReaderAdapter outside a running Fusion process raises.
from .fusion_adapter import FusionDesignReaderAdapter

__all__ = [
    "FusionDesignReader",
    "FusionOccurrence",
    "FusionJointInfo",
    "FusionPose",
    "FusionInertia",
    "build_robot_model",
    "ExtractionError",
    "UnsupportedJointTypeError",
    "FusionDesignReaderAdapter",
]
