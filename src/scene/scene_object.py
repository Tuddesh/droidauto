"""Manipulation objects extend shared physical data with grasp descriptions.

Observed pose/identity are inherited from PhysicalObject. Grasp frames are
expressed in the object's own coordinates; the reset planner will resolve them
into robot targets. Perception bindings belong to a localization adapter.
"""

from dataclasses import dataclass, field

import numpy as np

from .physical_object import PhysicalObject


@dataclass(kw_only=True, eq=False)
class SceneObject(PhysicalObject):
    """An identified task object with optional grasp and manipulation parameters."""

    grasps: list[np.ndarray] = field(default_factory=list, repr=False)
    manipulation: dict[str, object] = field(default_factory=dict)

    def grasp_poses(self) -> list[np.ndarray]:
        """Return object-relative grasp frames; grasp handling is not implemented."""
        raise NotImplementedError("Scene-object grasp handling is not implemented")


@dataclass(kw_only=True, eq=False)
class Cube(SceneObject):
    """Cube data uses inherited dimensions_m=(edge, edge, edge) and grasps.

    Object-specific localization and grasp generation remain unimplemented.
    """
