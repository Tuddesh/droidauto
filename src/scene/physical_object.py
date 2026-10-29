"""Common identity, dimensions and observed pose for every physical entity.

Buffer, BufferItem and SceneObject inherit these fields. A pose is a 4x4
T_reference_object in metres; pose_frame explicitly names the reference.
Supply object_id from configuration when identity must survive new processes.
Observation updates are declared for later implementation.
"""

from dataclasses import dataclass, field
from uuid import uuid4

import numpy as np


@dataclass(kw_only=True, eq=False)
class PhysicalObject:
    """Shared physical data; generated IDs identify otherwise unnamed instances."""

    object_id: str = field(default_factory=lambda: str(uuid4()))
    dimensions_m: tuple[float, float, float] | None = None
    pose: np.ndarray | None = field(default=None, repr=False)
    pose_frame: str | None = None
    observed_at: float | None = None

    def update_pose(self, pose: np.ndarray, reference_frame: str, timestamp: float) -> None:
        """Update this object's observed pose and provenance; not implemented yet."""
        raise NotImplementedError("Physical-object observation updates are not implemented")

    def mark_unobserved(self) -> None:
        """Mark an object missing from the latest observation; not implemented yet."""
        raise NotImplementedError("Missing-object observation handling is not implemented")
