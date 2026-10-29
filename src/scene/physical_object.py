"""Common identity, dimensions and observed pose for every physical entity.

Buffer, BufferItem and SceneObject inherit these fields. A pose is a 4x4
T_reference_object in metres; pose_frame explicitly names the reference.
Supply object_id from configuration when identity must survive new processes.
Observations are supplied by the caller; this class performs no localization.
"""

from dataclasses import dataclass, field
from uuid import uuid4
import math

import numpy as np

from .transforms import transform


@dataclass(kw_only=True, eq=False)
class PhysicalObject:
    """Shared physical data; generated IDs identify otherwise unnamed instances."""

    object_id: str = field(default_factory=lambda: str(uuid4()))
    dimensions_m: tuple[float, float, float] | None = None
    pose: np.ndarray | None = field(default=None, repr=False)
    pose_frame: str | None = None
    observed_at: float | None = None

    def update_pose(self, pose: np.ndarray, reference_frame: str, timestamp: float) -> None:
        """Copy a supplied measured pose and its named frame/timestamp."""
        if not isinstance(reference_frame, str) or not reference_frame.strip():
            raise ValueError("An observation needs a named reference frame")
        if not math.isfinite(timestamp):
            raise ValueError("Observation timestamp must be finite")
        self.pose = transform(pose)
        self.pose_frame, self.observed_at = reference_frame, float(timestamp)

    def mark_unobserved(self) -> None:
        """Clear an observation whose current pose is no longer known."""
        self.pose = self.pose_frame = self.observed_at = None
