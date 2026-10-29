# Physical scene models

This package prepares the shared object model for automatic scene reset.

```text
PhysicalObject (dataclass)
├── Buffer                         buffer/buffer.py
├── BufferItem                     buffer/buffer_item.py
│   └── ArucoTile                   buffer/items/aruco_tile.py
└── SceneObject                    scene_object.py
    └── Cube                       scene_object.py
```

## Shared parameters

`PhysicalObject` owns `object_id`, `dimensions_m`, `pose`, `pose_frame`, and
`observed_at`. Subclasses add their own parameters rather than another identity
or observed pose. Supply a configured `object_id` for identity across sessions;
the generated default UUID only identifies a newly constructed instance.

`pose` means `T_reference_object`: it maps object coordinates into the frame
named by `pose_frame`. Translation is in metres. Buffer observations may be
expressed in the robot base frame, while task-object observations may use the
named buffer frame. A matrix without its reference frame is not a complete
observation. `dimensions_m=None` means physical extents are unspecified.

Buffer's dimensions describe its grid reference-plane extent, with zero Z
thickness; they do not model the fixture's solid volume. BufferItem adds the
integer cell footprint. Its read-only `item_id` property is a compatibility
alias for the inherited `object_id`, a string.
Buffer placement uses `object_id` directly.

`SceneObject` adds object-relative grasp frames and manipulation parameters.
`Cube` uses those same fields; its dimensions can be declared `(edge, edge,
edge)`. Cube perception and automatic grasp generation are not implemented.

## Prepared interfaces

- `PhysicalObject.update_pose(pose, reference_frame, timestamp)`
- `PhysicalObject.mark_unobserved()`
- `SceneObject.grasp_poses()`

The observation/grasp methods intentionally raise `NotImplementedError`.
The concrete supplied-tool adapter API is deferred until integration.
`Buffer.localize()` and
`Buffer.localize_stereo()` retain their existing implementations and ownership.
They return camera-relative estimates; future manager observation code will
compose them into the desired reference frame and update shared object state.

Desired placements belong to `auto_reset.TargetConfiguration`; they do not
overwrite the objects' observed poses. See the [reset interfaces](../auto_reset/README.md).
