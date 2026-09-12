# M20 Evidence Mapping

Industrial robot-centric evidence voxel mapping for the M20 local navigation stack.

This package intentionally does not publish the manufacturer's `/NAV_POINTS` contract.
It publishes `/m20/local_mapping/evidence_cloud`; downstream systems must consume the
owned interface after validation.

Safety properties:

- Cloud frames are rejected when IMU/ODOM interpolation is unavailable.
- Non-monotonic cloud time and odometry jumps reset accumulation.
- Occupied, strong-free, weak-free and occluded evidence are separate states.
- Input timestamps are preserved exactly.
- The node never publishes motion commands.

The initial deployment is a side-channel evaluation only. Terrain clearance and final
impassable-area classification are implemented as the next audited stage after sensor
extrinsics and visibility geometry are validated from recorded data.
