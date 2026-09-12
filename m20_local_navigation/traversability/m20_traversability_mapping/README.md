# M20 Traversability Mapping

Robot-centric terrain analysis built on `/m20/local_mapping/evidence_cloud`. This package owns its
interfaces and does not publish velocity, gait, motion-control, fake odometry, or fake TF data.

Outputs:

- `/m20/local_mapping/traversability/grid`: `OccupancyGrid`, `-1` unknown, `0..90` traversable cost,
  `100` blocked/inflated.
- `/m20/local_mapping/traversability/obstacle_cloud`: blocked cells for trajectory collision checks.
- `/m20/local_mapping/traversability/terrain_cloud`: known terrain cells, intensity is cost.
- `/m20/local_mapping/traversability/diagnostics`: input freshness and processing statistics.

The grid preserves the evidence-cloud header stamp and frame. Empty space is not interpreted as free.
Only completely bounded, small holes with consistent boundary elevation are interpolated.
