# M20 FIRST Planner V2

V2 is an isolated planner chain. It does not modify or replace the saved V1
packages.

## Decision Order

The default planner executable is now `m20_first_planner_v2_cpp`. For every
10 Hz control cycle:

1. Evaluate all 4,641 official FIRST candidates at scale 1.00 and range 4.00 m.
2. If no candidate survives, retry at scale 0.75 and range 3.00 m.
3. If still blocked, keep scale 0.75 and reduce range through 2.50, 2.00,
   1.50, and 1.00 m.
4. Recompute static correspondence collisions and time-aligned dynamic costs
   for every fallback stage.
5. Only after all FIRST stages fail, evaluate the outer-layer escape cost
   against candidates that remain statically collision-free.
6. If escape also fails, publish zero `/NAV_CMD` and a one-point stop path.

## Escape Safety Contract

Escape is preventive motion, not collision recovery:

- A predicted collision with the stationary robot activates a short threat
  latch. The latch preserves only the threat state across brief perception
  gaps; it never preserves a path or a control command.
- Every control cycle recomputes all escape candidates against the latest
  object predictions.
- A candidate is rejected if it has any predicted physical contact during the
  configured 1.2 s escape horizon. A later collision in the 3 s scoring
  horizon does not prevent the planner from making an immediately safe
  evasive move and replanning at 10 Hz.
- A candidate's minimum clearance across the complete escape window must
  improve over the stationary robot's minimum clearance in that same window.
- If the true braking rollout intersects a dynamic object's future swept
  volume, the current position is marked as intruded. The planner bypasses
  stop-release hysteresis and executes a safe escape immediately.
- The intrusion state remains latched through prediction coasting. It is
  released only after four distinct fresh reachability frames report no
  current or latched threat.
- General dynamic-stop release counts distinct dynamic-cost generations, not
  repeated control-timer calls on the same result.
- If no strictly safe candidate exists, the planner stops. It never continues
  an earlier escape command blindly.

## Separation From V1

- Package: `m20_first_planner_v2`
- Node: `/m20_first_planner_v2`
- Dynamic cost matrix: `/m20/v2/first_dynamic_costs`
- Escape costs: `/m20/v2/first_dynamic_escape_costs`
- Planner state: `/m20/v2/planner_stage`
- Local path: `/m20/v2/local_plan`
- RViz markers: `/m20/v2/first_selected_prediction_markers`
- Controller output: `/NAV_CMD`

The shared `/NAV_CMD` is intentional because it is the real M20 controller
interface. V1 and V2 planners must never run at the same time.

## Dynamic Cost Matrix

`/m20/v2/first_dynamic_costs` is a `Float32MultiArray` with shape:

```text
[6 fallback stages, 4641 FIRST paths]
```

The C++ scorer recalculates the robot trajectory, exact-horizon collision,
1--3 second integrated prediction, and object clearance for every stage.
The planner rejects a matrix unless its dimensions match the configured
fallback stages.

## Atomic Snapshot Contract

The C++ dynamic scorer additionally publishes
`/m20/v2/dynamic_scoring_snapshot`. One DDS sample contains:

- generation and validity;
- all fallback-stage dynamic costs;
- escape costs;
- local dynamic-object geometry;
- true-controller speed limits;
- braking reachability and intrusion state.

The C++ planner captures immutable cloud, odometry, goal, and dynamic snapshot
references before scoring. It commits `/NAV_CMD` only when:

1. the dynamic bundle is complete and fresh;
2. cloud-to-dynamic receive-time skew is at most 0.20 s;
3. no captured input generation changed during scoring;
4. computation completed within the 80 ms deadline.

A superseded or late result is discarded and cannot overwrite a command
computed from newer perception. Static FIRST collision scoring and dynamic
prediction are therefore evaluated from one coherent planning snapshot.

The planner uses separate ROS callback groups and a multithreaded executor, so
sensor callbacks continue accepting new data while the immutable snapshot is
scored. Candidate selection also has a short commit hysteresis: a safe current
path is retained unless another path improves its score by the configured
margin. An unsafe path or active intrusion bypasses this hold immediately.

When `require_dynamic_input` is enabled, the planner rejects stale or missing
dynamic input. In that state it publishes a stop command instead of silently
falling back to static-only planning.

## Pedestrian Right Of Way

The outer C++ scorer classifies every moving-object interaction before adding
the dynamic cost:

- `YIELD_BEHIND`: the object clears the shared conflict zone first. This is the
  preferred behavior and adds no interaction penalty.
- `PASS_FIRST`: the robot can fully clear the zone first with the measured
  velocity and the true-controller acceleration limits. It remains feasible,
  but receives a strong preference penalty.
- `HIGH_RISK`: the occupancy windows overlap after prediction uncertainty is
  applied. It receives a larger risk penalty, while exact time-aligned physical
  overlap remains a hard collision.

This arbitration changes only V2 outer-layer scores. It does not modify the
4,641 official FIRST paths, static correspondence collision logic, or the
official `/NAV_CMD` execution chain.

## Acceptance Result

The close lateral crossing test used a 0.35 m radius moving cylinder at
0.35 m/s. The robot reached a minimum center distance of 0.968 m, made 4.248 m
of forward progress, and completed the goal without contact. Runtime states
showed `normal -> blocked/escape -> normal -> goal_reached`; `blocked` cycles
published stop commands and every `escape` cycle was selected from fresh
costs.

## Runtime

Use the independent scripts:

```bash
/home/ubuntu/social_nav_ws/restart_m20_social_gazebo_v2.sh
```

The launch defaults to the C++ planner. The retained Python implementation is
an explicit diagnostic fallback:

```bash
ros2 launch m20_first_planner_v2 m20_first_v2_navigation.launch.py \
  planner_executable:=m20_first_planner_v2.py
```

Initial shadow-mode verification on the simulation machine measured about
3.3--5.7 ms for one complete 4,641-path, six-stage planning cycle and a stable
9.5--10 Hz `/NAV_CMD` rate. The former Python planner required about
1.8--2.4 s per cycle under the same simulation load.

V1 remains available through the original
`restart_m20_social_gazebo.sh`.

## Goal Diagnostics

`m20_v2_goal_diagnostics` runs with the V2 launch but remains idle until a
user goal arrives on `/goal_pose`, `/target_goal`, or
`/move_base_simple/goal`. Each goal creates a directory below:

```text
/home/ubuntu/social_nav_ws/logs/v2_goal_diagnostics/
```

Files in each session:

- `timeline.csv`: 10 Hz robot pose, odometry, `/NAV_CMD`, goal distance,
  planner mode, six-stage dynamic blockage counts, escape availability,
  obstacle clearance, grids, and paths.
- `planner_events.csv`: planner mode and fallback-stage transitions.
- `dynamic_objects.jsonl`: every dynamic-object frame used by V2.
- `summary.json`: sustained-stop episodes grouped by diagnosed cause.

The live status topic is `/m20/v2/goal_diagnostic_status`. A stop is reported
after robot speed remains below 0.05 m/s for 0.8 s while the global goal is
more than 0.5 m away.
