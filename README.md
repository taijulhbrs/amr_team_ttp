# AMR Project — Robile4

Deployment of path/motion planning, localisation, and environment exploration
on the Robile modular mobile robot platform, developed as part of the
Autonomous Mobile Robots course project.

## Project Objectives

1. **Path & Motion Planning** — port a potential field planner to the real
   robot, and combine it with a global path planner (A*) so it can navigate
   large, mapped environments.
2. **Localisation** — implement a Monte Carlo Localisation (particle filter)
   from scratch to estimate the robot's pose on a known map.
3. **Environment Exploration** — combine frontier-based pose selection with
   SLAM so the robot can autonomously map unknown areas.

All three parts are implemented in this repository.

---

## Repository / Package Overview

| Package | Purpose | Used for |
|---|---|---|
| `robile_description` | Robot URDF/xacro model | Sim + real |
| `robile_gazebo` | Gazebo simulation launch files | Sim only |
| `robile_navigation` | SLAM (`slam_toolbox`) launch files, saved maps | Sim + real |
| `robile_bringup` | Real hardware driver bring-up | Real robot only |
| `robile_planning` | **Part 1** — A* global planner + potential field local planner | Sim + real |
| `robile_localization` | **Part 2** — Monte Carlo Localisation (particle filter) | Sim + real |
| `robile_exploration` | **Part 3** — frontier-based autonomous exploration | Sim + real |

---

## Part 1: Path & Motion Planning

**Package:** `robile_planning`

### Nodes

- **`global_planner`** — runs A* search over an inflated occupancy grid
  (`/map`), from the robot's current pose to a given goal (`/goal_pose`).
  The raw cell-by-cell path is simplified down to waypoints kept only at
  direction changes, and published on `/global_path`.
- **`potential_field_planner`** — subscribes to `/global_path` and tracks an
  index into the waypoint list. At each control step it computes an
  **attractive force** toward the current target waypoint and a
  **repulsive force** from any laser scan point (`/scan`) inside an
  influence radius, sums them, and converts the resultant force vector
  into a `/cmd_vel` command. A hard safety stop overrides this if any
  obstacle (static or dynamic, map-known or not) is closer than
  `safety_distance`. When the robot is within `waypoint_tolerance` of a
  waypoint it advances to the next one; reaching the last one stops the
  robot and logs `🎯 Reached goal!`.

### Key topics

| Topic | Type | Direction |
|---|---|---|
| `/map` | `nav_msgs/OccupancyGrid` | in (global_planner) |
| `/goal_pose` | `geometry_msgs/PoseStamped` | in (global_planner) — RViz "2D Goal Pose", or Part 3 |
| `/global_path` | `nav_msgs/Path` | out (global_planner) / in (potential_field_planner) |
| `/scan` | `sensor_msgs/LaserScan` | in (potential_field_planner) |
| `/cmd_vel` | `geometry_msgs/Twist` | out (potential_field_planner) |

### Pose source

Both nodes read the robot's pose via the `map → base_link` TF transform
(param `use_tf_pose: true`), provided by the Part 2 particle filter (or by
`slam_toolbox` during Part 3 exploration). If TF isn't available, they fall
back to raw `/odom`.

### Run it

```bash
ros2 launch robile_planning planning.launch.py
```

Send a goal via RViz's **"2D Goal Pose"** tool, or:
```bash
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
  '{header: {frame_id: "map"}, pose: {position: {x: 2.0, y: 1.0}}}'
```

### Tunable parameters (`config/planning_params.yaml`)

| Parameter | Effect |
|---|---|
| `attractive_gain` / `repulsive_gain` | Balance between reaching the goal vs. avoiding obstacles. Too much repulsive gain in narrow corridors causes oscillation/rotating-in-place. |
| `obstacle_influence_radius` | How far away obstacles start to repel the robot. |
| `inflation_radius_cells` | Obstacle buffer in the map used for A*. Too large can block narrow corridors entirely. |
| `max_linear_speed` / `max_angular_speed` | Speed caps. |
| The `abs(angle_error) > 1.2` threshold in `_apply_force_as_velocity` (code, not yaml) | Controls when the robot rotates-in-place vs. moves while turning. Lower it (e.g. `0.6`) for less "stop and rotate" behaviour. |

---

## Part 2: Localisation (Monte Carlo Localisation)

**Package:** `robile_localization`

### Node: `particle_filter`

A from-scratch particle filter (sequential importance resampling — the
"simple version" of MCL taught in lecture):

1. **Initialise** — particles scattered around an initial pose guess, given
   via RViz's **"2D Pose Estimate"** tool (`/initialpose`).
2. **Predict** — on each `/odom` update, every particle is moved by the
   same odometry delta plus noise (`alpha1`–`alpha4`).
3. **Update** — on each `/scan`, particles are weighted using a
   likelihood-field measurement model (a distance-to-nearest-obstacle grid
   precomputed once from the map via multi-source BFS).
4. **Resample** — low-variance resampling, proportional to weight.
5. **Publish** — the weighted mean pose is broadcast as the `map → odom` TF
   transform, plus `/particle_cloud` (`PoseArray`, visualise with RViz's
   PoseArray display) and `/pf_pose` (`PoseWithCovarianceStamped`).

### Run it

```bash
ros2 launch robile_localization localization.launch.py
```
In RViz: add a **PoseArray** display on `/particle_cloud`, then use
**"2D Pose Estimate"** to seed the initial pose.

### Known limitation: the kidnapped robot problem

This implementation **cannot recover automatically** if the robot's true
position is far from where the particles currently are — e.g. if the robot
is physically moved without giving a new "2D Pose Estimate", or if a wrong
initial pose is given. This is expected for the "simple version" of MCL
(no random-particle re-injection / global re-localisation). The project
description explicitly lists this as an **optional** extension (adaptive
MCL); it is not required for the core deliverable. Work-around: always give
a correct "2D Pose Estimate" after physically moving the robot.

### IMPORTANT: don't run this alongside SLAM

Both `particle_filter` and `slam_toolbox` broadcast the `map → odom`
transform. Running them at the same time causes a TF conflict. Use:
- `robile_localization` when navigating a **known, static** map (Part 1+2).
- `slam_toolbox` (`robile_navigation`'s `online_async.launch.py`) when
  **building** a map, including during Part 3 exploration.

---

## Part 3: Environment Exploration

**Package:** `robile_exploration`

### Node: `frontier_explorer`

Frontier-based autonomous exploration:

1. A **frontier cell** is a free cell with at least one unknown neighbour
   (the boundary between explored and unexplored space).
2. Frontier cells are clustered (4-connected BFS); clusters smaller than
   `min_frontier_size` are discarded as noise.
3. The nearest cluster centroid (by Euclidean distance to the robot) is
   chosen as the next goal — a greedy nearest-frontier strategy. Other
   strategies (largest frontier, information gain, etc.) could replace
   `_choose_target()`.
4. The goal is published on `/goal_pose`, consumed by the existing Part 1
   planner exactly like a manual RViz goal.
5. A timer re-checks every `explore_check_period` seconds: if the robot has
   reached the current goal (within `goal_tolerance`) or has been pursuing
   it too long (`goal_timeout`), a new frontier search runs. When no
   frontiers remain, exploration is declared complete.

### Run it (combined with SLAM + Part 1)

```bash
# Terminal A — SLAM (builds the live map AND provides map->odom TF)
ros2 launch robile_navigation online_async.launch.py

# Terminal B — Part 1 planner (drives the robot toward exploration goals)
ros2 launch robile_planning planning.launch.py

# Terminal C — Part 3 explorer (autonomously picks goals)
ros2 launch robile_exploration exploration.launch.py
```

Do **not** run `robile_localization` (Part 2) at the same time — see the
TF conflict note above. Once exploration finishes:
```bash
ros2 run nav2_map_server map_saver_cli -f explored_map
```

### How to tell it's working

- The map in RViz visibly grows (grey/unknown areas turn white/black) over
  time.
- The `frontier_explorer` terminal periodically logs
  `New exploration goal: (x, y) — N frontier region(s) available`, and `N`
  trends downward.
- The robot visibly moves on its own (no manual teleop).
- Eventually: `🗺️ Exploration complete! No more frontiers found.`

### Tunable parameters (`config/exploration_params.yaml`)

| Parameter | Effect |
|---|---|
| `min_frontier_size` | Minimum frontier cluster size to pursue (filters noise). |
| `goal_timeout` | How long to try reaching a frontier before giving up and picking a new one. |
| `goal_tolerance` | Distance at which a frontier goal is considered reached. |

### Note on map continuation

By default, `online_async.launch.py` starts SLAM from a **blank map** — it
does not automatically continue/extend a previously saved
`map_saver_cli` map (that only saves a flat image, not slam_toolbox's
internal pose graph). Continuing a specific previous SLAM session requires
slam_toolbox's own map serialization (`serialize_map` service) and loading
it back via the `map_file_name` / `mode: mapping` parameters — not covered
by the current setup.

---

## Full Pipeline: Map → Localisation → Planning (Part 1 + 2)

```
   Map (SLAM, saved once)
          │
          ▼
     map_server  ──────────────►  /map
          │
          ▼
  particle_filter (Part 2)  ─────►  /pf_pose, /particle_cloud, TF: map→odom
          │
          ▼
  global_planner (Part 1)  ──────►  /global_path   (A* over /map, using TF pose)
          │
          ▼
  potential_field_planner (Part 1) ─► /cmd_vel   (waypoint following + obstacle avoidance)
```

**The map used must match the actual environment the robot is driving in.**
A map built in one environment and used in a different one will cause the
particle filter to diverge (laser scans won't match the map, weights become
degenerate, pose estimate drifts outside the map bounds).

## Exploration Pipeline (Part 3)

```
  slam_toolbox  ──────────────►  /map (live, growing) + TF: map→odom
          │
          ▼
  frontier_explorer (Part 3)  ──►  /goal_pose  (autonomously chosen)
          │
          ▼
  global_planner + potential_field_planner (Part 1)  ──►  /cmd_vel
```

---

## Running on the Real Robot (Robile4)

```bash
# 1. Power on the robot, connect to "Robile5G" WiFi
ssh -x studentkelo@192.168.0.104          # password: area5142
tmux new -s robot_session
ros2 launch robile_bringup robot.launch.py   # leave running

# 2. On your laptop, in every new terminal:
export ROS_DOMAIN_ID=4

# --- For Part 1 + 2 (known map) ---
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:=<path_to_map>.yaml -p use_sim_time:=false
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
ros2 launch robile_localization localization.launch.py
# RViz: "2D Pose Estimate" to seed the particle filter
ros2 launch robile_planning planning.launch.py
# RViz: "2D Goal Pose" to send a goal

# --- For Part 3 (unknown/new area) ---
ros2 launch robile_navigation online_async.launch.py   # NOT particle_filter
ros2 launch robile_planning planning.launch.py
ros2 launch robile_exploration exploration.launch.py
```

## Running in Simulation (Gazebo)

Same as above, but:
- Launch `ros2 launch robile_gazebo gazebo_4_wheel.launch.py` instead of
  SSH-ing into hardware.
- Use `use_sim_time:=true` everywhere.
- **The map must be built via SLAM inside the exact same Gazebo world** you
  intend to test in — reusing a map from a different/mismatched world
  causes the particle filter to diverge.

---

## Troubleshooting Log (lessons learned)

These issues came up during development and are recorded here to save time
if they recur:

- **RobotModel shows as a small white box / "No transform from [base_link]
  to [map]"** — the TF chain `map → odom → base_link` is broken somewhere.
  Check each link with `ros2 run tf2_ros tf2_echo <parent> <child>`. Common
  causes: `platform_driver` (or the sim's odometry plugin) not actually
  publishing `/odom`/TF, or Part 2's particle filter not running.
- **`platform_driver` node running but not actually working** — check
  `ros2 node info /platform_driver`; if it has no `/cmd_vel` subscription
  or `/odom` publication, the hardware driver didn't initialise properly.
  Restart `robile_bringup`, and check for a pressed emergency-stop button
  or loose motor cabling.
- **A\* "start cell is occupied/inflated" / fails to find a path** — the
  robot's TF-derived position doesn't fall in free space on the map. Causes:
  particle filter hasn't converged yet, `inflation_radius_cells` too large
  for a narrow corridor, or the map/environment don't actually match.
- **Particle filter diverges (estimate ends up outside the map)** — almost
  always a map/environment mismatch (e.g. testing in a different Gazebo
  world, or with walls/obstacles removed) so the laser scan can never match
  the map, causing degenerate weights and effectively random resampling.
- **`map_server` shows "Node not found" after being active before** — it
  was accidentally Ctrl+C'd. It must stay running for the whole session, in
  its own terminal.
- **Map YAML `image:` field doesn't match after renaming the `.pgm` file**
  — `map_saver_cli` writes the original filename into the `.yaml`; renaming
  only the `.pgm` breaks the reference. Edit the `image:` line to match.
- **`bash: install/setup.bash: No such file or directory`** — always
  `source` with the full path (`~/amr_final_ws/install/setup.bash`), not a
  relative one, regardless of your current directory.

---

## Status

- ✅ Part 1 (A* + potential field planner) — implemented and tested in
  both Gazebo (matched map) and on the real robot.
- ✅ Part 2 (particle filter / MCL) — implemented, TF-integrated with
  Part 1; convergence verified when map and environment match. Kidnapped
  robot recovery intentionally not implemented (optional per spec).
- ✅ Part 3 (frontier-based exploration + SLAM) — implemented; runs
  alongside Part 1 to autonomously build a map of unknown space.

## Possible Future Work

- Adaptive MCL / kidnapped-robot recovery via random particle injection.
- Alternative frontier selection strategies (largest frontier, information
  gain) instead of nearest-frontier.
- SLAM map continuation across sessions via slam_toolbox serialization.