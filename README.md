# AMR Project — Robile4/Robile3 dular mobile robot platform, developed as part of
the Autonomous Mobile Robots course project.

## Project Objectives

1. **Path & Motion Planning** — port a potential field planner to the real
   robot, and combine it with a global path planner (A*) so it can navigate
   large, mapped environments.
2. **Localisation** — implement a Monte Carlo Localisation (particle filter)
   from scratch to estimate the robot's pose on a known map.
3. **Environment Exploration** *(not yet started)* — combine frontier-based
   pose selection with SLAM so the robot can autonomously map unknown areas.

This repository currently covers **Part 1 and Part 2**.

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

---

## Part 1: Path & Motion Planning

**Package:** `robile_planning`

### Nodes

- **`global_planner`** — runs A* search over an inflated occupancy grid
  (`/map`), from the robot's current pose to a given goal (`/goal_pose`).
  The raw cell-by-cell path is simplified down to waypoints kept only at
  direction changes, and published on `/global_path`.
- **`potential_field_planner`** — subscribes to `/global_path` and tracks an
  index into the waypoint list. At each control step it computes:
  - an **attractive force** toward the current target waypoint,
  - a **repulsive force** from any laser scan point (`/scan`) inside an
    influence radius,

  sums them, and converts the resultant force vector into a `/cmd_vel`
  command. When the robot is within `waypoint_tolerance` of a waypoint it
  advances to the next one; reaching the last one stops the robot and logs
  `🎯 Reached goal!`.

### Key topics

| Topic | Type | Direction |
|---|---|---|
| `/map` | `nav_msgs/OccupancyGrid` | in (global_planner) |
| `/goal_pose` | `geometry_msgs/PoseStamped` | in (global_planner) — e.g. RViz "2D Goal Pose" |
| `/global_path` | `nav_msgs/Path` | out (global_planner) / in (potential_field_planner) |
| `/scan` | `sensor_msgs/LaserScan` | in (potential_field_planner) |
| `/cmd_vel` | `geometry_msgs/Twist` | out (potential_field_planner) |

### Pose source

Both nodes read the robot's pose via the `map → base_link` TF transform
(param `use_tf_pose: true`), which is provided by the Part 2 particle filter.
If that transform isn't available, they fall back to raw `/odom`.

### Run it

```bash
ros2 launch robile_planning planning.launch.py
```

Then send a goal via RViz's **"2D Goal Pose"** tool, or:
```bash
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
  '{header: {frame_id: "map"}, pose: {position: {x: 2.0, y: 1.0}}}'
```

### Tunable parameters

See `config/planning_params.yaml` — key ones: `attractive_gain`,
`repulsive_gain`, `obstacle_influence_radius`, `inflation_radius_cells`.

---

## Part 2: Localisation (Monte Carlo Localisation)

**Package:** `robile_localization`

### Node: `particle_filter`

A from-scratch implementation of the sequential importance resampling
particle filter (the "simple version" of MCL), consisting of:

1. **Initialise** — particles scattered around an initial pose guess, given
   via RViz's **"2D Pose Estimate"** tool (topic `/initialpose`).
2. **Predict** — on each `/odom` update, every particle is moved by the same
   odometry delta plus noise, using the standard odometry motion model
   (`alpha1`–`alpha4` noise parameters).
3. **Update** — on each `/scan` message, particles are weighted using a
   likelihood-field measurement model: a distance-to-nearest-obstacle grid is
   precomputed once from the map (multi-source BFS), and each particle's
   weight reflects how well its predicted laser hits align with that field.
4. **Resample** — low-variance resampling, proportional to weight.
5. **Publish** — the weighted mean pose is broadcast as the `map → odom` TF
   transform (replacing the need for AMCL or a static placeholder
   transform), plus:
   - `/particle_cloud` (`geometry_msgs/PoseArray`) — visualise in RViz with
     a **PoseArray** display.
   - `/pf_pose` (`geometry_msgs/PoseWithCovarianceStamped`) — the pose
     estimate for other nodes to consume.

### Run it

```bash
ros2 launch robile_localization localization.launch.py
```

In RViz:
1. Add a **PoseArray** display on `/particle_cloud`.
2. Use **"2D Pose Estimate"** to seed the initial pose.
3. Move the robot — the particle cloud should converge (tighten) around the
   robot's true position as laser updates arrive.

### Tunable parameters

See `config/localization_params.yaml` — key ones: `num_particles`,
`laser_subsample` (performance vs. accuracy trade-off), `sigma_hit`,
`z_hit`/`z_rand`.

### Known limitation

Like the basic MCL taught in the lecture, this implementation cannot recover
from the "kidnapped robot" problem (no random-particle re-injection). This is
a natural direction for the optional adaptive MCL extension mentioned in the
assignment.

---

## Full Pipeline: Map → Localisation → Planning

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

**Important:** the map used must match the actual environment the robot is
driving in. A map built in one Gazebo world (or the real corridor) will
cause the particle filter to diverge if tested against a *different*
environment — laser scans won't match the map's obstacles, weights become
degenerate, and the pose estimate drifts outside the map bounds.

---

## Running on the Real Robot (Robile4)

```bash
# 1. Power on the robot, connect to "Robile5G" WiFi
ssh -x studentkelo@192.168.0.104          # password: area5142
tmux new -s robot_session
ros2 launch robile_bringup robot.launch.py   # leave running

# 2. On your laptop, in every new terminal:
export ROS_DOMAIN_ID=4

# 3. Serve the map
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:=<path_to_map>.yaml -p use_sim_time:=false
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate

# 4. Localise
ros2 launch robile_localization localization.launch.py
# In RViz: "2D Pose Estimate" to seed the particle filter

# 5. Plan and navigate
ros2 launch robile_planning planning.launch.py
# In RViz: "2D Goal Pose" to send a goal
```

## Running in Simulation (Gazebo)

Same as above, but:
- Launch `ros2 launch robile_gazebo gazebo_4_wheel.launch.py` instead of SSH-ing
  into hardware.
- Use `use_sim_time:=true` for `map_server`.
- **The map must be built via SLAM (`robile_navigation`'s `online_async.launch.py`)
  inside the exact same Gazebo world** you intend to test in — reusing the real
  robot's map in a mismatched simulated world will cause the particle filter
  to diverge.

---

## Status

- ✅ Part 1 (A* + potential field planner) — implemented, tested in both
  Gazebo (matched map) and on the real robot in the lab corridor.
- ✅ Part 2 (particle filter / MCL) — implemented; convergence verified when
  map and environment match; TF-integrated with Part 1.
- ⬜ Part 3 (frontier-based exploration + SLAM) — not started.

## Next Steps

- Build Part 3: frontier detection + autonomous exploration combined with
  `slam_toolbox`.
- Optional: tune particle filter noise parameters against real robot data;
  explore adaptive MCL (dynamic particle count).