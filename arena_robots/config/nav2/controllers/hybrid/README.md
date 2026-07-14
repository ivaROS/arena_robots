# Hybrid local planner (Phase 1: heuristic switch)

A single `controller_server` that loads **two** nav2 controllers and switches between them at
runtime based on the situation:

| id | plugin | role |
|----|--------|------|
| `DynamicGap` | `dynamic_gap::DynamicGapController` | default: geometric egocircle gap follower |
| `SICNav` | `nav2py_sicnav_controller::SicnavController` | crowd mode: interactive-crowd MPC (CasADi/IPOPT) |

This is the substrate for a hybrid planner. Phase 1 drives the switch with a hand-coded rule;
**Phase 2** will drop in a trained RL policy that publishes to the *same* topic — no substrate
change required.

## How it fits together

```
 controller_switch node ──"DynamicGap"/"SICNav"──▶ <robot_ns>/controller_selector (std_msgs/String, latched)
                                                          │
                                             bt_navigator: ControllerSelector BT node
                                                          │ writes {selected_controller}
                                                          ▼
                                             FollowPath controller_id="{selected_controller}"
                                                          │
                                             controller_server runs DynamicGap OR SICNav
```

- **`controllers/hybrid/controller_config.yaml`** — registers both plugins under keys
  `DynamicGap` / `SICNav` (merged into `controller_server` via the `${**controller_plugins_dict}`
  splat). Only the selected controller's `computeVelocityCommands()` is called each tick.
- **`interplanners/hybrid/interplanner_behavior.xml`** — the Arena default BT plus a
  `ControllerSelector` node; `FollowPath` uses `controller_id="{selected_controller}"`.
- **`controller_switch` node** (`arena_robots/controller_switch.py`, launched automatically in
  hybrid mode) — thresholds nearest-pedestrian distance (from `../arena_peds` + TF) and publishes
  the active controller id with hysteresis (enter `< 2.0 m` → `SICNav`, exit `> 3.0 m` → back to
  `DynamicGap`).

## Build & run

```bash
# 1. Rebuild the package (installs the new config/BT/node/script)
arena build arena_robots

# 2. Launch a sim with the hybrid planner. BOTH args are required and paired:
arena launch <your usual sim args> \
    mobile:=nav2 mobile.local_planner:=hybrid mobile.inter_planner:=hybrid
```

`mobile.local_planner:=hybrid` loads both controllers; `mobile.inter_planner:=hybrid` loads the
BT that can switch between them. Use a world with pedestrians (e.g. a hunav scenario) so the
switch actually triggers. SICNav requires IPOPT in the container (see the SICNav port notes).

## Verify it's working

```bash
# The selector topic exists and carries the active controller id (confirm the exact name/ns):
ros2 topic list | grep controller_selector
ros2 topic echo <robot_ns>/controller_selector          # flips DynamicGap <-> SICNav as peds approach

# The switch node logs each transition:
#   [controller_switch] switch DynamicGap -> SICNav (nearest ped 1.83 m)

# Both controllers are loaded:
ros2 param get /<robot_ns>/controller_server controller_plugins   # -> [DynamicGap, SICNav]
```

If `ControllerSelector` subscribes to a differently-scoped topic than the switch node publishes
(check `ros2 topic info` on both ends), override the node's `selector_topic` parameter — no code
change needed.

## Tuning

Switch thresholds are `Node` parameters set in `launch/adapters/mobile/nav2.launch.py`
(`enter_distance`, `exit_distance`, `default_controller`, `crowded_controller`, `peds_timeout_s`,
`publish_period_s`). SICNav's own params (`max_speed`, `neighbor_dist`, `time_horizon`, …) are set
in `controllers/hybrid/controller_config.yaml` under the `SICNav:` key. Because SICNav is a slower
MPC that plans anticipatorily, you may want to engage it a little earlier (larger `enter_distance`)
than you would a reactive planner.

## Known caveats (Phase 1)

1. **SICNav perceives crowds from the laser scan, not `arena_peds`.** The nav2py bridge clusters
   the scan into agents (`process_laserscan` in `nav2py_sicnav_controller/__main__.py`) and feeds
   those as human states to the MPC (with a 2 s motion extrapolation). This is genuine model-based
   crowd avoidance, but on *detected* agents — realistic, non-privileged perception. To use
   ground-truth tracks instead, subscribe to `arena_peds` in the bridge and build `human_states`
   from each pedestrian's `pose`/`twist` (a direct FullState mapping), replacing the
   `process_laserscan` output. That's a change in the `Sicnav` package, not here.
2. **SICNav is an IPOPT MPC — solve latency is non-trivial.** Its `computeVelocityCommands()`
   blocks on the Python child's solve, so when SICNav is selected the control loop runs at the
   solver's rate, which may be below `controller_frequency` (10 Hz). Fine for a single-robot
   prototype; it does not affect DynamicGap ticks.
3. **The SICNav Python child (CasADi/IPOPT) is spawned at `configure()`** whether or not SICNav is
   ever selected — expect one always-on idle process and a heavier `controller_server` startup.
4. **`odom_topic` is absolute/jackal-specific** in the config (mirrors the standalone SICNav
   config, whose C++ default is hardcoded to the jackal namespace). Change it for other robots.
5. **Arbitration, not blending.** Exactly one controller drives per tick; a switch can produce a
   small velocity discontinuity (the downstream velocity_smoother absorbs most of it).
```
