# dingo_omni

Clearpath **Dingo-Omni** — a holonomic (mecanum, 4-wheel) mobile base — for Arena on
ROS 2 Jazzy / gz-sim (Harmonic). It is the omnidirectional sibling of the diff-drive
`dingo` and is the first genuinely working holonomic robot in this workspace.

## Using it (collaborators)

`dingo_omni` ships in `ivaROS/Arena` (`jazzy`) and is self-contained — no extra mesh or
submodule download (no `arena feature robots add` needed). To pick it up:

```bash
arena pull      # update Arena + submodules  (or: git -C src/Arena pull && git -C src/Arena submodule update --init --recursive)
arena build     # rebuild so the new robot is discovered
```

Then select it with `robot:=dingo_omni`:

```bash
arena launch sim:=gazebo world:=map_empty robot:=dingo_omni       # simulate / teleop
arena launch sim:=gazebo robot:=dingo_omni mobile:=nav2           # navigation
```

For DRL training, use `--robot dingo_omni`. **Continuous `[vx, vy, wz]` training and nav
work out of the box — nothing extra is required.**

### Discrete-action holonomic training (extra step)

Only needed if you enable `action_space.discretization` (e.g. `strategy: robot_defined`).
It relies on a rosnav-rl fix that is **not** in upstream `Arena-Rosnav/rosnav-rl`; the fix
lives on the lab fork `ivaROS/rosnav-rl@jazzy` and is applied automatically by the training
feature, which overrides `deps/rosnav_rl` after submodule init:

```bash
arena feature training install     # or: arena feature training update
```

> ⚠️ `arena pull` / a bare `git submodule update` re-pin `deps/rosnav_rl` to the upstream
> commit and drop the fix. Re-run `arena feature training update` afterwards to re-apply it
> (manual fallback: `git -C src/Arena/arena_training/deps/rosnav_rl fetch ivaros jazzy &&
> git -C src/Arena/arena_training/deps/rosnav_rl checkout FETCH_HEAD`).

## What makes it holonomic

- `caps/mobile.yaml` sets `is_holonomic: true`. Arena's training stack then builds an
  `OmnidirectionalActionSpace` `[vx, vy, wz]` (the `continuous.linear` range bounds both
  `vx` and `vy`) and the cmd_vel publisher emits `linear.y`. CrowdNav / PaS planners
  detect `kinematics == 'holonomic'` and command 2-D `(vx, vy)` velocities.
- The base accepts a full `geometry_msgs/Twist` on `cmd_vel` (forward, **strafe**, yaw).

## Drive design (why it actually strafes)

gz-sim's `MecanumDrive` / ros2_control `mecanum_drive_controller` both rely on
mecanum-roller friction (anisotropic per-wheel `fdir1`), which is fiddly and unreliable
through URDF→SDF conversion. Instead this robot uses a **kinematic** drive:

- `gz::sim::systems::VelocityControl` applies the commanded **body-frame** velocity
  (`vx, vy, wz`) directly to the base every step, so pure sideways motion is guaranteed
  with plain cylinder wheels — no roller modelling.
- `gz::sim::systems::OdometryPublisher` publishes `/model/<name>/odometry` from the base
  pose (planar) for nav / training.
- `gz::sim::systems::PosePublisher` feeds Arena's pose→TF (odom→base_link).

This is the `gazebo_native` pattern (no `control:` block in `model_params.yaml`, like
`WLP311D`): `cmd_vel` is bridged ROS→gz and `odom` gz→ROS in `mappings.yaml`.

Because `VelocityControl` holds the commanded velocity each step (including
`linear.z = 0`), the spawn height must be the **resting** height, not a drop height —
hence `z_offset: 0.05` (= wheel radius), not the `0.37` used by physics-settled robots.

## Files

- `model_params.yaml` — identity, sensors, `z_offset`; no `control:` block (gazebo_native).
- `caps/mobile.yaml` — `is_holonomic: true`, holonomic action space, footprint, laser, nav2.
- `mappings.yaml` — gz⇄ROS bridges incl. `cmd_vel` (ROS→gz) and `odometry` (gz→ROS).
- `urdf/dingo_omni.urdf.xacro` — self-contained body (primitive box chassis + 4 cylinder
  wheels) + shared Arena lidar/imu sensor macros.
- `urdf/dingo_omni.gazebo` — VelocityControl + OdometryPublisher + PosePublisher.

## Notes / limitations

- **Meshes:** the upstream Dingo mesh submodule is access-gated, so the chassis is a
  primitive box. Drop `omni_chassis.dae` into a `meshes/` dir and swap the box visual if
  a prettier model is wanted; it has no functional effect.
- **Wheels don't spin** (the drive is kinematic). Cosmetic only.
- **Action space:** both paths work. Continuous `[vx, vy, wz]` (`is_holonomic: true` →
  `OmnidirectionalActionSpace`) is the default. Discrete primitives are authored in the
  holonomic caps schema `{linear, lateral, angular}` (incl. `strafe_left`/`strafe_right`)
  and work via `action_space.discretization` (e.g. `strategy: robot_defined`). The latter
  required a small fix to the rosnav action-space layer
  (`deps/rosnav_rl/.../cfg/action_spaces.py`): `resolve_discretization` now normalises the
  caps `DiscreteAction` attrs objects with `attrs.asdict` instead of `dict(a)` (which raised
  `TypeError`), and `OmnidirectionalActionSpace.decode` accepts both the caps `linear`/`lateral`
  keys and the grid `linear`/`angular` keys. (That fix was generic — it also unbroke discrete
  training for the diff-drive robots.)
- If a physically-realistic mecanum (spinning wheels, slip) is later required, switch the
  drive to `mecanum_drive_controller` (ros2_control, à la `rbkairos`) plus per-wheel
  mecanum `fdir1` friction — at the cost of strafing reliability.

## Try it

```bash
# launch a sim with this robot, then strafe it sideways:
ros2 topic pub -r 10 /<robot_ns>/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.5, z: 0.0}, angular: {z: 0.0}}'
# the base should translate sideways; /<robot_ns>/odom should track the lateral motion.
```
