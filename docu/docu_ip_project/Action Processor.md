[action_processors.py](vscode-webview://1710if58gbam554r33s9emcjqfr16pt2q37pag02is8fhlpq32mp/pixi_franka_crisp_ws/src/franka_policy/franka_policy/client/action_processors.py) is the **decoder layer** between the policy and ROS. The policy (or a replayed dataset) emits one flat float vector per timestep — for the bimanual setup, 20 numbers:

```
[0:9]   right arm pose   (3 position + 6D rotation)
[9:10]  right gripper
[10:19] left arm pose
[19:20] left gripper
```

Nothing in that vector says what it _means_. The processors carve it into slices and turn each slice into a real ROS message aimed at a real endpoint. Which slice goes where is pure config — the `actions:` list in [replay_client.yaml:22-46](vscode-webview://1710if58gbam554r33s9emcjqfr16pt2q37pag02is8fhlpq32mp/pixi_franka_crisp_ws/src/franka_policy/config/replay_client.yaml#L22-L46) gives each entry a `type`, `start_idx`/`end_idx`, and a destination.

## The mechanics

A registry decorator (`register_action_processor`) maps a YAML `type` string to a class, so the client just does `ACTION_PROCESSORS[config["type"]](**config)` ([replay_client.py:137](vscode-webview://1710if58gbam554r33s9emcjqfr16pt2q37pag02is8fhlpq32mp/pixi_franka_crisp_ws/src/franka_policy/franka_policy/client/replay_client.py#L137)). Two class attributes on [BaseActionProcessor](vscode-webview://1710if58gbam554r33s9emcjqfr16pt2q37pag02is8fhlpq32mp/pixi_franka_crisp_ws/src/franka_policy/franka_policy/client/action_processors.py#L21-L37) tell the client how to wire it up:

- `sink` — `"topic"` publishes the returned message; `"service"` calls `SetPoseStamped` on `processor.service` (the safety node's route, the same one homing and teleop use).
- `needs_anchor` — if true, the client subscribes to `fijk_debug` and passes the live robot pose into `process()` as a third argument.

Then per frame, [execute_actions](vscode-webview://1710if58gbam554r33s9emcjqfr16pt2q37pag02is8fhlpq32mp/pixi_franka_crisp_ws/src/franka_policy/franka_policy/client/replay_client.py#L248-L274) loops over `(sink, processor)` pairs, calls `process()`, and drops the frame if it returns `None`.

## The four processors

**`PoseWorldAbsolute6D`** — what your replay config actually uses. Takes the 9-element slice as an absolute world pose, decodes the 6D rotation to a quaternion, sends it as `PoseStamped` to `/franka_<side>/target_pose`. No anchor, no accumulation: a dropped or clamped frame costs exactly one frame of motion, because the next frame re-specifies the full pose.

**`PoseAbsolute6D`** — the closed-loop delta variant. Same slice layout, but treated as an _increment_: it adds `d_pos` to the live anchor pose and left-multiplies the rotation (`R_delta * R_anchor`), inverting exactly how `DeltaCartesianGenerator` built the label. It refuses to act if there's no anchor yet, and the client refuses anchors older than `anchor_timeout` (0.1 s default) — commanding relative to a pose the arm has already left is worse than skipping. This is the z-sag fix in your memory: anchor on `actual_pose`, not `cmd_pose`.

**`PoseDelta6D`** — the legacy path, and the one the docstrings warn about. It publishes a `Twist` on `/delta_pose`, which `franka_ijk` reads as a Cartesian _velocity_ scaled by `velocity_gain`. There is no pose anywhere on that path, so the label's anchor has no counterpart at replay time and every per-frame bias integrates into permanent drift. It's also the only processor still carrying debug `print()` calls (lines 68-69, 84) — those fire every control tick.

**`GripperAbsolute`** — the 1-element slice, scaled by `scale_factor` (1.1 in your config), wrapped in a single-point `JointTrajectory` with `time_from_start` set to one control period (1/frequency).

**`DeltaJoint`** — packs raw joint deltas into a `JointState.position` field for a C++ node to consume. Note it takes the _unsliced_ convention differently from the others: it slices `action_slice[start:end]` from what it names `action_slice`, whereas the pose processors name the same argument `action_array`. Same behaviour, inconsistent naming.

## The rotation decode

All three pose processors share the same 6D→matrix step — Gram-Schmidt on two 3-vectors:

```python
x = r6d[0:3] / ‖r6d[0:3]‖
y = r6d[3:6] − (r6d[3:6]·x) x   then normalized
z = x × y
```

This is Zhou et al.'s continuous rotation representation: 6D avoids the discontinuities that make quaternions and Euler angles bad regression targets for a network. The code is duplicated verbatim in three places, which is the obvious candidate if you ever refactor this file.