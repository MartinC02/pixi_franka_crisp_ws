## The topology: safety wraps IK on both sides

The thing that isn't obvious from the file names — this is a **loop, not a pipeline**. The safety node is both the entry point and the exit point; the IK node sits inside it as a pure velocity solver:

```
teleop / policy client
        │  SetPoseStamped service  /<ns>/target_pose
        ▼
┌─ SAFETY NODE (franka_controller_node) ──────────────┐
│  adjustToSafePose()  ──►  publishes safe_target_pose│
└──────────────────────────────┬──────────────────────┘
                               ▼
┌─ IK NODE (franka_ijk) ──────────────────────────────┐
│  interpolate → Cartesian error → DLS + nullspace    │
│  publishes  target_dq  (joint velocities)           │
└──────────────────────────────┬──────────────────────┘
                               ▼
┌─ SAFETY NODE (same node, control loop) ─────────────┐
│  arbiter → Cartesian vel limit → integrate q_cmd_   │
│  → spring clamp → publishes target_joint            │
└──────────────────────────────┬──────────────────────┘
                               ▼
       crisp_controllers cartesian_controller (ros2_control, 1 kHz)
```

So a Cartesian target makes a full round trip through both nodes before a joint command exists. `safe_target_pose` — the field every dataset action is built from — is emitted at _step one_, before the IK node has seen anything.

## Three entry paths, two arbiters

|path|topic/service|enters at|preempts|
|---|---|---|---|
|**Pose**|`target_pose` (service)|safety node|policy, and delta_pose|
|**Cartesian velocity**|`delta_pose` (Twist)|**IK node directly**|pose tracking|
|**Policy joint velocity**|`policy_target_dq`|**safety node directly**|IK|

The `delta_pose` path is worth pausing on: it goes straight into [deltaPoseCallback](vscode-webview://1710if58gbam554r33s9emcjqfr16pt2q37pag02is8fhlpq32mp/pixi_franka_crisp_ws/src/robot_ik_layer/src/franka_ijk.cpp#L162-L187) and **never touches `adjustToSafePose`**. It's scaled by `velocity_gain` (5.0), clamped to `cartesian_velocity_limit`, and fed to `runJacobianNullspaceControl`. The pose-level obstacle avoidance simply doesn't apply to it — only the downstream dq limits do. That's the path `robi_client.yaml` (V1) uses.

Both arbiters are watchdog-based, both at **0.25 s**:

- **Safety node** ([controlLoop:416-431](vscode-webview://1710if58gbam554r33s9emcjqfr16pt2q37pag02is8fhlpq32mp/pixi_franka_crisp_ws/src/robot_safety_layer/src/franka_controller_node.cpp#L416-L431)): `policy_target_dq_` wins while fresh, else falls back to `target_dq_ik_`. A `target_pose` service call also sets `is_policy_active_ = false` immediately — a Cartesian target always beats a running policy.
- **IK node** ([controlLoop:324-338](vscode-webview://1710if58gbam554r33s9emcjqfr16pt2q37pag02is8fhlpq32mp/pixi_franka_crisp_ws/src/robot_ik_layer/src/franka_ijk.cpp#L324-L338)): `delta_pose` velocity wins while fresh; on timeout it freezes `target_se3_` at current FK so it doesn't lurch to a stale target. A `safe_target_pose` arrival preempts velocity mode.

## Four safety mechanisms, applied in order

1. **`adjustToSafePose`** — pose-level, slides the desired pose away from obstacles. Runs in the service callback, against `q_cmd_` FK (not measured).
2. **`getMaxSafeVelocity`** — scales the whole `dq` so EE Cartesian speed respects the obstacle distance along the velocity direction.
3. **`joint_velocity_limit`** (1.0 rad/s) — applied to policy dq on arrival, and to IK dq in `publishDq`.
4. **`max_spring_stretch`** (0.2 rad) — per-joint leash: `q_cmd_` can never drift more than this from measured `q_`.

`bypass_safety` disables **only 1 and 2**. The velocity limit and the spring clamp always run — which is why a nonsense Cartesian target produces a slow creep rather than a lurch.

Correcting my previous message: `max_spring_stretch` is **0.2** rad, not 0.1 — [safety_params.yaml](vscode-webview://1710if58gbam554r33s9emcjqfr16pt2q37pag02is8fhlpq32mp/pixi_franka_crisp_ws/src/robot_safety_layer/config/safety_params.yaml) overrides the node's 0.1 default, and [start_ijk.launch.py](vscode-webview://1710if58gbam554r33s9emcjqfr16pt2q37pag02is8fhlpq32mp/pixi_franka_crisp_ws/src/robot_ik_layer/launch/start_ijk.launch.py#L41-L48) loads that file into both nodes. Same for `dls_lambda`: 0.01 deployed, 0.0001 in the code default.

## The IK control law

Per 200 Hz tick, when tracking a pose:

1. **Interpolate** — `interpolation_factor = dt / target_dt` = 0.005/0.02 = **0.25**. It's a first-order filter, not a bounded ramp: each tick closes 25% of the remaining gap (lerp on translation, `log3`/`exp3` slerp on rotation). ~68% of a step lands within 20 ms, and it never fully arrives. This is why a target that jumps still produces a smooth command.
2. **Error → velocity** — world-frame translation error, and angular error computed in the EE frame then rotated to world. Both scaled by `velocity_gain` = 5.0, so it's a P controller with a 5 s⁻¹ gain.
3. **Clamp** to `cartesian_velocity_limit` = 1.0, with a 1e-3 deadband that zeroes small commands.
4. **DLS pseudo-inverse** — `J⁺ = Jᵀ(JJᵀ + λI)⁻¹`, λ = 0.01. Fairly heavy damping; it trades tracking accuracy for stability near singularities.
5. **Nullspace posture** — `dq_null = N · K_NULL · (q_init − q)`, K_NULL = 0.1, pulling the elbow back toward the launch configuration without disturbing the EE.

Note the IK node solves against **measured `q_`**, while the safety node integrates and clamps **`q_cmd_`**. That split is exactly the `actual_pose` vs `cmd_pose` gap behind the z-sag: `cmd_pose` rides above `actual_pose` because the arm sags under gravity, and `FIJKDebug` reports both.

## Frequencies

|stage|rate|source|
|---|---|---|
|ros2_control arm loop|**1000 Hz**|[controllers.yaml:4](vscode-webview://1710if58gbam554r33s9emcjqfr16pt2q37pag02is8fhlpq32mp/pixi_franka_crisp_ws/src/franka_py/franka_launch/config/controllers.yaml#L4)|
|`franka/joint_states` broadcast|**1000 Hz**|`state_publish_rate: 1000.0`|
|Safety node control loop|**200 Hz**|`timer_dt: 0.005`|
|IK node control loop|**200 Hz**|`timer_dt: 0.005`|
|IK target interpolation constant|50 Hz equiv.|`target_dt: 0.02`|
|Dynamixel gripper (`franka_gripper/joint_states`)|**60 Hz**|[dynamixel_controllers.yaml:7](vscode-webview://1710if58gbam554r33s9emcjqfr16pt2q37pag02is8fhlpq32mp/pixi_franka_crisp_ws/src/franka_py/franka_launch/config/dynamixel_controllers.yaml#L7)|
|Quest teleop ROS timer|**50 Hz**|[main.py:166](vscode-webview://1710if58gbam554r33s9emcjqfr16pt2q37pag02is8fhlpq32mp/pixi_franka_crisp_ws/src/franka_py/franka_meta_quest/franka_meta_quest/oculus_action/main.py#L166)|
|Quest reader thread|**50 Hz**|`_update_internal_state(hz=50)`|
|Policy action apply|**50 Hz**|`frequency` in robi_client_v2|
|Policy inference / chunk request|**3 Hz**|`policy_inference_fps` (measured 0.296 s)|
|ZED cameras|**30 fps**|`camera_fps`|
|Dataset grid|**50 fps**|converter `fps`|
|Policy watchdog|0.25 s|safety node|
|`delta_pose` watchdog|0.25 s|`delta_pose_timeout_`|

## What gets logged

`fijk_debug` is published by the **safety node**, not the IK node, once per 200 Hz tick from [publishDebugInfos](vscode-webview://1710if58gbam554r33s9emcjqfr16pt2q37pag02is8fhlpq32mp/pixi_franka_crisp_ws/src/robot_safety_layer/src/franka_controller_node.cpp#L497-L541):

|field|what it actually is|
|---|---|
|`actual_pose`|FK of measured `q_`|
|`cmd_pose` / `cmd_final_pose`|FK of `q_cmd_` — **identical**, both computed from the same SE3|
|`target_pose`|cached raw pose from the last service call|
|`safe_target_pose`|same call after `adjustToSafePose`|
|`tf_pose`|**always identity** — this node has no TF listener|
|`cartesian_velocity`|`J(q_cmd_) · active_dq`|
|`safety_distance`, `..._along_velocity`|obstacle distances|
|`q_actual`, `q_target`, `dq_commanded`|measured, commanded, applied|

Two are latched, not sampled: `target_pose` and `safe_target_pose` hold their last service value and keep republishing it at 200 Hz. During teleop the service fires at 50 Hz, so each value repeats ~4×. During a policy rollout at 3 Hz inference the client still calls at 50 Hz, so that's fine — but if the client ever stalls, `fijk_debug` keeps confidently reporting a stale target.

`mq_debug` (FMQDebug) comes from the teleop node at 50 Hz and carries `movement_enabled` plus the full VR/robot pose chain — that's the gate the converter's `trigger` block uses.

The [data collector](vscode-webview://1710if58gbam554r33s9emcjqfr16pt2q37pag02is8fhlpq32mp/pixi_franka_crisp_ws/src/franka_py/franka_data_collection/config/data_collector.yaml) records six topics per arm into MCAP at their native rates: `franka/joint_states` (1 kHz), `franka_gripper/joint_states` (60 Hz), `mq_debug` (50 Hz), `fijk_debug` (200 Hz), the gripper `joint_trajectory` (50 Hz, **only while that arm's teleop is enabled** — the gap that forced `interpolation: previous`), and `internal/target_pose`. Video goes to separate mp4s at 30 fps with a `timestamps_<cam>.npy` sidecar.

## Two stale comments worth cleaning

[main.py:453](vscode-webview://1710if58gbam554r33s9emcjqfr16pt2q37pag02is8fhlpq32mp/pixi_franka_crisp_ws/src/franka_py/franka_meta_quest/franka_meta_quest/oculus_action/main.py#L453) says `# ~66ms duration for trajectory execution to match 15Hz loop` above `nanosec = 20_000_000` — that's 20 ms, and the loop is 50 Hz. The value is right, the comment is two refactors old. [main.py:573](vscode-webview://1710if58gbam554r33s9emcjqfr16pt2q37pag02is8fhlpq32mp/pixi_franka_crisp_ws/src/franka_py/franka_meta_quest/franka_meta_quest/oculus_action/main.py#L573) has the same "natively at 15Hz" claim. Anyone reasoning about gripper timing from those comments would be off by 3×.

Want me to publish this as a page with the control-flow diagram drawn properly? It'd be easier to hand to someone than terminal scrollback.