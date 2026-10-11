## 1. Collection: nothing is computed, everything is logged raw

[data_collector_node.py](vscode-webview://1710if58gbam554r33s9emcjqfr16pt2q37pag02is8fhlpq32mp/pixi_franka_crisp_ws/src/franka_py/franka_data_collection/franka_data_collection/data_collector_node.py) is a thin wrapper around `ros2 bag record`. It shells out to record a fixed topic list ([data_collector.yaml](vscode-webview://1710if58gbam554r33s9emcjqfr16pt2q37pag02is8fhlpq32mp/pixi_franka_crisp_ws/src/franka_py/franka_data_collection/config/data_collector.yaml)) into an MCAP bag, and separately encodes the ZED `RigSnapshot` stream into per-camera mp4s. Per arm it captures:

- `franka/joint_states` — 7 joint positions + velocities
- `franka_gripper/joint_states` — gripper width
- `fijk_debug` — the IK layer's full state dump
- `gripper/.../joint_trajectory` — the commanded gripper
- `mq_debug` — carries `movement_enabled`, the teleop-active flag

So: **no EE poses are "collected" as a design choice** — they arrive as a side effect of logging `FIJKDebug`, which carries six different poses per message ([FIJKDebug.msg](vscode-webview://1710if58gbam554r33s9emcjqfr16pt2q37pag02is8fhlpq32mp/pixi_franka_crisp_ws/src/franka_py/franka_custom_msgs/msg/FIJKDebug.msg)):

|field|meaning|
|---|---|
|`safe_target_pose`|what the operator commanded, after the safety layer clamped it ([franka_controller_node.cpp:378](vscode-webview://1710if58gbam554r33s9emcjqfr16pt2q37pag02is8fhlpq32mp/pixi_franka_crisp_ws/src/robot_safety_layer/src/franka_controller_node.cpp#L378))|
|`target_pose`|the raw target the IK node received|
|`actual_pose`|measured — forward kinematics of `q_`|
|`cmd_pose`|the IK layer's internal integrated command|
|`cmd_final_pose`, `tf_pose`|further internals|

All of these are **absolute world-frame poses**. The delta/absolute decision is made later, at conversion time, purely by which of these fields the converter reads.

