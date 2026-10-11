#!/bin/bash
# The jetson has to join the same DDS domain as the PC nodes -- never ship an empty
# value over ssh (that silently puts the cameras on domain 0).
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-5}"

if ! command -v tmux &> /dev/null
then
    echo "tmux is not installed. Running commands in background instead."
    ros2 launch franka_launch example.launch.py spawn_franka_left:=true spawn_franka_right:=true use_fake_hardware:=false &
    ssh jetson "cd Projects/ros2_ws && source install/setup.bash && export ROS_DOMAIN_ID=$ROS_DOMAIN_ID && bash launch_zed.sh bimanual" &
    ros2 launch zed_rig_aggregator_node aggregator.launch.py &
    ros2 launch robot_ik_layer start_ijk.launch.py spawn_franka_left:=true spawn_franka_right:=true bypass_safety:=true &
    python scripts/view_wrist_cameras.py &
    wait
    exit 0
fi

# Use tmux to open multiple terminals side-by-side
SESSION="robot_run"

# A leftover session from a previous run makes `new-session` fail, but the
# split-window calls below still succeed against the stale session -- you get a
# half-built layout with none of the nodes actually running. Start clean.
tmux kill-session -t $SESSION 2>/dev/null

tmux new-session -d -s $SESSION "pixi run -e humble ros2 launch franka_launch example.launch.py spawn_franka_left:=true spawn_franka_right:=true use_fake_hardware:=false ; exec bash"
tmux split-window -h -t $SESSION "env -i HOME=$HOME USER=$USER /usr/bin/ssh -t jetson 'cd Projects/ros2_ws && source install/setup.bash && export ROS_DOMAIN_ID=$ROS_DOMAIN_ID && bash launch_zed.sh bimanual' ; exec bash" # choose config csil|max|"custom"
tmux split-window -v -t $SESSION "pixi run -e humble ros2 launch zed_rig_aggregator_node aggregator.launch.py ; exec bash"
tmux split-window -v -t $SESSION "pixi run -e humble ros2 launch robot_ik_layer start_ijk.launch.py spawn_franka_left:=true spawn_franka_right:=true bypass_safety:=true ; exec bash"
# Wrist camera preview (own OpenCV window); give the ZED stack a moment to come up first.
# tmux split-window -v -t $SESSION "sleep 5; pixi run -e humble ros2 launch zed_remote_launcher start_zed.launch.py project:=lwk_2"
tmux split-window -v -t $SESSION "sleep 15 ; pixi run -e humble python scripts/view_wrist_cameras.py ; exec bash"
tmux select-layout -t $SESSION tiled

echo "Started robot launch nodes in a tmux session."

# attach-session refuses to nest when this script is itself run from inside tmux
# (the "sessions should be nested with care" message) -- the session is already
# built and running at this point, so just say how to reach it.
if [ -n "$TMUX" ]; then
    echo "Already inside tmux: run 'tmux switch-client -t $SESSION' to view it."
else
    tmux attach-session -t $SESSION
fi
