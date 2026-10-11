#/usr/bin/env sh

# Ensure conda libs take precedence over system libs
export LD_LIBRARY_PATH=$CONDA_PREFIX/lib:$LD_LIBRARY_PATH

ROS_ENV_FILE="scripts/personal_ros_env.sh"

if [ -f "$ROS_ENV_FILE" ]; then
    echo "$ROS_ENV_FILE already exists. Sourcing it..."
    . "$ROS_ENV_FILE"
fi

# Defaults for whatever the personal env file did not set. Deliberately NOT gated on
# $ROS_DISTRO: in the *default* pixi environment ROS_DISTRO is exported by a conda
# activation script that runs after this one, so the old guard silently skipped this
# block. `pixi run robot` (default env) then sent `export ROS_DOMAIN_ID=` over ssh and
# the jetson cameras came up on domain 0 while every PC node ran on domain 5.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-5}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}" #rmw_cyclonedds_cpp

# Fast DDS profile: whitelist loopback + camera link so the multi-homed PC does
# not announce unreachable locators (docker0 etc.) that blackhole Jetson->PC data
FASTDDS_PC_PROFILE="${PIXI_PROJECT_ROOT:-$(pwd)}/config/fastdds_pc.xml"
if [ -f "$FASTDDS_PC_PROFILE" ]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="$FASTDDS_PC_PROFILE"
fi

WORKSPACE_SETUP="${PIXI_PROJECT_ROOT:-$(pwd)}/install/setup.bash"
if [ -f "$WORKSPACE_SETUP" ]; then
    . "$WORKSPACE_SETUP"
fi

ros2 daemon stop
ros2 daemon start
