#!/bin/bash
# Run the pi0.5 policy server or client.
#
#     scripts/run_policy.sh server
#     scripts/run_policy.sh client [config]        # default: robi_client_v2.yaml
#
# The two halves live in DIFFERENT environments and that is not accidental:
#   server -> micromamba `lerobot` env: needs lerobot + torch, has no rclpy
#   client -> pixi `humble` env:        needs rclpy + the built franka_custom_msgs,
#                                       has no torch
# They talk over gRPC on 127.0.0.1:8080, so run them in two terminals.
#
# NB run_client.sh is stale (points at franka_client and does not wrap the python
# call in `pixi run`); use this instead.
set -euo pipefail
cd "$(dirname "$0")/.."
ROLE=${1:-}
case "$ROLE" in
  server)
    exec env PYTHONPATH="$PWD/src/franka_policy" \
      /home/lewerkzeugkasten/micromamba/envs/lerobot/bin/python \
      src/franka_policy/franka_policy/server/policy_server.py
    ;;
  client)
    CFG=${2:-config/robi_client_v2.yaml}
    # Path is relative to src/franka_policy unless absolute.
    [[ "$CFG" = /* ]] || CFG="$PWD/src/franka_policy/$CFG"
    [ -f "$CFG" ] || { echo "config not found: $CFG" >&2; exit 1; }
    # PYTHONPATH must be APPENDED: set_ros_env.sh sources install/setup.bash,
    # which is what puts franka_custom_msgs on the path. Overwriting it gives
    # "ModuleNotFoundError: No module named 'franka_custom_msgs'".
    exec pixi run -e humble bash -c "
      . scripts/set_ros_env.sh
      export PYTHONPATH=\"\$PYTHONPATH:$PWD/src/franka_policy:$PWD/src/franka_policy/franka_policy/client\"
      python src/franka_policy/franka_policy/client/client.py '$CFG'"
    ;;
  *) echo "usage: $0 {server|client} [config]" >&2; exit 2 ;;
esac
