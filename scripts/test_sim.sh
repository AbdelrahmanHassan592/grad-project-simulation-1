#!/usr/bin/env bash
set -euo pipefail
# Run against a live simulation; intervals are in s, linear velocity m/s, yaw rad.
exec ros2 run my_car_description test_sim "$@"
