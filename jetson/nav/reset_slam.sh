#!/bin/bash
PID=$(pgrep -f async_slam_toolbox_node)
if [ -n "$PID" ]; then
  echo "Killing slam_toolbox (PID $PID)..."
  kill $PID
  sleep 2
fi

echo "Launching fresh slam_toolbox..."
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=false \
  slam_params_file:=/home/nvidia/ros2_ws/src/delivery_robot/config/mapper_params_medbot.yaml
