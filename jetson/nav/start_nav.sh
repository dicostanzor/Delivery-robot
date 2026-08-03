#!/bin/bash
set -m

echo "Bringing up CAN..."
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000 restart-ms 100

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

echo "Launching sensing/localization inputs (VSLAM, scan, motor control, EKF)..."
ros2 launch delivery_robot full_localization.launch.py &
SENSING_PID=$!
sleep 15

echo "Launching Nav2 localization against saved map..."
ros2 launch nav2_bringup localization_launch.py \
  map:=/home/nvidia/facility_map_july27_8pm.yaml \
  use_sim_time:=false &
LOC_PID=$!
sleep 10

echo "Forcing localization lifecycle activation..."
ros2 service call /lifecycle_manager_localization/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{command: 0}"
ros2 param set /amcl base_frame_id base_link

echo "Launching Nav2 navigation stack..."
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=false &
NAV_PID=$!
sleep 10

echo "Forcing navigation lifecycle activation..."
ros2 service call /lifecycle_manager_navigation/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{command: 0}"

echo "Setting initial pose at map origin..."
ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
  header: {frame_id: 'map'},
  pose: {
    pose: {
      position: {x: 0.0, y: 0.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  }
}"

echo ""
echo "=== Stack is up. Checking status... ==="
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /behavior_server

echo ""
echo "=== Press Ctrl+C to shut everything down ==="
wait $SENSING_PID $LOC_PID $NAV_PID
