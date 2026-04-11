#!/usr/bin/env bash
set -euo pipefail

# Automated map generation for the AWS Small Warehouse world.
# Runs inside osrf/ros:humble-simulation Docker image on a Buildkite agent.
# Produces warehouse.pgm + warehouse.yaml as artifacts.

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

echo "--- Installing dependencies"
apt-get update -qq
apt-get install -y --no-install-recommends \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-turtlebot3-gazebo \
  ros-humble-turtlebot3-description \
  ros-humble-aws-robomaker-small-warehouse-world \
  ros-humble-slam-toolbox \
  ros-humble-nav2-simple-commander \
  ros-humble-robot-state-publisher \
  xvfb > /dev/null 2>&1

source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_DATABASE_URI=""

PKG_TB3=$(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo
PKG_WAREHOUSE=$(ros2 pkg prefix aws_robomaker_small_warehouse_world)/share/aws_robomaker_small_warehouse_world

export GAZEBO_MODEL_PATH="${PKG_TB3}/models:${PKG_WAREHOUSE}/models:${PKG_WAREHOUSE}:/usr/share/gazebo-11/models:${GAZEBO_MODEL_PATH:-}"

WORLD="${PKG_WAREHOUSE}/worlds/small_warehouse/small_warehouse.world"
URDF="${PKG_TB3}/urdf/turtlebot3_burger.urdf"
SDF="${PKG_TB3}/models/turtlebot3_burger/model.sdf"

echo "--- Starting Xvfb"
Xvfb :99 -screen 0 1280x720x24 &
XVFB_PID=$!
export DISPLAY=:99
sleep 2

echo "--- Launching Gazebo server"
ros2 launch gazebo_ros gzserver.launch.py world:="$WORLD" &
GZSERVER_PID=$!
sleep 10

echo "--- Launching robot_state_publisher"
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p "robot_description:=$(cat "$URDF")" -p use_sim_time:=true &
RSP_PID=$!
sleep 2

echo "--- Spawning TurtleBot3 Burger"
ros2 run gazebo_ros spawn_entity.py \
  -entity turtlebot3_burger \
  -file "$SDF" \
  -x 0.0 -y -2.0 -z 0.01 -Y 0.0
sleep 5

echo "--- Launching SLAM Toolbox"
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true &
SLAM_PID=$!
sleep 10

echo "--- Running automated exploration (180 seconds)"
python3 "$SCRIPT_DIR/map_explorer.py" || true
sleep 5

echo "--- Saving map"
mkdir -p "$REPO_ROOT/map_output"
cd "$REPO_ROOT/map_output"
ros2 run nav2_map_server map_saver_cli \
  -f warehouse \
  --ros-args -p use_sim_time:=true || true
sleep 3

echo "--- Map files generated:"
ls -la "$REPO_ROOT/map_output/"

echo "--- Cleaning up"
kill $SLAM_PID $RSP_PID $GZSERVER_PID $XVFB_PID 2>/dev/null || true
wait 2>/dev/null || true

if [ -f "$REPO_ROOT/map_output/warehouse.pgm" ]; then
  echo "Map generation successful!"
else
  echo "WARNING: warehouse.pgm not found. Map generation may have failed."
  exit 1
fi
