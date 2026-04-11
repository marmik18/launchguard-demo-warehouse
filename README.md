# LaunchGuard Warehouse Demo

A ROS2 warehouse AMR demo using TurtleBot3 Burger + Nav2 in the AWS Small Warehouse Gazebo world. Designed for use with [LaunchGuard](https://launchguard.dev) continuous testing.

## What it does

The robot autonomously navigates between waypoints in a simulated warehouse. Two `launch_testing` tests verify:

1. **test_navigate_to_goal** -- the robot successfully reaches all waypoints
2. **test_safe_distance** -- the robot maintains minimum clearance from obstacles throughout navigation

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run tests

```bash
colcon test --packages-select warehouse_bot --return-code-on-test-failure
colcon test-result --verbose
```

## Triggering a regression

To simulate a navigation regression, edit `src/warehouse_bot/config/nav2_params.yaml`:

- Change `inflation_radius` from `0.55` to `0.05` in both `local_costmap` and `global_costmap`
- Change `robot_radius` from `0.22` to `0.105` in both `local_costmap` and `global_costmap`

This removes the safety buffer, causing the planner to route dangerously close to shelves. `test_safe_distance` will fail, and LaunchGuard will capture the comparison video.
