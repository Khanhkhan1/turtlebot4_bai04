#!/usr/bin/env bash
# Stops everything sim.launch.py starts. Ctrl-C on the launch normally does this; use it if
# a run was killed and left ros_gz bridges behind (a second /clock publisher makes time
# jump backwards and breaks TF, AMCL and Nav2 on the next launch).
# "ros2 launch turtlebot4_bai0", "lib/turtlebot4_bai0": launches and nodes of bài 04-06 (killed first:
# the bài 05/06 launches respawn RViz2).
for p in "ros2 launch turtlebot4_bai0" lib/turtlebot4_bai0 "ign gazebo" ros_gz_bridge/parameter_bridge ros_gz_sim/create \
         irobot_create_nodes irobot_create_ignition_toolbox robot_state_publisher joint_state_publisher \
         static_transform_publisher controller_manager/spawner slam_toolbox /amcl /map_server \
         controller_server planner_server behavior_server bt_navigator waypoint_follower \
         velocity_smoother smoother_server lifecycle_manager rviz2 \
         explore_route.py send_goals.py watch_goals.py compare_pose.py check_topics.py; do
  pkill -9 -f "$p" 2>/dev/null
done
echo "stopped"
