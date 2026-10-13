cmake_minimum_required(VERSION 3.8)
project(turtlebot4_bai04)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config rviz maps worlds macos
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/check_topics.py
  scripts/compare_pose.py
  scripts/demo.sh
  scripts/explore_route.py
  scripts/send_goals.py
  scripts/stop_sim.sh
  scripts/watch_goals.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
