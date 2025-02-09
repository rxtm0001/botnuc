cmake_minimum_required(VERSION 3.5)
project(behavior_tree)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Instalar scripts ejecutables
install(PROGRAMS
  src/navigate_to_flag.py
  src/ApproachFlagController.py
  src/General_controller.py
  src/capture_flag_node.py
  src/SearchFlag.py
  src/electromagnet_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
