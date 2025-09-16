cmake_minimum_required(VERSION 3.8)
project(navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Navigation2 core
find_package(nav2_bringup REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)

# Localization
find_package(nav2_amcl REQUIRED)

# Map & lifecycle management
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(launch REQUIRED)
find_package(launch_ros REQUIRED)
find_package(ament_index_python REQUIRED)

find_package(nav2_map_server REQUIRED)
find_package(nav2_lifecycle_manager REQUIRED)


install(
  DIRECTORY maps/
  DESTINATION share/${PROJECT_NAME}/maps
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
