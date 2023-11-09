cmake_minimum_required(VERSION 3.5)
project(eut_ground_vehicle_twist_odometry)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(ground_vehicle_twist_odometry_node src/eut_ground_vehicle_twist_odometry_main.cpp)
# target_compile_definitions(ground_vehicle_twist_odometry_node PRIVATE <ANY_DEFINITION>)
target_include_directories(ground_vehicle_twist_odometry_node PRIVATE include)
target_compile_options(ground_vehicle_twist_odometry_node PRIVATE -Wall -Wconversion -Wextra -Wpedantic) # -Werror
ament_target_dependencies(ground_vehicle_twist_odometry_node rclcpp geometry_msgs nav_msgs tf2 tf2_geometry_msgs tf2_ros)

add_executable(ground_vehicle_timestamped_twist_odometry_node src/eut_ground_vehicle_timestamped_twist_odometry_main.cpp)
# target_compile_definitions(ground_vehicle_timestamped_twist_odometry_node PRIVATE <ANY_DEFINITION>)
target_include_directories(ground_vehicle_timestamped_twist_odometry_node PRIVATE include)
target_compile_options(ground_vehicle_timestamped_twist_odometry_node PRIVATE -Wall -Wconversion -Wextra -Wpedantic) # -Werror
ament_target_dependencies(ground_vehicle_timestamped_twist_odometry_node rclcpp geometry_msgs nav_msgs tf2 tf2_geometry_msgs tf2_ros)

install(TARGETS
  ground_vehicle_twist_odometry_node
  ground_vehicle_timestamped_twist_odometry_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
