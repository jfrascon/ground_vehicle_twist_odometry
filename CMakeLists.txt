cmake_minimum_required(VERSION 3.5)
project(ground_vehicle_twist_odometry)

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

# Centralized dependencies list
set(dependencies
    geometry_msgs
    nav_msgs
    rclcpp
    std_srvs
    tf2
    tf2_geometry_msgs
    tf2_ros)

# Find all dependencies
foreach(dep IN LISTS dependencies)
  find_package(${dep} REQUIRED)
endforeach()

# Generate both executables without duplicating configuration, one with
# timestamped Twist input and one without timestamp
set(gvto_targets)

foreach(use_timestamped_twist IN ITEMS 0 1)
  if(use_timestamped_twist EQUAL 1)
    set(suffix _w_timestamp)
  else()
    set(suffix)
  endif()

  set(target ground_vehicle_twist_odometry_node${suffix})

  add_executable(${target} src/ground_vehicle_twist_odometry_main.cpp)
  target_compile_definitions(
    ${target} PRIVATE USE_TIMESTAMPED_TWIST=${use_timestamped_twist})
  target_include_directories(${target} PRIVATE include)
  target_compile_options(${target} PRIVATE -Wall -Wconversion -Wextra
                                           -Wpedantic) # -Werror
  ament_target_dependencies(${target} ${dependencies})

  list(APPEND gvto_targets ${target})
endforeach()

install(TARGETS ${gvto_targets} DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights uncomment
  # the line when a copyright and license is not present in all source files
  # set(ament_cmake_copyright_FOUND TRUE) the following line skips cpplint (only
  # works in a git repo) uncomment the line when this package is not in a git
  # repo set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
