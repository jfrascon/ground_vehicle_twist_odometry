cmake_minimum_required(VERSION 3.5)
project(ground_vehicle_twist_odometry)

# Default to C++17.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

# Centralized dependencies list
set(dependencies
    geometry_msgs
    nav_msgs
    rclcpp
    std_srvs
    tf2
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
  target_compile_options(
    ${target} PRIVATE -Wall -Wconversion -Wextra -Wpedantic)
  ament_target_dependencies(${target} ${dependencies})

  list(APPEND gvto_targets ${target})
endforeach()

install(TARGETS ${gvto_targets} DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME}/)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # Exclude linters that require package-specific configuration.
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_clang_format
    ament_cmake_cpplint
    ament_cmake_flake8
    ament_cmake_cppcheck)

  # Register the remaining linters declared as test dependencies in package.xml.
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_clang_format REQUIRED)

  ament_clang_format(CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/.clang-format" include src test)

  find_package(ament_cmake_cpplint REQUIRED)

  ament_cpplint(
    MAX_LINE_LENGTH
    120
    FILTERS
    -whitespace/newline
    -whitespace/forcolon
    -build/include_order
    -runtime/indentation_namespace
    -legal/copyright)

  find_package(ament_cmake_flake8 REQUIRED)

  ament_flake8(CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/ament_flake8.ini")

  find_package(ament_cmake_cppcheck REQUIRED)

  # Check only headers and source files.
  # Some gtest macros such as TEST_F and TEST_P can produce cppcheck false positives in test
  # sources.
  ament_cppcheck(include src INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/include")

  # ament_cppcheck skips cppcheck versions known to be slow unless this environment variable
  # explicitly allows them.
  set_tests_properties(cppcheck PROPERTIES ENVIRONMENT "AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1")

  ament_add_gtest(test_ground_vehicle_twist_odometry test/test_ground_vehicle_twist_odometry.cpp)
  target_include_directories(test_ground_vehicle_twist_odometry PRIVATE include)
  ament_target_dependencies(
    test_ground_vehicle_twist_odometry
    geometry_msgs
    nav_msgs
    rclcpp
    std_srvs
    tf2
    tf2_ros)

  ament_add_pytest_test(test_launch_contract test/test_launch_contract.py)
endif()

ament_package()
