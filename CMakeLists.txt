cmake_minimum_required(VERSION 3.8)
project(duatic_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# gz-transport rather than the `gz` CLI: a service call should not cost a process.
# Both come from the ROS vendor packages, so no extra system dependency.
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(duatic_helper_msgs REQUIRED)

add_executable(sim_scene_node src/sim_scene_node.cpp)
target_link_libraries(sim_scene_node
  rclcpp::rclcpp
  gz-transport13::core
  gz-msgs10::core
  ${duatic_helper_msgs_TARGETS}
)

install(TARGETS sim_scene_node DESTINATION lib/${PROJECT_NAME})

# Install  directories
install(DIRECTORY
  launch
  worlds
  models
  docker
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

# Install scripts as executables
install(PROGRAMS
  scripts/start_sim.py
  scripts/unpause_sim.py
  DESTINATION lib/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  # Enable testing
  find_package(ament_cmake_ros REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  function(add_ros_isolated_launch_test path)
    set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
    add_launch_test("${path}" RUNNER "${RUNNER}" ${ARGN})
  endfunction()
  add_ros_isolated_launch_test(test/launch_gazebo_warehouse.py)
endif()

ament_package()
