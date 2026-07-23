cmake_minimum_required(VERSION 3.8)
project(kv_executive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(ompl REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rcl_interfaces REQUIRED)

ament_python_install_package(${PROJECT_NAME})

include_directories(${OMPL_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS})
add_executable(motion_planning src/planner_node.cpp)
ament_target_dependencies(motion_planning
    rclcpp ompl geometry_msgs nav_msgs rcl_interfaces octomap octomap_msgs
    tf2 tf2_geometry_msgs mavros_msgs
)
target_link_libraries(motion_planning ${OMPL_LIBRARIES} ${OCTOMAP_LIBRARIES})
install(TARGETS
    motion_planning
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Python executables
install(PROGRAMS
  scripts/executive_node.py
  DESTINATION lib/${PROJECT_NAME}
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
