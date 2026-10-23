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
find_package(action_msgs REQUIRED)
find_package(service_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PlanPath.srv"
  "action/ExecuteMission.action"
  "msg/BatteryUsage.msg"
  "msg/BatteryReturnEstimate.msg"
  DEPENDENCIES nav_msgs geometry_msgs std_msgs
)

# rosidl_generator_py installs the generated interfaces with this same
# name, so install the handwritten Python modules directly alongside it.
install(
  DIRECTORY ${PROJECT_NAME}/
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
  PATTERN "*.pyc" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
)

include_directories(${OMPL_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS})
add_executable(motion_planning src/planner_node.cpp)
ament_target_dependencies(motion_planning
    rclcpp ompl geometry_msgs nav_msgs rcl_interfaces octomap octomap_msgs
    tf2 tf2_ros tf2_geometry_msgs mavros_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
target_link_libraries(motion_planning
  ${OMPL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  "${cpp_typesupport_target}"
)
add_executable(battery_monitor_node src/battery_monitor_node.cpp)
target_compile_features(battery_monitor_node PRIVATE cxx_std_17)
ament_target_dependencies(battery_monitor_node
  rclcpp geometry_msgs nav_msgs sensor_msgs mavros_msgs std_msgs
)
target_link_libraries(battery_monitor_node "${cpp_typesupport_target}")
install(TARGETS
    motion_planning
    battery_monitor_node
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
  scripts/battery_monitor_node.py
  scripts/diagnose_ros_executor.py
  scripts/configure_px4_external_odometry.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_add_pytest_test(mission_geometry test/test_mission_geometry.py)
  ament_add_pytest_test(mission_executive test/test_mission_executive.py)
  ament_add_pytest_test(battery_model test/test_battery_model.py TIMEOUT 45)
  ament_add_pytest_test(battery_monitor test/test_battery_monitor.py TIMEOUT 45)
  ament_add_pytest_test(battery_monitor_ros test/test_battery_monitor_ros.py TIMEOUT 45)
  ament_add_pytest_test(
    battery_monitor_native_ros test/test_battery_monitor_native_ros.py TIMEOUT 90)
  ament_add_pytest_test(
    battery_monitor_constant_ros test/test_battery_monitor_constant_ros.py TIMEOUT 120)
  ament_add_pytest_test(planner_inputs test/test_planner_inputs.py TIMEOUT 45)
  # FCU state updates, telemetry loss, and three successive simulated-clock goals.
  ament_add_pytest_test(mission_action test/test_mission_action.py TIMEOUT 210)
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
