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
find_package(unique_identifier_msgs REQUIRED)
find_package(service_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
set(Python3_USE_STATIC_LIBS FALSE)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Embed)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PlanPath.srv"
  "srv/PlanToPose.srv"
  "srv/PlanInspectionPath.srv"
  "srv/QueryInspectionGeometry.srv"
  "action/ExecuteMission.action"
  "msg/BatteryUsage.msg"
  "msg/BatteryReturnEstimate.msg"
  "msg/MissionStateRequest.msg"
  "msg/MissionPhase.msg"
  "msg/FireSourceReport.msg"
  "msg/InspectionReport.msg"
  "msg/InspectionImageEvidence.msg"
  "msg/CompletedInspectionCluster.msg"
  "msg/CompletedInspectionClusters.msg"
  DEPENDENCIES nav_msgs geometry_msgs sensor_msgs std_msgs unique_identifier_msgs
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
    rclcpp ompl geometry_msgs nav_msgs sensor_msgs rcl_interfaces octomap octomap_msgs
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
add_executable(python_shared_runtime src/python_shared_runtime.cpp)
target_compile_features(python_shared_runtime PRIVATE cxx_std_17)
set_property(TARGET python_shared_runtime PROPERTY POSITION_INDEPENDENT_CODE TRUE)
target_compile_definitions(python_shared_runtime PRIVATE
  KV_PYTHON_EXECUTABLE="${Python3_EXECUTABLE}")
target_link_libraries(python_shared_runtime Python3::Python)
install(TARGETS
    motion_planning
    battery_monitor_node
    python_shared_runtime
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
  scripts/detection_monitor_node.py
  scripts/battery_monitor_node.py
  scripts/diagnose_ros_executor.py
  scripts/configure_px4_external_odometry.py
  scripts/relay_external_odometry.py
  scripts/view_inspection_images.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_add_pytest_test(mission_geometry test/test_mission_geometry.py)
  ament_add_pytest_test(launch_transport test/test_launch_transport.py)
  ament_add_pytest_test(external_odometry_relay test/test_external_odometry_relay.py TIMEOUT 45)
  ament_add_pytest_test(shared_python_runtime test/test_shared_python_runtime.py TIMEOUT 45)
  ament_add_pytest_test(mission_executive test/test_mission_executive.py)
  ament_add_pytest_test(verification test/test_verification.py TIMEOUT 45)
  ament_add_pytest_test(fire_loss_recovery test/test_fire_loss_recovery.py TIMEOUT 45)
  ament_add_pytest_test(mission_navigation test/test_mission_navigation.py TIMEOUT 45)
  ament_add_pytest_test(inspection_navigation test/test_inspection_navigation.py TIMEOUT 45)
  ament_add_pytest_test(inspection_heading test/test_inspection_heading.py TIMEOUT 45)
  ament_add_pytest_test(inspection_settling test/test_inspection_settling.py TIMEOUT 45)
  ament_add_pytest_test(inspection_observation test/test_inspection_observation.py TIMEOUT 45)
  ament_add_pytest_test(inspection_evidence test/test_inspection_evidence.py TIMEOUT 45)
  ament_add_pytest_test(inspection_report_view test/test_inspection_report_view.py TIMEOUT 45)
  ament_add_pytest_test(retained_wall_reporting test/test_retained_wall_reporting.py TIMEOUT 45)
  ament_add_pytest_test(candidate_selection test/test_candidate_selection.py TIMEOUT 45)
  ament_add_pytest_test(cluster_motion test/test_cluster_motion.py TIMEOUT 45)
  ament_add_pytest_test(inspection_departure test/test_inspection_departure.py TIMEOUT 45)
  ament_add_pytest_test(motion_policy test/test_motion_policy.py TIMEOUT 45)
  # Real mode handoffs and the committed-climb case make this a long DDS suite.
  ament_add_pytest_test(verify_ros test/test_verify_ros.py TIMEOUT 3600)
  ament_add_pytest_test(verify_process test/test_verify_process.py TIMEOUT 300)
  ament_add_pytest_test(detection_monitor test/test_detection_monitor.py)
  ament_add_pytest_test(detection_monitor_ros test/test_detection_monitor_ros.py TIMEOUT 120)
  ament_add_pytest_test(battery_model test/test_battery_model.py TIMEOUT 45)
  ament_add_pytest_test(battery_monitor test/test_battery_monitor.py TIMEOUT 45)
  ament_add_pytest_test(battery_monitor_ros test/test_battery_monitor_ros.py TIMEOUT 45)
  ament_add_pytest_test(
    battery_monitor_native_ros test/test_battery_monitor_native_ros.py TIMEOUT 90)
  ament_add_pytest_test(
    battery_monitor_constant_ros test/test_battery_monitor_constant_ros.py TIMEOUT 120)
  ament_add_pytest_test(planner_inputs test/test_planner_inputs.py TIMEOUT 300)
  # Statistical native-planner benchmark; skipped unless explicitly enabled.
  ament_add_pytest_test(
    smoke_wall_false_positives test/test_smoke_wall_false_positives.py TIMEOUT 600)
  ament_add_pytest_test(
    expanded_wall_evidence test/test_expanded_smoke_wall_evidence.py TIMEOUT 600)
  ament_add_pytest_test(
    smoke_navigation_sparse_terrain test/test_smoke_navigation_sparse_terrain.py TIMEOUT 45)
  ament_add_pytest_test(source_localization test/test_source_localization.py TIMEOUT 45)
  ament_add_pytest_test(completed_sites test/test_completed_sites.py TIMEOUT 45)
  # Repeated Exploration cycles, parent-state requests, and simulated FCU recovery.
  ament_add_pytest_test(mission_action test/test_mission_action.py TIMEOUT 300)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  # Register these checks explicitly so local generated trees are not source
  # inputs, and use this repository's first-line multiline docstring summaries.
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  # Avoid forked flake8 workers inheriting active Python runtime threads.
  execute_process(
    COMMAND "${Python3_EXECUTABLE}" "-c"
      "import pathlib, ament_flake8; print(pathlib.Path(ament_flake8.__file__).parent / 'configuration' / 'ament_flake8.ini')"
    OUTPUT_VARIABLE _flake8_default_config
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE _flake8_config_result)
  if(NOT _flake8_config_result EQUAL 0)
    message(FATAL_ERROR "Cannot locate the installed ament flake8 configuration")
  endif()
  configure_file("${_flake8_default_config}" "${CMAKE_CURRENT_BINARY_DIR}/flake8.ini" COPYONLY)
  file(APPEND "${CMAKE_CURRENT_BINARY_DIR}/flake8.ini" "\njobs = 1\n")
  ament_flake8(
    CONFIG_FILE "${CMAKE_CURRENT_BINARY_DIR}/flake8.ini"
    EXCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/build,${CMAKE_CURRENT_SOURCE_DIR}/install")
  ament_lint_cmake("${CMAKE_CURRENT_SOURCE_DIR}/CMakeLists.txt")
  ament_pep257(kv_executive launch scripts test --add-ignore D213 --add-select D212)
  ament_uncrustify(src include)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_flake8 ament_cmake_lint_cmake ament_cmake_pep257 ament_cmake_uncrustify)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
