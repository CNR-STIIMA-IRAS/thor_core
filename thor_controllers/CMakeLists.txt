cmake_minimum_required(VERSION 3.16)
project(thor_controllers LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  hardware_interface
  joint_trajectory_controller
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  thor_msgs
  trajectory_msgs
)

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(thor_math REQUIRED)
foreach(dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()

generate_parameter_library(thor_parameters src/thor_parameters.yaml)

# The MPC without ROS: tested on its own.
add_library(thor_core SHARED src/thor_core.cpp)
target_compile_features(thor_core PUBLIC cxx_std_20)
target_include_directories(thor_core PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(thor_core PUBLIC thor_math::thor_math pinocchio::pinocchio Eigen3::Eigen)

add_library(thor_controllers SHARED
  src/thor_ros_common.cpp
  src/thor_trajectory_controller.cpp
)
target_compile_features(thor_controllers PUBLIC cxx_std_20)
target_include_directories(thor_controllers PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(thor_controllers PUBLIC
  thor_core
  thor_parameters
  controller_interface::controller_interface
  hardware_interface::hardware_interface
  joint_trajectory_controller::joint_trajectory_controller
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  realtime_tools::realtime_tools
  ${thor_msgs_TARGETS}
  ${trajectory_msgs_TARGETS}
)
pluginlib_export_plugin_description_file(controller_interface thor_controllers.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  set(THOR_TEST_URDF "${CMAKE_CURRENT_SOURCE_DIR}/test/ur10.urdf")

  ament_add_gmock(test_thor_core test/test_thor_core.cpp)
  target_link_libraries(test_thor_core thor_core)
  target_compile_definitions(test_thor_core PRIVATE THOR_TEST_URDF="${THOR_TEST_URDF}")

  ament_add_gmock(test_thor_trajectory_controller test/test_thor_trajectory_controller.cpp)
  target_link_libraries(test_thor_trajectory_controller thor_controllers)
  target_compile_definitions(test_thor_trajectory_controller PRIVATE THOR_TEST_URDF="${THOR_TEST_URDF}")
endif()

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS thor_core thor_controllers thor_parameters
  EXPORT export_thor_controllers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_thor_controllers HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS} thor_math pinocchio Eigen3)
ament_package()
