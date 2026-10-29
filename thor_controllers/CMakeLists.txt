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
# Pinocchio looks for Boost: use the BoostConfig.cmake of Boost, not the
# FindBoost module removed in CMake 3.30 (the policy is new in that version).
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()
find_package(pinocchio REQUIRED)
find_package(thor_math REQUIRED)
foreach(dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()

generate_parameter_library(thor_parameters src/thor_parameters.yaml)
generate_parameter_library(thor_chainable_parameters src/thor_chainable_parameters.yaml)

# The MPC without ROS: tested on its own.
add_library(thor_core SHARED src/thor_core.cpp)
target_compile_features(thor_core PUBLIC cxx_std_20)
target_include_directories(thor_core PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(thor_core PUBLIC thor_math::thor_math pinocchio::pinocchio Eigen3::Eigen)

# The ROS side shared by the controllers. It must not link
# joint_trajectory_controller: a plugin library that links another plugin
# library hides the plugins of the latter from class_loader.
add_library(thor_ros_common SHARED src/thor_ros_common.cpp)
target_compile_features(thor_ros_common PUBLIC cxx_std_20)
target_include_directories(thor_ros_common PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(thor_ros_common PUBLIC
  thor_core
  thor_parameters
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  realtime_tools::realtime_tools
  ${thor_msgs_TARGETS}
)

add_library(thor_chainable_controller SHARED src/thor_chainable_controller.cpp)
target_link_libraries(thor_chainable_controller PUBLIC
  thor_ros_common
  thor_chainable_parameters
  controller_interface::controller_interface
  hardware_interface::hardware_interface
  pluginlib::pluginlib
)

add_library(thor_trajectory_controller SHARED src/thor_trajectory_controller.cpp)
target_link_libraries(thor_trajectory_controller PUBLIC
  thor_ros_common
  joint_trajectory_controller::joint_trajectory_controller
  controller_interface::controller_interface
  hardware_interface::hardware_interface
  pluginlib::pluginlib
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
  target_link_libraries(test_thor_trajectory_controller thor_trajectory_controller)
  target_compile_definitions(test_thor_trajectory_controller PRIVATE THOR_TEST_URDF="${THOR_TEST_URDF}")

  ament_add_gmock(test_thor_chainable_controller test/test_thor_chainable_controller.cpp)
  target_link_libraries(test_thor_chainable_controller thor_chainable_controller)
  target_compile_definitions(test_thor_chainable_controller PRIVATE THOR_TEST_URDF="${THOR_TEST_URDF}")
endif()

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS thor_core thor_ros_common thor_chainable_controller thor_trajectory_controller
    thor_parameters thor_chainable_parameters
  EXPORT export_thor_controllers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_thor_controllers HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS} thor_math pinocchio Eigen3)
ament_package()
