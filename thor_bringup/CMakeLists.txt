cmake_minimum_required(VERSION 3.16)
project(thor_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch meshes rviz urdf DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/human_sim.py
  scripts/robot_goal.py
  scripts/thor_markers.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_ur10_thor.launch.py TIMEOUT 120)
  add_launch_test(test/test_ur10_thor.launch.py
    TARGET test_ur10_thor_chained
    ARGS "chained:=true"
    TIMEOUT 120
  )
endif()

ament_package()
