# BSD 3-Clause License
#
# Copyright (c) 2026, Nicola Pedrocchi
# Copyright (c) Manuel Beschi (manuel.beschi@unibs.it)
# Copyright (c) Marco Faroni (marco.faroni@polimi.it)
# Copyright (c) Federico Parma (federico.parma@unibs.it)
# All rights reserved.
#
# See the project license for redistribution terms and warranty disclaimer.

cmake_minimum_required(VERSION 3.16.3)

# Keep the package.xml version as the single source of truth.
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/package.xml" THOR_MATH_PACKAGE_XML)
string(REGEX MATCH "<version>([0-9]+[.][0-9]+[.][0-9]+)</version>"
  _version_match "${THOR_MATH_PACKAGE_XML}")
if(NOT CMAKE_MATCH_1)
  message(FATAL_ERROR "Could not read the project version from package.xml")
endif()

project(thor_math VERSION "${CMAKE_MATCH_1}" LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(BUILD_UNIT_TESTS "Build the unit tests" OFF)
option(BUILD_INTEGRATION_TESTS "Build the integration test executable" OFF)

find_package(Eigen3 REQUIRED COMPONENTS Core Dense Geometry)
find_package(pinocchio REQUIRED CONFIG)

add_library(thor_math SHARED src/thor_math/thor_math.cpp)
add_library(thor_math::thor_math ALIAS thor_math)

target_compile_features(thor_math PUBLIC cxx_std_20)
set_target_properties(thor_math PROPERTIES
  CXX_EXTENSIONS OFF
  POSITION_INDEPENDENT_CODE ON
  VERSION "${PROJECT_VERSION}"
  SOVERSION "${PROJECT_VERSION_MAJOR}"
)

target_include_directories(thor_math BEFORE
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(thor_math
  PUBLIC
    Eigen3::Eigen
    pinocchio::pinocchio
)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(thor_math PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Wunreachable-code
    "$<$<CONFIG:Release>:-O3>"
  )
elseif(MSVC)
  target_compile_options(thor_math PRIVATE /W4)
endif()

if(BUILD_UNIT_TESTS OR BUILD_INTEGRATION_TESTS)
  include(CTest)
  enable_testing()
  find_package(GTest REQUIRED CONFIG)
  include(GoogleTest)
endif()

if(BUILD_UNIT_TESTS)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)

  add_executable(thor_math_unit_tests test/test_evolution.cpp)
  target_link_libraries(thor_math_unit_tests
    PRIVATE
      thor_math::thor_math
      GTest::gtest_main
  )
  gtest_discover_tests(thor_math_unit_tests
    TEST_PREFIX "thor_math.unit."
  )

  add_test(
    NAME thor_math.unit.python_tools
    COMMAND "${Python3_EXECUTABLE}" -m unittest
      discover -s "${CMAKE_CURRENT_SOURCE_DIR}/test"
      -p "test_python_tools.py" -v
  )
endif()

if(BUILD_INTEGRATION_TESTS)
  add_executable(thor_math_integration_test test/test_thor_math.cpp)
  target_link_libraries(thor_math_integration_test
    PRIVATE
      thor_math::thor_math
      GTest::gtest_main
  )
  target_compile_definitions(thor_math_integration_test
    PRIVATE
      THOR_MATH_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test"
  )
  gtest_discover_tests(thor_math_integration_test
    TEST_PREFIX "thor_math.integration."
    DISCOVERY_TIMEOUT 30
  )
endif()

set(THOR_MATH_INSTALL_CMAKEDIR
  "${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/cmake"
  CACHE STRING "Install directory for thor_math CMake package files"
)

install(TARGETS thor_math
  EXPORT thor_mathTargets
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
  INCLUDES DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
)

install(DIRECTORY include/thor_math
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

install(DIRECTORY include/eigen_matrix_utils
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

install(DIRECTORY include/qpmad
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

install(FILES
  third_party/qpmad/LICENSE
  third_party/qpmad/NOTICE
  third_party/qpmad/VERSION
  DESTINATION "${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/third_party/qpmad"
)

configure_package_config_file(
  cmake/config.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/thor_mathConfig.cmake"
  INSTALL_DESTINATION "${THOR_MATH_INSTALL_CMAKEDIR}"
)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/thor_mathConfigVersion.cmake"
  VERSION "${PROJECT_VERSION}"
  COMPATIBILITY SameMajorVersion
)

install(EXPORT thor_mathTargets
  FILE thor_mathTargets.cmake
  NAMESPACE thor_math::
  DESTINATION "${THOR_MATH_INSTALL_CMAKEDIR}"
)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/thor_mathConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/thor_mathConfigVersion.cmake"
  DESTINATION "${THOR_MATH_INSTALL_CMAKEDIR}"
)
