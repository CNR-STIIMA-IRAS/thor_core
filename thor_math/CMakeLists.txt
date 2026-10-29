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

# Cap parallel compile and link jobs by physical memory: translation units
# that include Pinocchio peak at a few GB each. Job pools are honored by the
# Ninja generators only; with Makefiles the caller's -j decides.
set(THOR_MATH_JOB_MEMORY_MB 3072 CACHE STRING
  "Estimated peak memory of one compile job, used to size the job pool")
set(THOR_MATH_RESERVED_MEMORY_MB 4096 CACHE STRING
  "Memory left to the rest of the system when sizing the job pool")
if(NOT CMAKE_JOB_POOL_COMPILE)
  cmake_host_system_information(RESULT _total_memory_mb QUERY TOTAL_PHYSICAL_MEMORY)
  math(EXPR _max_jobs
    "(${_total_memory_mb} - ${THOR_MATH_RESERVED_MEMORY_MB}) / ${THOR_MATH_JOB_MEMORY_MB}")
  if(_max_jobs LESS 1)
    set(_max_jobs 1)
  endif()
  set_property(GLOBAL APPEND PROPERTY JOB_POOLS thor_math_jobs=${_max_jobs})
  set(CMAKE_JOB_POOL_COMPILE thor_math_jobs)
  set(CMAKE_JOB_POOL_LINK thor_math_jobs)
  if(CMAKE_GENERATOR MATCHES "Ninja")
    message(STATUS "thor_math: at most ${_max_jobs} parallel jobs (${_total_memory_mb} MB RAM)")
  endif()
endif()

option(BUILD_UNIT_TESTS "Build the unit tests" OFF)
option(BUILD_INTEGRATION_TESTS "Build the integration test executable" OFF)
option(BUILD_EXAMPLES "Build the examples" OFF)
option(BUILD_DOCS "Add the thor_math_docs target (Doxygen HTML reference)" OFF)
option(ENABLE_COVERAGE "Enable source-based test coverage instrumentation" OFF)
# Flags that change the layout of Eigen objects (e.g. -march=x86-64-v3) must
# be the same in Pinocchio, thor_math and every program that uses them, or
# they crash. They are exported with the target, so that its users get them.
set(THOR_MATH_ARCH_FLAGS "" CACHE STRING
  "Architecture flags shared with every user of thor_math (e.g. -march=x86-64-v3); Pinocchio must be built with the same")

find_package(Eigen3 REQUIRED COMPONENTS Core Dense Geometry)
find_package(pinocchio REQUIRED CONFIG)

add_library(thor_math SHARED
  src/thor_math/cbf.cpp
  src/thor_math/thor_math.cpp
)
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

if(THOR_MATH_ARCH_FLAGS)
  separate_arguments(_arch_flags UNIX_COMMAND "${THOR_MATH_ARCH_FLAGS}")
  target_compile_options(thor_math PUBLIC ${_arch_flags})
  message(STATUS "thor_math: architecture flags exported to users: ${THOR_MATH_ARCH_FLAGS}")
endif()

if(ENABLE_COVERAGE)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(thor_math PRIVATE -O0 -g --coverage)
    target_link_options(thor_math PRIVATE --coverage)
  else()
    message(FATAL_ERROR "ENABLE_COVERAGE requires GCC or Clang")
  endif()
endif()

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
  find_package(GTest REQUIRED)
  include(GoogleTest)
endif()

if(BUILD_UNIT_TESTS)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)

  add_executable(thor_math_unit_tests test/test_evolution.cpp test/test_cbf.cpp)
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

if(BUILD_EXAMPLES)
  add_executable(thor_math_cbf_mpc_example examples/cbf_mpc_example.cpp)
  target_link_libraries(thor_math_cbf_mpc_example PRIVATE thor_math::thor_math)
  target_compile_definitions(thor_math_cbf_mpc_example
    PRIVATE
      THOR_MATH_EXAMPLE_URDF="${CMAKE_CURRENT_SOURCE_DIR}/test/data/ur10/ur10_with_intermediates.urdf"
  )
endif()

if(BUILD_DOCS)
  find_package(Doxygen REQUIRED)
  set(THOR_MATH_DOCS_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/docs")
  configure_file(docs/Doxyfile.in "${CMAKE_CURRENT_BINARY_DIR}/Doxyfile" @ONLY)
  add_custom_target(thor_math_docs
    COMMAND "${CMAKE_COMMAND}" -E make_directory "${THOR_MATH_DOCS_OUTPUT_DIR}"
    COMMAND Doxygen::doxygen "${CMAKE_CURRENT_BINARY_DIR}/Doxyfile"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    COMMENT "Generating the thor_math API reference in ${THOR_MATH_DOCS_OUTPUT_DIR}/html"
    VERBATIM
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
