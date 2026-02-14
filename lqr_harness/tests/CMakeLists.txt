cmake_minimum_required(VERSION 3.22)

project(linearize_hip_tests LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(USE_FETCHCONTENT_CATCH2 "Fetch Catch2 if not found" ON)

# Note: MuJoCo is provided by parent CMakeLists.txt as mujoco::mujoco interface target

# Define test model path (absolute path to robot.xml)
set(TEST_MODEL_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../myRobot/robot.xml")

include(FetchContent)
find_package(Catch2 3 QUIET)

if(NOT Catch2_FOUND AND USE_FETCHCONTENT_CATCH2)
  FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG v3.5.4
  )
  FetchContent_MakeAvailable(Catch2)
endif()

if(NOT Catch2_FOUND AND NOT TARGET Catch2::Catch2WithMain)
  message(FATAL_ERROR "Catch2 v3 not found. Install it or enable USE_FETCHCONTENT_CATCH2.")
endif()

enable_testing()

# Test 1: Equilibrium solver regression test
add_executable(test_equilibrium_solver
  test_equilibrium_solver.cpp
)

target_include_directories(test_equilibrium_solver PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/..
)

target_compile_definitions(test_equilibrium_solver PRIVATE
  TEST_MODEL_PATH="${TEST_MODEL_PATH}"
)

target_link_libraries(test_equilibrium_solver PRIVATE
  Catch2::Catch2WithMain
  mujoco::mujoco
)

add_test(NAME test_equilibrium_solver COMMAND test_equilibrium_solver)

# Test 2: Linearization output validation
add_executable(test_linearization_output
  test_linearization_output.cpp
)

target_include_directories(test_linearization_output PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/..
)

target_compile_definitions(test_linearization_output PRIVATE
  TEST_MODEL_PATH="${TEST_MODEL_PATH}"
)

target_link_libraries(test_linearization_output PRIVATE
  Catch2::Catch2WithMain
  mujoco::mujoco
)

add_test(NAME test_linearization_output COMMAND test_linearization_output)

# Test 3: Position actuator control verification
add_executable(test_position_actuators
  test_position_actuators.cpp
)

target_include_directories(test_position_actuators PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/..
)

target_compile_definitions(test_position_actuators PRIVATE
  TEST_MODEL_PATH="${TEST_MODEL_PATH}"
)

target_link_libraries(test_position_actuators PRIVATE
  Catch2::Catch2WithMain
  mujoco::mujoco
)

add_test(NAME test_position_actuators COMMAND test_position_actuators)
