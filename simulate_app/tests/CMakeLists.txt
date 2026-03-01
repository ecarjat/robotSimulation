cmake_minimum_required(VERSION 3.22)

option(USE_FETCHCONTENT_CATCH2 "Fetch Catch2 if not found" ON)

include(FetchContent)
find_package(Catch2 3 QUIET)

if(NOT Catch2_FOUND AND USE_FETCHCONTENT_CATCH2 AND NOT TARGET Catch2::Catch2WithMain)
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

set(SIM_TEST_MODEL_PATH "${CMAKE_CURRENT_LIST_DIR}/../../myRobot/scene.xml")

# Catch2-backed simulate_app tests (TestMate introspectable)
add_executable(test_motion_controller_bridge
  test_motion_controller_bridge.cc
  ../motion_controller_bridge.cc
  ${FIRMWARE_DIR}/app/control/MotionController.cpp
  ${FIRMWARE_DIR}/app/control/StateEstimator.cpp
  ${FIRMWARE_DIR}/app/control/lqr_lut.c
  ${FIRMWARE_DIR}/app/control/ekf/BalancerEKF.cpp
  ${FIRMWARE_DIR}/tests/fakes/log_fake.c
)

target_include_directories(test_motion_controller_bridge PRIVATE
  ${MUJOCO_INCLUDE_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/..
  ${FIRMWARE_DIR}/app/control
  ${FIRMWARE_DIR}/app/control/ekf
  ${FIRMWARE_DIR}/app/utils
  ${FIRMWARE_DIR}/app/drivers
  ${FIRMWARE_DIR}/libs/TinyEKF
)

target_compile_definitions(test_motion_controller_bridge PRIVATE
  APP_CONFIG_HOST
  APP_CONFIG_SIM_FAST
  SIM_TEST_MODEL_PATH="${SIM_TEST_MODEL_PATH}"
  TEST_BALANCE_BIN="$<TARGET_FILE:test_balance>"
)

target_link_libraries(test_motion_controller_bridge PRIVATE
  Catch2::Catch2WithMain
  ${MUJOCO_LIB}
)

add_test(
  NAME simulate_app_catch2
  COMMAND $<TARGET_FILE:test_motion_controller_bridge>
)
set_tests_properties(simulate_app_catch2 PROPERTIES TIMEOUT 60)
