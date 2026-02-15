cmake_minimum_required(VERSION 3.22)

if(APPLE)
  project(mujoco_simulate_mc LANGUAGES C CXX OBJCXX)
else()
  project(mujoco_simulate_mc LANGUAGES C CXX)
endif()

include(CTest)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(MUJOCO_DIR $ENV{MUJOCO_DIR})
if(NOT MUJOCO_DIR)
  set(MUJOCO_DIR $ENV{MUJOCO_ROOT})
endif()

# VSCode/CMake Tools friendly default: local mujoco.framework in repo root.
if(NOT MUJOCO_DIR)
  set(MUJOCO_DIR "${CMAKE_CURRENT_LIST_DIR}/../mujoco.framework")
endif()

if(NOT EXISTS "${MUJOCO_DIR}")
  message(FATAL_ERROR "Set MUJOCO_DIR or MUJOCO_ROOT to your MuJoCo install path.")
endif()

# Support both the standard SDK layout and macOS .framework layout.
if(EXISTS "${MUJOCO_DIR}/Headers/mujoco.h")
  set(MUJOCO_FRAMEWORK_HEADERS "${MUJOCO_DIR}/Headers")
  set(MUJOCO_INCLUDE_SHIM "${CMAKE_BINARY_DIR}/mujoco_include")
  file(MAKE_DIRECTORY "${MUJOCO_INCLUDE_SHIM}")
  if(NOT EXISTS "${MUJOCO_INCLUDE_SHIM}/mujoco")
    file(CREATE_LINK "${MUJOCO_FRAMEWORK_HEADERS}"
         "${MUJOCO_INCLUDE_SHIM}/mujoco" SYMBOLIC)
  endif()
  set(MUJOCO_INCLUDE_DIR "${MUJOCO_INCLUDE_SHIM}")
  file(GLOB MUJOCO_LIB_CAND
    "${MUJOCO_DIR}/Versions/*/libmujoco*.dylib"
    "${MUJOCO_DIR}/Versions/*/mujoco.tbd"
    "${MUJOCO_DIR}/mujoco.tbd"
  )
  list(GET MUJOCO_LIB_CAND 0 MUJOCO_LIB)
else()
  find_path(MUJOCO_INCLUDE_DIR mujoco.h
    PATHS ${MUJOCO_DIR}/include/mujoco ${MUJOCO_DIR}/include ${MUJOCO_DIR}
  )
  find_library(MUJOCO_LIB mujoco
    PATHS ${MUJOCO_DIR}/lib ${MUJOCO_DIR}/lib64 ${MUJOCO_DIR}/bin
  )
endif()

if(NOT MUJOCO_INCLUDE_DIR OR NOT MUJOCO_LIB)
  message(FATAL_ERROR "Could not find MuJoCo headers or library in ${MUJOCO_DIR}.")
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(GLFW REQUIRED IMPORTED_TARGET glfw3)

set(FIRMWARE_DIR ${CMAKE_CURRENT_LIST_DIR}/../../stm32Controller/firmware)

set(SIMULATE_SOURCES
  main.cc
  simulate.cc
  platform_ui_adapter.cc
  glfw_adapter.cc
  glfw_dispatch.cc
  motion_controller_bridge.cc
  ${FIRMWARE_DIR}/app/control/MotionController.cpp
  ${FIRMWARE_DIR}/app/control/StateEstimator.cpp
  ${FIRMWARE_DIR}/app/control/lqr_lut.c
  ${FIRMWARE_DIR}/app/control/ekf/BalancerEKF.cpp
  ${FIRMWARE_DIR}/tests/fakes/log_fake.c
)

if(APPLE)
  list(APPEND SIMULATE_SOURCES glfw_corevideo.mm macos_gui.mm)
endif()

add_executable(simulate_mc ${SIMULATE_SOURCES})

target_include_directories(simulate_mc PRIVATE
  ${MUJOCO_INCLUDE_DIR}
  ${CMAKE_CURRENT_LIST_DIR}
  ${FIRMWARE_DIR}/app/control
  ${FIRMWARE_DIR}/app/control/ekf
  ${FIRMWARE_DIR}/app/utils
  ${FIRMWARE_DIR}/app/drivers
  ${FIRMWARE_DIR}/libs/TinyEKF
)

target_compile_definitions(simulate_mc PRIVATE APP_CONFIG_HOST)

target_link_libraries(simulate_mc PRIVATE ${MUJOCO_LIB} PkgConfig::GLFW)

if(APPLE)
  target_compile_definitions(simulate_mc PRIVATE GL_SILENCE_DEPRECATION)
  target_link_libraries(simulate_mc PRIVATE
    "-framework Cocoa"
    "-framework OpenGL"
    "-framework CoreVideo"
    "-framework IOKit"
    "-framework QuartzCore"
  )
endif()

# Headless balance test executable
add_executable(test_balance
  test_balance.cc
  motion_controller_bridge.cc
  ${FIRMWARE_DIR}/app/control/MotionController.cpp
  ${FIRMWARE_DIR}/app/control/StateEstimator.cpp
  ${FIRMWARE_DIR}/app/control/lqr_lut.c
  ${FIRMWARE_DIR}/app/control/ekf/BalancerEKF.cpp
  ${FIRMWARE_DIR}/tests/fakes/log_fake.c
)

target_include_directories(test_balance PRIVATE
  ${MUJOCO_INCLUDE_DIR}
  ${CMAKE_CURRENT_LIST_DIR}
  ${FIRMWARE_DIR}/app/control
  ${FIRMWARE_DIR}/app/control/ekf
  ${FIRMWARE_DIR}/app/utils
  ${FIRMWARE_DIR}/app/drivers
  ${FIRMWARE_DIR}/libs/TinyEKF
)

target_compile_definitions(test_balance PRIVATE APP_CONFIG_HOST)

target_link_libraries(test_balance PRIVATE ${MUJOCO_LIB})

option(BUILD_SIMULATE_TESTS "Build simulate_app regression tests" ON)
if(BUILD_SIMULATE_TESTS AND BUILD_TESTING)
  add_subdirectory(tests)
endif()
