cmake_minimum_required(VERSION 3.22)

project(robot2wheel_simulation LANGUAGES C CXX OBJCXX)

# Top-level entry for VSCode/CMake Tools.
# Build targets via: lqr_harness (linearize_hip) and simulate_app.

add_subdirectory(lqr_harness)
add_subdirectory(simulate_app)
