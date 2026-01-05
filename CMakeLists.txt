# Top-level CMake for ICM42688 Zephyr module
# Adds public include dir and driver sources when used as a Zephyr module

zephyr_include_directories(${CMAKE_CURRENT_LIST_DIR}/drivers/include)
add_subdirectory(drivers)
