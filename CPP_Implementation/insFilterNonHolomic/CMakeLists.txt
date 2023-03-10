cmake_minimum_required(VERSION 3.5)
project(ins_filter_non_holomic)

# Include InsFilterNonHolomicState dependency
include(${CMAKE_CURRENT_SOURCE_DIR}/types/CMakeLists.txt)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(Eigen3 REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(mmr_common_functions REQUIRED)

# Find include directories
include_directories(
  include
  "types/include"
)

# Adding InsFilterNonHolomic library
add_library(ins_filter_non_holomic src/InsFilterNonHolomic.cpp)

target_link_libraries(ins_filter_non_holomic
  Eigen3
)