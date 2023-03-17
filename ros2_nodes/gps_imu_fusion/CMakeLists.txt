cmake_minimum_required(VERSION 3.5)
project(gps_imu_fusion)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Add compile option for insFilterNonHolonomic library
add_link_options("-lins_filter_non_holonomic")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# find include directories
include_directories(
  include
  "/home/tommaso/Documents/Tesi/fusion_odom/CPP_Implementation/insFilterNonHolonomic/include"
  "/home/tommaso/Documents/Tesi/fusion_odom/CPP_Implementation/insFilterNonHolonomic/types/include"
)

add_library(${PROJECT_NAME}_class src/${PROJECT_NAME}.cpp)

ament_target_dependencies(${PROJECT_NAME}_class
  rclcpp
  sensor_msgs
  geometry_msgs
)

target_link_libraries(${PROJECT_NAME}_class
  ins_filter_non_holonomic
)

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_class
)

# Install target
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config directory
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
