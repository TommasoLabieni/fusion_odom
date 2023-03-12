cmake_minimum_required(VERSION 3.5)
project(ins_filter_non_holomic_types)

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


# Adding Quarternion library
add_library(quaternion_lib types/src/Quaternion.cpp)
# Target include directory specific to Quaternion
target_include_directories(quaternion_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/types/include>
    $<INSTALL_INTERFACE:include>
)

# Adding Gyroscope bias library
add_library(gyrobias_lib types/src/GyroscopeBias.cpp)
# Target include directory specific to Gyroscope bias
target_include_directories(gyrobias_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/types/include>
    $<INSTALL_INTERFACE:include>
)

# Adding NEDPosition library
add_library(nedposition_lib types/src/NEDPosition.cpp)
# Target include directory specific to NEDPosition
target_include_directories(nedposition_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/types/include>
    $<INSTALL_INTERFACE:include>
)

# Adding NEDVelocities library
add_library(nedvelocities_lib types/src/NEDVelocities.cpp)
# Target include directory specific to NEDVelocities
target_include_directories(nedvelocities_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/types/include>
    $<INSTALL_INTERFACE:include>
)

# Adding Accelerometer bias library
add_library(accelbias_lib types/src/AccelerometerBias.cpp)
# Target include directory specific to Accelerometer bias
target_include_directories(accelbias_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/types/include>
    $<INSTALL_INTERFACE:include>
)

# Adding InsFilterNonHolomicState library
add_library(ins_filter_non_holomic_state types/src/InsFilterNonHolomicState.cpp)
# Target include directory specific to InsFilterNonHolomicState
target_include_directories(ins_filter_non_holomic_state PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/types/include>
    $<INSTALL_INTERFACE:include>
)