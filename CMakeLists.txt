cmake_minimum_required(VERSION 3.0)
set(PROJECT_NAME imuSimWithPointLine)

project(${PROJECT_NAME})

# SET(CMAKE_BUILD_TYPE Debug) # Release, RelWithDebInfo , Debug

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

set(CMAKE_CXX_STANDARD 17)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(src)

add_executable(data_gen src/main.cpp src/utilities.cpp src/imu.cpp)
