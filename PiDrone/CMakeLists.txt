cmake_minimum_required(VERSION 3.10)
project(PiDroneProject)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Include directories (add the 'include' folder)
include_directories(${PROJECT_SOURCE_DIR}/include)

# Add source files
add_executable(PiDrone
    src/main.cpp
    src/PiDrone.cpp
    src/MPU6050.cpp
    src/TCPServer.cpp
)

# Link necessary libraries
target_link_libraries(PiDrone PRIVATE i2c pthread pigpio fmt)
