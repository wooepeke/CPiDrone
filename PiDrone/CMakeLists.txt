cmake_minimum_required(VERSION 3.10)
project(PiDroneProject)

# Specify C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add executable target
add_executable(PiDrone
    main.cpp
    PiDrone.cpp
)
