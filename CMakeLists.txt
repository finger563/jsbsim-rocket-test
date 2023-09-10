cmake_minimum_required(VERSION 3.0)
project(RocketSimulation)

set(CMAKE_CXX_STANDARD 20)

# jsbsim include directories are in include
# jsbsim library files are in lib
set(JSBSIM_INCLUDE_DIRS include)
link_directories(lib)
set(JSBSIM_LIBRARIES libJSBSim.dylib)

# Include JSBSim headers
include_directories(${JSBSIM_INCLUDE_DIRS})

add_executable(rocket_sim rocket_sim.cpp)

# Link the JSBSim library
target_link_libraries(rocket_sim ${JSBSIM_LIBRARIES})
