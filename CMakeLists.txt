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

# copy the ../aircraft and ../engine directories to the build directory
file(COPY aircraft DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
file(COPY engine DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
file(COPY scripts DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

# Link the JSBSim library
target_link_libraries(rocket_sim ${JSBSIM_LIBRARIES})
