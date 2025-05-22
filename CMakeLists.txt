cmake_minimum_required(VERSION 3.12)
project(RocketSimulation)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find JSBSim
find_path(JSBSIM_INCLUDE_DIR 
    NAMES FGFDMExec.h
    PATHS 
        ${CMAKE_CURRENT_SOURCE_DIR}/external/jsbsim/src
        /usr/local/include/JSBSim
        /usr/include/JSBSim
    PATH_SUFFIXES src
)

find_library(JSBSIM_LIBRARY
    NAMES JSBSim jsbsim
    PATHS 
        ${CMAKE_CURRENT_SOURCE_DIR}/external/jsbsim/build/src
        /usr/local/lib
        /usr/lib
)

if(JSBSIM_INCLUDE_DIR AND JSBSIM_LIBRARY)
    message(STATUS "Found JSBSim: ${JSBSIM_LIBRARY}")
    message(STATUS "JSBSim headers: ${JSBSIM_INCLUDE_DIR}")
else()
    message(FATAL_ERROR "JSBSim not found. Please build JSBSim first.")
endif()

# Create the executable
add_executable(rocket_sim rocket_sim.cpp)

# Link JSBSim
target_include_directories(rocket_sim PRIVATE ${JSBSIM_INCLUDE_DIR})
target_link_libraries(rocket_sim ${JSBSIM_LIBRARY})

# Copy aircraft configuration to build directory
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/aircraft 
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

# Set runtime path for JSBSim
if(UNIX AND NOT APPLE)
    set_target_properties(rocket_sim PROPERTIES
        INSTALL_RPATH_USE_LINK_PATH TRUE)
endif()
