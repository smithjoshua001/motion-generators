#
# This is based on orocos sample cmakelists, modified slightly
# The find_package macro for Orocos-RTT works best with
# cmake >= 2.8.3
#
cmake_minimum_required(VERSION 2.8.3)

#
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
#
project(MotionGenerators)
INCLUDE(GNUInstallDirs)

set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14  -Wall")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
string(REPLACE ":" ";" newstring "$ENV{CMAKE_MODULE_PATH}")
set(CMAKE_MODULE_PATH "${newstring}")
# include(vscode OPTIONAL)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


find_package(RapidJSON REQUIRED)
include_directories(BEFORE SYSTEM ${RapidJSON_INCLUDE_DIRS})
link_directories(${RapidJSON_LIBRARY_DIR} ${RapidJSON_LIBRARY_DIRS})

list(APPEND DEPENDENCY_LIBRARIES ${RapidJSON_LIBRARIES} ${EIGEN3_LIBRARIES})
list(APPEND DEPENDENCY_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/include" ${RapidJSON_INCLUDE_DIRS} ${RapidJSON_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR})

# add_definitions(-DTIMING)

file(GLOB_RECURSE ${CMAKE_PROJECT_NAME}_HEADERS
    "${PROJECT_SOURCE_DIR}/include/*.h"
)
message(STATUS ${CMAKE_INSTALL_INCLUDEDIR}/pkgconfig)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

CONFIGURE_FILE (
  ${CMAKE_CURRENT_SOURCE_DIR}/${CMAKE_PROJECT_NAME}.pc.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.pc @ONLY
  )
  
INSTALL (
  FILES  ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.pc
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig
  )

find_package(SWIG REQUIRED)
if (NOT SWIG_FOUND)
message (ERROR "\nCould not find SWIG.")
else()
  add_subdirectory(python)
endif()

find_package(OROCOS-RTT REQUIRED COMPONENTS rtt-scripting rtt-typekit)
if (NOT OROCOS-RTT_FOUND)
  message (ERROR "\nCould not find Orocos-RTT. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
else()
  add_subdirectory(orocos)
endif()
include(vscode OPTIONAL)