
string(REPLACE ":" ";" newstring "$ENV{CMAKE_MODULE_PATH}")
set(CMAKE_MODULE_PATH "${newstring}")


# find_package(OROCOS-RTT REQUIRED COMPONENTS rtt-scripting rtt-typekit)
# if (NOT OROCOS-RTT_FOUND)
#   message (FATAL_ERROR "\nCould not find Orocos-RTT. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
# endif()

find_package(rtt-core-extensions REQUIRED)
include_directories(${RTT-CORE-EXTENSIONS_INCLUDE_DIRS})
link_directories(${RTT-CORE-EXTENSIONS_LIBRARY_DIRS})

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

find_package(RST-RT REQUIRED)
message(STATUS "RST-RT version: ${RST-RT_VERSION}")
include_directories(BEFORE SYSTEM ${RST-RT_INCLUDE_DIRS})
add_definitions(${RST-RT_CFLAGS})
link_directories(${RST-RT_LIBRARY_DIR} ${RST-RT_LIBRARY_DIRS})

# add_definitions(-DTIMING)

include_directories(include
${USE_OROCOS_INCLUDE_DIRS}
${OROCOS-RTT_INCLUDE_DIRS}
${RST-RT_INCLUDE_DIRS})

list(APPEND DEPENDENCY_LIBRARIES ${OROCOS-RTT_LIBRARIES} ${OROCOS-RTT_RTT-SCRIPTING_LIBRARY} ${OROCOS-RTT_TYPEKIT_LIBRARIES} ${USE_OROCOS_LIBRARIES} ${RST-RT_LIBRARIES})

# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#


# Creates a component library libexample-<target>.so
# and installs in the directory lib/orocos/example/
#
# orocos_component(${CMAKE_PROJECT_NAME} simple-rtt-component.hpp simple-rtt-component.cpp) # ...you may add multiple source files
orocos_component(RTT_${CMAKE_PROJECT_NAME}
${PROJECT_SOURCE_DIR}/orocos/src/DynamicAttractor.cpp
${PROJECT_SOURCE_DIR}/orocos/src/TaskDynamicAttractor.cpp
) # ...you may add multiple source files

set_target_properties(RTT_${CMAKE_PROJECT_NAME} PROPERTIES COMPILE_DEFINITIONS RTT_COMPONENT)
#
# You may add multiple orocos_component statements.

#
# Building a typekit (recommended):
#
# Creates a typekit library libexample-types-<target>.so
# and installs in the directory lib/orocos/example/types/
#
#orocos_typegen_headers(example-types.hpp) # ...you may add multiple header files
#
# You may only have *ONE* orocos_typegen_headers statement !

#
# Building a normal library (optional):
#
# Creates a library libsupport-<target>.so and installs it in
# lib/
#
#orocos_library(motion-generators PulsatingJointTrajectory.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_library statements.


#
# Building a Plugin or Service (optional):
#
# Creates a plugin library libexample-service-<target>.so or libexample-plugin-<target>.so
# and installs in the directory lib/orocos/example/plugins/
#
# Be aware that a plugin may only have the loadRTTPlugin() function once defined in a .cpp file.
# This function is defined by the plugin and service CPP macros.
#
#orocos_service(example-service example-service.cpp) # ...only one service per library !
#orocos_plugin(example-plugin example-plugin.cpp) # ...only one plugin function per library !
#
# You may add multiple orocos_plugin/orocos_service statements.

# target_link_libraries()
target_link_libraries(RTT_${CMAKE_PROJECT_NAME} ${DEPENDENCY_LIBRARIES})

#
# Additional headers (not in typekit):
#
# Installs in the include/orocos/example/ directory
#
# orocos_install_headers( example-component.hpp ) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

#
# Generates and installs our package. Must be the last statement such
# that it can pick up all above settings.
#
orocos_generate_package(INCLUDE_DIRS include)