
string(REPLACE ":" ";" newstring "$ENV{CMAKE_MODULE_PATH}")
set(CMAKE_MODULE_PATH "${newstring}")

find_package(OROCOS-RTT REQUIRED COMPONENTS rtt-scripting rtt-typekit)

find_package(rtt-core-extensions REQUIRED)
include_directories(${RTT-CORE-EXTENSIONS_INCLUDE_DIRS})
link_directories(${RTT-CORE-EXTENSIONS_LIBRARY_DIRS})


include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

find_package(RST-RT REQUIRED)
message(STATUS "RST-RT version: ${RST-RT_VERSION}")
include_directories(BEFORE SYSTEM ${RST-RT_INCLUDE_DIRS})
add_definitions(${RST-RT_CFLAGS})
link_directories(${RST-RT_LIBRARY_DIR} ${RST-RT_LIBRARY_DIRS})
#
orocos_use_package(CosimaUtilities-orocos OROCOS_ONLY REQUIRED VERBOSE)
include_directories(${CosimaUtilities-orocos_INCLUDE_DIRS})
message(STATUS "COSIMAUTILITIES INCLUDE: ${CosimaUtilities-orocos_INCLUDE_DIRS}")

# add_definitions(-DTIMING)

include_directories(
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/orocos/include
include
${USE_OROCOS_INCLUDE_DIRS}
${OROCOS-RTT_INCLUDE_DIRS}
${DEPENDENCY_INCLUDE_DIRS}
${RapidJSON_INCLUDE_DIRS}
${RST-RT_INCLUDE_DIRS})

list(APPEND DEPENDENCY_LIBRARIES ${OROCOS-RTT_LIBRARIES} ${OROCOS-RTT_RTT-SCRIPTING_LIBRARY} ${OROCOS-RTT_TYPEKIT_LIBRARIES} ${USE_OROCOS_LIBRARIES} ${RST-RT_LIBRARIES})


orocos_component(RTT_${CMAKE_PROJECT_NAME}
${PROJECT_SOURCE_DIR}/orocos/src/JointDynamicAttractorOrocos.cpp
#${PROJECT_SOURCE_DIR}/orocos/src/TaskDynamicAttractor.cpp
${PROJECT_SOURCE_DIR}/orocos/src/ModifiedFourierTrajectoryOrocos.cpp
${PROJECT_SOURCE_DIR}/orocos/src/SineWaveJointOrocos.cpp
${PROJECT_SOURCE_DIR}/orocos/src/CJDAOrocos.cpp
)

set_target_properties(RTT_${CMAKE_PROJECT_NAME} PROPERTIES COMPILE_DEFINITIONS RTT_COMPONENT)
target_link_libraries(RTT_${CMAKE_PROJECT_NAME} ${DEPENDENCY_LIBRARIES})
orocos_generate_package(INCLUDE_DIRS include)
