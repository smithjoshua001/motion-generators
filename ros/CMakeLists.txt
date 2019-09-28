string(REPLACE ":" ";" newstring "$ENV{CMAKE_MODULE_PATH}")
set(CMAKE_MODULE_PATH "${newstring}")

find_package(catkin REQUIRED
   COMPONENTS
   loco
   loco_ros_anymal
   anymal_msgs
   kindr_ros
   parameter_handler
    roscpp
    std_msgs
    geometry_msgs
    robot_state_publisher
    tf2
    tf2_ros
    anymal_model
    anymal_description
    tf2_geometry_msgs
    sensor_msgs
    )
        # Attempt to find catkinized kindr
find_package(kindr QUIET)
if(NOT kindr_FOUND)
  # Attempt to find package-based kindr
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(kindr kindr REQUIRED)
endif()
pkg_check_modules(JSON11 REQUIRED json11)
find_library(json11_LIBRARY NAMES ${JSON11_LIBRARIES} HINTS ${JSON11_LIBRARY_DIRS})
set(JSON11_LIBRARIES "${json11_LIBRARY}") 
    catkin_package(INCLUDE_DIRS
    include
    ros/include
    LIBRARIES
    ${CMAKE_PROJECT_NAME}
    CATKIN_DEPENDS
    loco
    loco_ros_anymal
    parameter_handler
   anymal_msgs
   kindr_ros
    roscpp
    std_msgs
    geometry_msgs
    robot_state_publisher
    tf2
    tf2_ros
    kindr
    anymal_model
    anymal_description
    tf2_geometry_msgs
    sensor_msgs
    DEPENDS 
    JSON11)



include_directories(
include
ros/include
${kindr_INCLUDE_DIRS}
${catkin_INCLUDE_DIRS}
${json11_INCLUDE_DIRS}
)

list(APPEND DEPENDENCY_LIBRARIES ${catkin_LIBRARIES})

add_library(
  ${CMAKE_PROJECT_NAME}
  ${PROJECT_SOURCE_DIR}/ros/src/TaskDynamicAttractorRos.cpp
  ${PROJECT_SOURCE_DIR}/ros/src/HumanInputTrajectoryRos.cpp
)
add_dependencies(
  ${CMAKE_PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${CMAKE_PROJECT_NAME} ${json11_LIBRARIES} ${DEPENDENCY_LIBRARIES})

install(DIRECTORY ros/include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})


install(TARGETS ${CMAKE_PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
