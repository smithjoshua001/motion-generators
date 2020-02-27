
find_package(PkgConfig REQUIRED)
pkg_search_module(DynamicsLib REQUIRED DynamicsLib)
include_directories(${DynamicsLib_INCLUDE_DIRS})
link_directories(${DynamicsLib_LIBRARY_DIR} ${DynamicsLib_LIBRARY_DIRS})

find_package (Threads)
include_directories(${Threads_INCLUDE_DIRS})


add_executable(${CMAKE_PROJECT_NAME}_NPIdealRobotTestExciting NonParametricTestsIdealRobotExciting.cpp)

target_link_libraries(${CMAKE_PROJECT_NAME}_NPIdealRobotTestExciting ${CMAKE_THREAD_LIBS_INIT} ${Threads_LIBRARIES} ${DEPENDENCY_LIBRARIES_FULL}  ${DynamicsLib_LIBRARIES})

install(TARGETS ${CMAKE_PROJECT_NAME}_NPIdealRobotTestExciting RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

