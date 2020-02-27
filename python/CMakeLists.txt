
include (UseSWIG)

include(SwigPython)

message(STATUS "JSON11: ${DEPENDENCY_LIBRARIES} ${JSON11_LIBRARY}")
add_swig_python_module(ModifiedFourierTrajectory ModifiedFourierTrajectory.i
                       CPLUSPLUS
                       INCLUDE_DIRS "${DEPENDENCY_INCLUDE_DIRS};${DynamicsLib_INCLUDE_DIRS};${CMAKE_CURRENT_SOURCE_DIR}/include;${RapidJSON_INCLUDE_DIRS}"
                        LINK_LIBRARIES ${DEPENDENCY_LIBRARIES}
                       SWIG_INCLUDE_DIRS ${SWIG_DIR}/../python ${SWIG_DIR}/../common
DESTINATION ${CMAKE_INSTALL_LIBDIR}/python2.7/site-packages/)

find_package(PkgConfig REQUIRED)
pkg_search_module(DynamicsLib DynamicsLib)
if(${DynamicsLib_FOUND})
    include_directories(${DynamicsLib_INCLUDE_DIRS})
    set(LIBDIRS "")
    foreach(TempLib ${DynamicsLib_LIBRARY_DIRS})
        set(LIBDIRS ${LIBDIRS} ${TempLib})
    endforeach(TempLib)
    set(DynamicsLibraries "")
    foreach(DLib ${DynamicsLib_LIBRARIES})
    find_library(${DLib}_LIBRARY ${DLib} HINTS ${LIBDIRS})
    set(DynamicsLibraries ${DynamicsLibraries} ${${DLib}_LIBRARY})
    endforeach(DLib)
    message(STATUS "${DynamicsLib_INCLUDE_DIR} =====================${DynamicsLibraries}==========================")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DLOG_DEBUG_ON=3 -O3")

    add_swig_python_module(RegressorCostFunction RegressorCostFunction.i
                        CPLUSPLUS
                        INCLUDE_DIRS "${DEPENDENCY_INCLUDE_DIRS};${DynamicsLib_INCLUDE_DIRS};${CMAKE_CURRENT_SOURCE_DIR}/include;${RapidJSON_INCLUDE_DIRS}"
                        LINK_LIBRARIES ${DynamicsLibraries} ${DEPENDENCY_LIBRARIES}
                        SWIG_INCLUDE_DIRS ${SWIG_DIR}/../python ${SWIG_DIR}/../common
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/python2.7/site-packages/)

endif(${DynamicsLib_FOUND})


file(GLOB_RECURSE ${CMAKE_PROJECT_NAME}_PYTHON
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.py"
)
file(GLOB_RECURSE ${CMAKE_PROJECT_NAME}_PYTHON
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.py"
)

file(GLOB_RECURSE ${CMAKE_PROJECT_NAME}_PYTHON_BIN
    "${CMAKE_CURRENT_SOURCE_DIR}/../scripts/*.py"
)

INSTALL (
  FILES  ${${CMAKE_PROJECT_NAME}_PYTHON}
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/python2.7/site-packages/
  )

  INSTALL (
  FILES  ${${CMAKE_PROJECT_NAME}_PYTHON_BIN}
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
  DESTINATION ${CMAKE_INSTALL_BINDIR}/
  )
