
include (UseSWIG)

include(SwigPython)

add_swig_python_module(PulsatingJointTrajectory PulsatingJointTrajectory.i
                       CPLUSPLUS
                       INCLUDE_DIRS ${DEPENDENCY_INCLUDE_DIRS}
                       LINK_LIBRARIES ${json11_LIBRARIES}
                       SWIG_INCLUDE_DIRS ${SWIG_DIR}/../python ${SWIG_DIR}/../common
DESTINATION ${CMAKE_INSTALL_LIBDIR}/python2.7/site-packages/)
add_swig_python_module(ModifiedFourierTrajectory ModifiedFourierTrajectory.i
                       CPLUSPLUS
                       INCLUDE_DIRS ${DEPENDENCY_INCLUDE_DIRS}
                       LINK_LIBRARIES ${json11_LIBRARIES}
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
    message(STATUS "${DynamicsLib_INCLUDE_DIR} ===============================================")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DLOG_DEBUG_ON=3 -O3")
    # function(pkg_check_variable _pkg _name)
    #     string(TOUPPER ${_pkg} _pkg_upper)
    #     string(TOUPPER ${_name} _name_upper)
    #     string(REPLACE "-" "_" _pkg_upper ${_pkg_upper})
    #     string(REPLACE "-" "_" _name_upper ${_name_upper})
    #     set(_output_name "${_pkg_upper}_${_name_upper}")

    #     execute_process(COMMAND ${PKG_CONFIG_EXECUTABLE} --variable=${_name} ${_pkg}
    #                     OUTPUT_VARIABLE _pkg_result
    #                     OUTPUT_STRIP_TRAILING_WHITESPACE)

    #     set("${_output_name}" "${_pkg_result}" CACHE STRING "pkg-config variable ${_name} of ${_pkg}")
    # endfunction()
    # pkg_check_variable(DynamicsLib includedir)
    # message(STATUS "${DYNAMICSLIB_INCLUDEDIR} ===============================================")

    # CONFIGURE_FILE (
    #     ${CMAKE_CURRENT_SOURCE_DIR}/RegressorCostFunction.i.in
    #     ${CMAKE_CURRENT_SOURCE_DIR}/RegressorCostFunction.i @ONLY
    # )

    add_swig_python_module(RegressorCostFunction RegressorCostFunction.i
                        CPLUSPLUS
                        INCLUDE_DIRS "${DEPENDENCY_INCLUDE_DIRS};${DynamicsLib_INCLUDE_DIRS};${CMAKE_CURRENT_SOURCE_DIR}/include;${RapidJSON_INCLUDE_DIRS}"
                        LINK_LIBRARIES ${DynamicsLibraries} ${json11_LIBRARIES}
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
