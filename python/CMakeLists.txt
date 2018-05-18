
include (UseSWIG)

include(SwigPython)

add_swig_python_module(PulsatingJointTrajectory PulsatingJointTrajectory.i
                       CPLUSPLUS
                       INCLUDE_DIRS ${DEPENDENCY_INCLUDE_DIRS}
                       #LINK_LIBRARIES inverter
                       SWIG_INCLUDE_DIRS ${SWIG_DIR}/../python ${SWIG_DIR}/../common
DESTINATION ${CMAKE_INSTALL_LIBDIR}/python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/site-packages/PulsatingJointTrajectory)
add_swig_python_module(ModifiedFourierTrajectory ModifiedFourierTrajectory.i
                       CPLUSPLUS
                       INCLUDE_DIRS ${DEPENDENCY_INCLUDE_DIRS}
                       #LINK_LIBRARIES inverter
                       SWIG_INCLUDE_DIRS ${SWIG_DIR}/../python ${SWIG_DIR}/../common
DESTINATION ${CMAKE_INSTALL_LIBDIR}/python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/site-packages/ModifiedFourierTrajectory)