#
# Copyright (c) 2020, New York University and Max Planck Gesellschaft.
#
# License BSD-3 clause
#


CMAKE_MINIMUM_REQUIRED(VERSION 3.1)

# ----------------------------------------------------
# --- CXX FLAGS --------------------------------------
# ----------------------------------------------------

SET(CXX_DISABLE_WERROR True)
SET(CMAKE_VERBOSE_MAKEFILE True)

# These variables have to be defined before running SETUP_PROJECT
SET(PROJECT_NAME odri_control_interface)
SET(PROJECT_DESCRIPTION "Common interface for controlling robots build with the odri master board.")
SET(PROJECT_URL https://github.com/open-dynamic-robot-initiative/odri_control_interface)
set(PROJECT_USE_CMAKE_EXPORT TRUE)
SET(PROJECT_VERSION_COMPUTATION_METHODS ROS_PACKAGE_XML_FILE)

# Check if the submodule cmake have been initialized
IF(NOT EXISTS "${CMAKE_SOURCE_DIR}/cmake/base.cmake")
  MESSAGE(FATAL_ERROR "\nPlease run the following command first:\ngit submodule update --init\n")
ENDIF()

# --- OPTIONS ----------------------------------------
OPTION(BUILD_PYTHON_INTERFACE "Build the python binding" ON)
OPTION(PYTHON_STANDARD_LAYOUT "Enable standard Python package layout" ON)
OPTION(PYTHON_DEB_LAYOUT "Enable Debian-style Python package layout" OFF)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/python.cmake)
INCLUDE(cmake/ide.cmake)

COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX)
PROJECT(${PROJECT_NAME} ${PROJECT_ARGS})
CHECK_MINIMAL_CXX_STANDARD(11 ENFORCE)

# ----------------------------------------------------
# --- DEPENDENCIES -----------------------------------
# ----------------------------------------------------

ADD_PROJECT_DEPENDENCY(Eigen3 REQUIRED PKG_CONFIG_REQUIRES "eigen3 >= 3.0.5")
ADD_PROJECT_DEPENDENCY(master_board_sdk REQUIRED)

# Set component to fetch from boost
# Get the python interface for the bindings
IF(BUILD_PYTHON_INTERFACE)
  FINDPYTHON(REQUIRED)
  SEARCH_FOR_BOOST_PYTHON(REQUIRED)
  ADD_PROJECT_DEPENDENCY(eigenpy 2.5.0 REQUIRED PKG_CONFIG_REQUIRES "eigenpy >= 2.5.0")
ENDIF(BUILD_PYTHON_INTERFACE)

# ----------------------------------------------------
# --- INCLUDE ----------------------------------------
# ----------------------------------------------------

# --- MAIN LIBRARY -------------------------------------------------------------
SET(ODRI_CONTROL_INTERFACE_SRC
    src/joint_modules.cpp
)
ADD_LIBRARY(${PROJECT_NAME} SHARED
  ${ODRI_CONTROL_INTERFACE_SRC}
)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR})
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${MASTER_BOARD_SDK_INCLUDE_DIRS})
# TARGET_LINK_LIBRARIES(${PROJECT_NAME} pthread)
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include>)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${TARGETS_EXPORT_NAME}
  DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

# --- BINDINGS -----------------------------------------------------------------

IF(BUILD_PYTHON_INTERFACE)
  # --- Setup the wrapper name and source files --- #
  SET(PYWRAP ${PROJECT_NAME}_pywrap)
  SET(${PYWRAP}_HEADERS
    srcpy/bindings.h
  )
  SET(${PYWRAP}_SOURCES
    srcpy/bindings.cpp
  )

  # --- Build the wrapper library --- #
  SET(${PYWRAP}_INSTALL_DIR ${PYTHON_SITELIB})
  ADD_LIBRARY(${PYWRAP} SHARED ${${PYWRAP}_SOURCES} ${${PYWRAP}_HEADERS})
  TARGET_INCLUDE_DIRECTORIES(${PYWRAP} SYSTEM PRIVATE ${PYTHON_INCLUDE_DIR})
  TARGET_LINK_LIBRARIES(${PYWRAP} ${PROJECT_NAME} eigenpy::eigenpy master_board_sdk::master_board_sdk)
  TARGET_LINK_BOOST_PYTHON(${PYWRAP})
  SET_TARGET_PROPERTIES(${PYWRAP} PROPERTIES
    INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
  INSTALL(TARGETS ${PYWRAP} DESTINATION ${${PYWRAP}_INSTALL_DIR})

  # --- Allow to do: make python --- #
  ADD_CUSTOM_TARGET(python)
  SET_TARGET_PROPERTIES(python PROPERTIES EXCLUDE_FROM_DEFAULT_BUILD True)
  ADD_DEPENDENCIES(python ${PYWRAP})

ENDIF(BUILD_PYTHON_INTERFACE)


# --- CHECK DEMOS -----------------------------------------------------------
ADD_EXECUTABLE(demo_solo12 demos/demo_solo12.cpp)
TARGET_LINK_LIBRARIES(demo_solo12 ${PROJECT_NAME})
INSTALL(TARGETS demo_solo12 DESTINATION bin)
