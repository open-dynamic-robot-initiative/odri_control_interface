#
# Copyright (c) 2020, New York University and Max Planck Gesellschaft.
#
# License BSD-3 clause
#

cmake_minimum_required(VERSION 3.1)

# ----------------------------------------------------
# --- CXX FLAGS --------------------------------------
# ----------------------------------------------------

set(CXX_DISABLE_WERROR True)
set(CMAKE_VERBOSE_MAKEFILE True)

# These variables have to be defined before running SETUP_PROJECT
set(PROJECT_NAME odri_control_interface)
set(PROJECT_DESCRIPTION
    "Common interface for controlling robots build with the odri master board.")
set(PROJECT_URL
    https://github.com/open-dynamic-robot-initiative/odri_control_interface)
set(PROJECT_USE_CMAKE_EXPORT TRUE)

# Check if the submodule cmake have been initialized
if(NOT EXISTS "${CMAKE_SOURCE_DIR}/cmake/base.cmake")
  message(
    FATAL_ERROR
      "\nPlease run the following command first:\ngit submodule update --init\n"
  )
endif()

# --- OPTIONS ----------------------------------------
option(BUILD_PYTHON_INTERFACE "Build the python binding" ON)
option(PYTHON_STANDARD_LAYOUT "Enable standard Python package layout" ON)
option(PYTHON_DEB_LAYOUT "Enable Debian-style Python package layout" OFF)

include(cmake/base.cmake)
include(cmake/boost.cmake)
include(cmake/python.cmake)
include(cmake/ide.cmake)

compute_project_args(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})
check_minimal_cxx_standard(11 ENFORCE)

# ----------------------------------------------------
# --- DEPENDENCIES -----------------------------------
# ----------------------------------------------------

add_project_dependency(yaml-cpp CONFIG REQUIRED)
add_project_dependency(Eigen3 REQUIRED PKG_CONFIG_REQUIRES "eigen3 >= 3.0.5")
add_project_dependency(master_board_sdk REQUIRED)

# Set component to fetch from boost Get the python interface for the bindings
if(BUILD_PYTHON_INTERFACE)
  findpython(REQUIRED)
  search_for_boost_python(REQUIRED)
  add_project_dependency(eigenpy 2.5.0 REQUIRED PKG_CONFIG_REQUIRES
                         "eigenpy >= 2.5.0")
endif(BUILD_PYTHON_INTERFACE)

# ----------------------------------------------------
# --- INCLUDE ----------------------------------------
# ----------------------------------------------------

# --- MAIN LIBRARY -------------------------------------------------------------
set(ODRI_CONTROL_INTERFACE_SRC src/joint_modules.cpp src/imu.cpp src/robot.cpp
                               src/calibration.cpp src/utils.cpp src/powerboard.cpp)
add_library(${PROJECT_NAME} SHARED ${ODRI_CONTROL_INTERFACE_SRC})
target_link_libraries(${PROJECT_NAME} ${YAML_CPP_LIBRARIES})
target_link_libraries(${PROJECT_NAME} master_board_sdk::master_board_sdk)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include>)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${TARGETS_EXPORT_NAME}
  DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

# --- BINDINGS -----------------------------------------------------------------

if(BUILD_PYTHON_INTERFACE)
  # --- Setup the wrapper name and source files --- #
  set(PYWRAP ${PROJECT_NAME}_pywrap)
  set(${PYWRAP}_HEADERS srcpy/bindings.h)
  set(${PYWRAP}_SOURCES srcpy/bindings.cpp)

  # --- Build the wrapper library --- #
  set(${PYWRAP}_INSTALL_DIR ${PYTHON_SITELIB})
  add_library(${PYWRAP} SHARED ${${PYWRAP}_SOURCES} ${${PYWRAP}_HEADERS})
  target_include_directories(${PYWRAP} SYSTEM PRIVATE ${PYTHON_INCLUDE_DIR})
  target_link_libraries(${PYWRAP} ${PROJECT_NAME} eigenpy::eigenpy)
  target_link_boost_python(${PYWRAP})
  set_target_properties(${PYWRAP} PROPERTIES INSTALL_RPATH
                                             "${CMAKE_INSTALL_PREFIX}/lib")
  install(TARGETS ${PYWRAP} DESTINATION ${${PYWRAP}_INSTALL_DIR})

  # --- Allow to do: make python --- #
  add_custom_target(python)
  set_target_properties(python PROPERTIES EXCLUDE_FROM_DEFAULT_BUILD True)
  add_dependencies(python ${PYWRAP})

endif(BUILD_PYTHON_INTERFACE)

# --- CHECK DEMOS -----------------------------------------------------------
macro(create_demo source)

  set(demo_name ${PROJECT_NAME}_${source})
  add_executable(${demo_name} demos/${source}.cpp)
  target_link_libraries(${demo_name} ${PROJECT_NAME})
  target_compile_definitions(
    ${demo_name}
    PUBLIC CONFIG_SOLO12_YAML="${PROJECT_SOURCE_DIR}/demos/config_solo12.yaml"
    PUBLIC CONFIG_TESTBENCH_YAML="${PROJECT_SOURCE_DIR}/demos/config_testbench.yaml")
  install(TARGETS ${demo_name} DESTINATION bin)

endmacro(create_demo source)

create_demo(demo_solo12)
create_demo(demo_testbench)
