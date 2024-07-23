#
# Copyright (c) 2020, New York University and Max Planck Gesellschaft.
#
# License BSD-3 clause
#

cmake_minimum_required(VERSION 3.10)

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
set(JRL_CMAKE_MODULES "${CMAKE_CURRENT_LIST_DIR}/cmake")
if(EXISTS "${JRL_CMAKE_MODULES}/base.cmake")
  message(STATUS "JRL cmakemodules found in 'cmake/' git submodule")
else()
  find_package(jrl-cmakemodules QUIET CONFIG)
  if(jrl-cmakemodules_FOUND)
    get_property(
      JRL_CMAKE_MODULES
      TARGET jrl-cmakemodules::jrl-cmakemodules
      PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
    message(STATUS "JRL cmakemodules found on system at ${JRL_CMAKE_MODULES}")
  elseif(${CMAKE_VERSION} VERSION_LESS "3.14.0")
    message(
      FATAL_ERROR
        "\nCan't find jrl-cmakemodules. Please either:\n"
        "  - use git submodule: 'git submodule update --init'\n"
        "  - or install https://github.com/jrl-umi3218/jrl-cmakemodules\n"
        "  - or upgrade your CMake version to >= 3.14 to allow automatic fetching\n"
    )
  else()
    message(STATUS "JRL cmakemodules not found. Let's fetch it.")
    include(FetchContent)
    FetchContent_Declare(
      "jrl-cmakemodules"
      GIT_REPOSITORY "https://github.com/jrl-umi3218/jrl-cmakemodules.git")
    FetchContent_MakeAvailable("jrl-cmakemodules")
    FetchContent_GetProperties("jrl-cmakemodules" SOURCE_DIR JRL_CMAKE_MODULES)
  endif()
endif()

# --- OPTIONS ----------------------------------------
option(BUILD_PYTHON_INTERFACE "Build the python binding" ON)
option(PYTHON_STANDARD_LAYOUT "Enable standard Python package layout" ON)
option(PYTHON_DEB_LAYOUT "Enable Debian-style Python package layout" OFF)

include("${JRL_CMAKE_MODULES}/base.cmake")
include("${JRL_CMAKE_MODULES}/boost.cmake")
include("${JRL_CMAKE_MODULES}/python.cmake")
include("${JRL_CMAKE_MODULES}/ide.cmake")
include("${JRL_CMAKE_MODULES}/apple.cmake")

compute_project_args(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})
apply_default_apple_configuration()
check_minimal_cxx_standard(11 ENFORCE)

# ----------------------------------------------------
# --- DEPENDENCIES -----------------------------------
# ----------------------------------------------------

add_project_dependency(yaml-cpp CONFIG REQUIRED)
add_project_dependency(Eigen3 REQUIRED PKG_CONFIG_REQUIRES "eigen3 >= 3.0.5")
add_project_dependency(master_board_sdk REQUIRED)

# Set component to fetch from boost Get the python interface for the bindings
if(BUILD_PYTHON_INTERFACE)
  set(PYTHON_COMPONENTS Interpreter Development.Module)
  add_project_dependency(eigenpy 2.7.10 REQUIRED PKG_CONFIG_REQUIRES
                         "eigenpy >= 2.7.10")
  set_boost_default_options()
endif(BUILD_PYTHON_INTERFACE)

# ----------------------------------------------------
# --- INCLUDE ----------------------------------------
# ----------------------------------------------------

# --- MAIN LIBRARY -------------------------------------------------------------
set(ODRI_CONTROL_INTERFACE_SRC src/joint_modules.cpp src/imu.cpp src/robot.cpp
                               src/calibration.cpp src/utils.cpp)
add_library(${PROJECT_NAME} SHARED ${ODRI_CONTROL_INTERFACE_SRC})
if(TARGET yaml-cpp::yaml-cpp)
  target_link_libraries(${PROJECT_NAME} yaml-cpp::yaml-cpp)
else()
  target_link_libraries(${PROJECT_NAME} yaml-cpp)
endif()
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

  # target_link_libraries(${PYWRAP} ${PROJECT_NAME} eigenpy::eigenpy)
  # target_link_boost_python(${PYWRAP})
  # set_target_properties(${PYWRAP} PROPERTIES INSTALL_RPATH
  #                                            "${CMAKE_INSTALL_PREFIX}/lib")
  set_target_properties(${PYWRAP} PROPERTIES SUFFIX ${PYTHON_EXT_SUFFIX})
  target_link_libraries(${PYWRAP} PUBLIC ${PROJECT_NAME} eigenpy::eigenpy)
  target_link_boost_python(${PYWRAP} PUBLIC)
  if(UNIX)
    get_relative_rpath(${${PYWRAP}_INSTALL_DIR} ${PYWRAP}_INSTALL_RPATH)
    set_target_properties(${PYWRAP} PROPERTIES INSTALL_RPATH
                                               "${${PYWRAP}_INSTALL_RPATH}")
  endif()
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
    PUBLIC CONFIG_BOLT_YAML="${PROJECT_SOURCE_DIR}/demos/config_bolt.yaml"
    PUBLIC CONFIG_TESTBENCH_YAML="${PROJECT_SOURCE_DIR}/demos/config_testbench.yaml")
  install(TARGETS ${demo_name} DESTINATION bin)

endmacro(create_demo source)

create_demo(demo_solo12)
create_demo(demo_bolt)
create_demo(demo_testbench)
