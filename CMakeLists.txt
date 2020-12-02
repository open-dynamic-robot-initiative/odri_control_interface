#
# Copyright (c) 2019, New York University and Max Planck Gesellschaft.
#
# License BSD-3 clause
#

#
# set up the project
#
cmake_minimum_required(VERSION 3.10.2)

project(odri_control_interface)

# Using C++17
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

#
# Dependencies
#

# depend on ament macros
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Usual dependencies
find_package(mpi_cmake_modules REQUIRED)
find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Threads REQUIRED)
find_package(master_board_sdk REQUIRED)

#
# Setup the environment for dependencies
#
ament_export_dependencies(mpi_cmake_modules master_board_sdk Threads pybind11)
ament_export_interfaces(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
set(all_targets)

#
# Add the main library
#

# Create the CMake target
add_library(${PROJECT_NAME} INTERFACE)
# Add the include dependencies
target_include_directories(
  ${PROJECT_NAME}
  INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>)
# Link the dependencies
target_link_libraries(${PROJECT_NAME} INTERFACE Threads::Threads)
target_link_libraries(${PROJECT_NAME}
                      INTERFACE master_board_sdk::master_board_sdk)
target_link_libraries(${PROJECT_NAME} ${master_board_sdk_LIBRARIES})
# For the installation
list(APPEND all_targets ${PROJECT_NAME})

#
# Demos.
#
macro(add_odri_control_interface_demo demo_name)
  # set(demo_target_name ${PROJECT_NAME}_${demo_name})
  set(demo_target_name ${demo_name})
  # add the executable.
  add_executable(${demo_target_name} demos/${demo_name}.cpp)
  # save the name for installation
  list(APPEND all_targets ${demo_target_name})
  # dependencies
  target_include_directories(
    ${demo_target_name}
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  target_link_libraries(${demo_target_name} ${PROJECT_NAME})
endmacro(add_odri_control_interface_demo target_name)

add_odri_control_interface_demo(demo_solo12)
add_odri_control_interface_demo(demo_solo12_pd)
# add_real_time_tools_demo(demo_realtime_strict_check)
# add_real_time_tools_demo(demo_spinner) add_real_time_tools_demo(demo_timing)
# add_real_time_tools_demo(demo_thread)
# add_real_time_tools_demo(demo_usb_stream_imu_3DM_GX3_25)
# add_real_time_tools_demo(demo_checkpoint_timer)

#
# Executables.
#

# set(EXE1 realtime_test) add_executable(${EXE1} src/bin/realtime_test.cpp)
# target_link_libraries(${EXE1} ${PROJECT_NAME} ${catkin_LIBRARIES})

# set(EXE2 realtime_test_display) add_executable(${EXE2}
# src/bin/realtime_test_display.cpp) target_link_libraries(${EXE2}
# ${PROJECT_NAME} ${catkin_LIBRARIES})

#
# Python wrapper.
#
# add_library(${PROJECT_NAME}_cpp_bindings MODULE srcpy/real_time_tools.cpp)
# target_link_libraries(${PROJECT_NAME}_cpp_bindings PRIVATE pybind11::module)
# target_link_libraries(${PROJECT_NAME}_cpp_bindings PRIVATE
# ${PYTHON_LIBRARIES}) target_link_libraries(${PROJECT_NAME}_cpp_bindings
# PRIVATE ${PROJECT_NAME}) set_target_properties(${PROJECT_NAME}_cpp_bindings
# PROPERTIES PREFIX "" SUFFIX "${PYTHON_MODULE_EXTENSION}")
# target_include_directories( ${PROJECT_NAME}_cpp_bindings PUBLIC
# $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# $<INSTALL_INTERFACE:include> SYSTEM PUBLIC ${PYTHON_INCLUDE_DIRS})
# _ament_cmake_python_get_python_install_dir() install(TARGETS
# ${PROJECT_NAME}_cpp_bindings DESTINATION ${PYTHON_INSTALL_DIR}) install(FILES
# srcpy/__init__.py DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME})

#
# Install the package
#
install(DIRECTORY include/ DESTINATION include)

install(
  TARGETS ${all_targets}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)

#
# Building documentation.
#
add_documentation()

#
# Export as an ament_package
#
ament_package()
