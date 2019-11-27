cmake_minimum_required(VERSION 2.8.3)
project(tesseract_python)

# Read version
FILE (STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/package.xml" tesseract_python_version1 REGEX "<version>[0-9]+\\.[0-9]+\\.[0-9]+</version>")
IF ("${tesseract_python_version1}" STREQUAL "" )
MESSAGE(FATAL_ERROR "Could not read Robot Raconteur version.")
ENDIF()
STRING (REGEX REPLACE "<version>([0-9]+\\.[0-9]+\\.[0-9]+)</version>" "\\1" tesseract_python_version ${tesseract_python_version1})
MESSAGE(STATUS "tesseract_python version: ${tesseract_python_version}")

if (NOT MSVC)
  #add_compile_options(-std=c++11 -Wall -Wextra -Wno-write-strings -Wno-unused-paramete -Wno-suggest-override)
  add_compile_options(-std=c++11 -w)
endif()

find_package(trajopt REQUIRED)
find_package(tesseract REQUIRED)
find_package(tesseract_motion_planners REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(tesseract_geometry REQUIRED)
find_package(tesseract_visualization REQUIRED)
find_package(PCL REQUIRED COMPONENTS core features filters io segmentation surface)
find_package(trajopt REQUIRED)

find_package(PythonInterp REQUIRED)
find_package(PythonLibs REQUIRED)

include(FindSWIG)
find_package(SWIG 4.0.0 REQUIRED)
include(UseSWIG)

# Primary tesseract_python library

set(SWIG_CXX_EXTENSION cxx)
set_property(SOURCE swig/tesseract_python.i PROPERTY CPLUSPLUS ON)
set_property(SOURCE swig/tesseract_python.i PROPERTY SWIG_FLAGS -relativeimport -threads)

set(CMAKE_SWIG_OUTDIR ${CMAKE_CURRENT_BINARY_DIR}/python/tesseract)
set(SWIG_OUTFILE_DIR ${CMAKE_CURRENT_BINARY_DIR})
swig_add_module(tesseract_python python swig/tesseract_python.i)
swig_link_libraries(tesseract_python tesseract::tesseract_motion_planners_trajopt tesseract::tesseract_motion_planners_core tesseract::tesseract_visualization tesseract::tesseract trajopt::trajopt ${PYTHON_LIBRARIES} ${EIGEN3_LIBRARIES} ${PYTHON_LIBRARIES})
target_include_directories(${SWIG_MODULE_tesseract_python_REAL_NAME} PUBLIC 
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/swig>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${SWIG_MODULE_tesseract_python_REAL_NAME}  SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${console_bridge_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${TinyXML2_INCLUDE_DIRS}
    $<TARGET_PROPERTY:tesseract::tesseract,INTERFACE_INCLUDE_DIRECTORIES>)

set_target_properties(${SWIG_MODULE_tesseract_python_REAL_NAME}
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/python/tesseract)

add_custom_command(TARGET ${SWIG_MODULE_tesseract_python_REAL_NAME} POST_BUILD COMMAND 
${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/swig/__init__.py
$<SHELL_PATH:$<TARGET_FILE_DIR:${SWIG_MODULE_tesseract_python_REAL_NAME}>>/__init__.py)

# Check for ROS support
find_package(tesseract_msgs)
find_package(tesseract_rosutils)
find_package(geometry_msgs)
find_package(std_msgs)
find_package(roscpp)
find_package(roslib)

if( tesseract_msgs_FOUND AND tesseract_rosutils_FOUND AND geometry_msgs_FOUND AND std_msgs_FOUND AND roscpp_FOUND AND roslib_FOUND)
message(STATUS "Found Tesseract ROS support, building tesseract_ros_python")
set (BUILD_TESSERACT_ROS_PYTHON ON CACHE BOOL "Build tesseract_ros_python module")
else()
set (BUILD_TESSERACT_ROS_PYTHON OFF CACHE BOOL "Build tesseract_ros_python module")
endif()

if(BUILD_TESSERACT_ROS_PYTHON)
  set(SWIG_CXX_EXTENSION cxx)
  set_property(SOURCE swig/ros/tesseract_ros_python.i PROPERTY CPLUSPLUS ON)
  set_property(SOURCE swig/tesseract_ros_python.i PROPERTY SWIG_FLAGS -relativeimport -threads)

  set(CMAKE_SWIG_OUTDIR ${CMAKE_CURRENT_BINARY_DIR}/python/tesseract_ros)
  set(SWIG_OUTFILE_DIR ${CMAKE_CURRENT_BINARY_DIR})
  swig_add_module(tesseract_ros_python python swig/ros/tesseract_ros_python.i)
  swig_link_libraries(tesseract_ros_python 
    ${tesseract_rosutils_LIBRARIES}
    ${tesseract_msgs_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${roslib_LIBRARIES}
    tesseract::tesseract_motion_planners_trajopt 
    tesseract::tesseract_motion_planners_core 
    tesseract::tesseract_visualization 
    tesseract::tesseract trajopt::trajopt 
    ${PYTHON_LIBRARIES} 
    ${EIGEN3_LIBRARIES} ${PYTHON_LIBRARIES})
  target_include_directories(${SWIG_MODULE_tesseract_ros_python_REAL_NAME} PUBLIC 
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/swig>"
      "$<INSTALL_INTERFACE:include>")
  target_include_directories(${SWIG_MODULE_tesseract_ros_python_REAL_NAME}  SYSTEM PUBLIC
      ${tesseract_rosutils_INCLUDE_DIRS}
      ${EIGEN3_INCLUDE_DIRS}
      ${console_bridge_INCLUDE_DIRS}
      ${Boost_INCLUDE_DIRS}
      ${TinyXML2_INCLUDE_DIRS}
      $<TARGET_PROPERTY:tesseract::tesseract,INTERFACE_INCLUDE_DIRECTORIES>)

  set_target_properties(${SWIG_MODULE_tesseract_ros_python_REAL_NAME}
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/python/tesseract_ros)

  add_custom_command(TARGET ${SWIG_MODULE_tesseract_ros_python_REAL_NAME} POST_BUILD COMMAND 
    ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/swig/ros/__init__.py
    $<SHELL_PATH:$<TARGET_FILE_DIR:${SWIG_MODULE_tesseract_ros_python_REAL_NAME}>>/__init__.py)

endif()

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/cmake/setup.py.in" "${CMAKE_CURRENT_BINARY_DIR}/python/setup.py" @ONLY)

set(enable_setuptools_deb_layout OFF)
if(EXISTS "/etc/debian_version")
  set(enable_setuptools_deb_layout ON)
endif()
option(SETUPTOOLS_DEB_LAYOUT "Enable debian style python package layout" ${enable_setuptools_deb_layout})
if(SETUPTOOLS_DEB_LAYOUT)
  message(STATUS "Using Debian Python package layout")
  set(SETUPTOOLS_ARG_EXTRA "--install-layout=deb")
endif()

install(CODE "message(STATUS \"Running setup.py in ${CMAKE_CURRENT_BINARY_DIR}/python\")
execute_process(COMMAND python setup.py install -f
--root=/ --prefix=${CMAKE_INSTALL_PREFIX} ${SETUPTOOLS_ARG_EXTRA} --single-version-externally-managed WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/python)")
