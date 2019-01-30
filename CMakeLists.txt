# 
#  Copyright (C) 2017 IIT-ADVR
#  Author: 
#  email: 
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU Lesser General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
#  GNU Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public License
#  along with this program. If not, see <http://www.gnu.org/licenses/>
#

# CMake 3.5.1 required 
cmake_minimum_required(VERSION 3.5.1)

# XBot RT Plugin
project(test_collision_avoidance)

# C++11
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# set default build type
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE RelWithDebInfo)
ENDIF(NOT CMAKE_BUILD_TYPE)

# Add cmake dir and include custom macros
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
include(MacroYCMInstallLibrary)

# find required packages
find_package(XCM REQUIRED)

find_package(RBDL REQUIRED)

find_package(OpenSoT REQUIRED)

find_package(catkin REQUIRED COMPONENTS actionlib_msgs robot_state_publisher tf_conversions roscpp roslib std_msgs sensor_msgs std_srvs geometry_msgs message_generation eigen_conversions)

# XENOMAI
set(ENABLE_XENO CACHE BOOL "ON")
if ( ${ENABLE_XENO} )
   find_package(Xenomai)
endif()

catkin_package(
#  INCLUDE_DIRS include
   LIBRARIES 
   CATKIN_DEPENDS robot_state_publisher roscpp roslib actionlib_msgs
   DEPENDS
)

# include
include_directories(${XCM_INCLUDE_DIRS}
                    ${RBDL_INCLUDE_DIR}
                    ${OpenSoT_INCLUDE_DIRS} 
                    ${catkin_INCLUDE_DIRS}
                    )

add_executable(test_collision_avoidance
   test_collision_avoidance.cpp
)

target_link_libraries(test_collision_avoidance 
		      ${XCM_LIBRARIES} 
		      ${catkin_LIBRARIES}
		      yaml-cpp
		      ${RBDL_LIBRARY}
		      ${RBDL_URDFReader_LIBRARY}
		      ${OpenSoT_LIBRARIES})
		      
# install
install(TARGETS test_collision_avoidance
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
