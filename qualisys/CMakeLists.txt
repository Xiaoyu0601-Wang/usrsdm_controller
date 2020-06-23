cmake_minimum_required(VERSION 2.8.3)
project(qualisys)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  geometry_msgs
  tf
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp)
# Support new yaml-cpp API (yaml-cpp>=0.5)
if(${YAMLCPP_FOUND})
  if(NOT(${YAMLCPP_VERSION} VERSION_LESS 0.5))
    add_definitions(-DHAVE_NEW_YAMLCPP)
  endif()
endif()

add_message_files(DIRECTORY msg)
generate_messages(DEPENDENCIES geometry_msgs)

catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    message_runtime geometry_msgs tf
  DEPENDS
  Boost Eigen3 YAMLCPP
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${YAMLCPP_INCLUDE_DIRS}
)

add_definitions("-std=c++0x -Wall")

message("EIGEN3_INCLUDE_DIR: " ${EIGEN3_INCLUDE_DIR})

add_library(qualisys_driver
  src/QualisysDriver.cpp
  include/qualisys/RTProtocol.cpp
  include/qualisys/NBC_Markup.cpp
  include/qualisys/RTPacket.cpp
  include/qualisys/Network.cpp
)
target_link_libraries(qualisys_driver
  ${catkin_LIBRARIES}
)
add_dependencies(qualisys_driver
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

#add_library(qualisys_calib
#  src/QualisysCalib.cpp
#)
#target_link_libraries(qualisys_calib
#  ${catkin_LIBRARIES}
#  ${YAMLCPP_LIBRARIES}
#)
#add_dependencies(qualisys_calib
#  ${${PROJECT_NAME}_EXPORTED_TARGETS}
#  ${catkin_EXPORTED_TARGETS}
#)

add_library(qualisys_odom
  src/KalmanFilter.cpp
  src/QualisysOdom.cpp
)
target_link_libraries(qualisys_odom
  ${catkin_LIBRARIES}
)
add_dependencies(qualisys_odom
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_executable(qualisys_node
  src/qualisys.cpp
)
target_link_libraries(qualisys_node
  qualisys_driver
  ${catkin_LIBRARIES}
)

#add_executable(qualisys_calib_node
#  src/qualisys_calib.cpp
#)
#target_link_libraries(qualisys_calib_node
#  qualisys_calib
#  ${catkin_LIBRARIES}
#)

add_executable(qualisys_odom_node
  src/qualisys_odom.cpp
)
target_link_libraries(qualisys_odom_node
  qualisys_odom
  ${catkin_LIBRARIES}
)
