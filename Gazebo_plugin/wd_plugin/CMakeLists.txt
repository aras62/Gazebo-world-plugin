cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_msg REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  roscpp
  custom_msg
 )
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS} ${custom_msg_INCLUDE_DIRS})

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS} ${roscpp_CXX_FLAGS}")
set (CMAKE_CXX_STANDARD 11)
# Build our plugin
add_library(wd_plugin SHARED wd_plugin.cc)
target_link_libraries(wd_plugin ${GAZEBO_libraries} ${roscpp_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})



