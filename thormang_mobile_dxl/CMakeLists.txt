cmake_minimum_required(VERSION 2.8.3)
project(thormang_mobile_dxl)

find_package(catkin REQUIRED COMPONENTS
  dynamixel_sdk
  geometry_msgs
  roscpp
  std_msgs
)catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES thormang_mobile_dxl
#  CATKIN_DEPENDS dynamixel_sdk geometry_msgs roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

file(GLOB_RECURSE HeaderFiles "include/*.h")
add_custom_target(thormang_mobile_dxl_headers SOURCES ${HeaderFiles})

add_executable(thormang_mobile_dxl_node src/thormang_mobile_dxl_node.cpp)

target_link_libraries(thormang_mobile_dxl_node
  ${catkin_LIBRARIES}
)

#SET(CMAKE_BUILD_TYPE Debug)
SET(CMAKE_BUILD_TYPE Release)
