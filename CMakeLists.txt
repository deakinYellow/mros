cmake_minimum_required(VERSION 3.0.2)
project(mros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

set( Eigen3_DIR "/usr/include/eigen3" )

include_directories(
  #include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_DIR}
)
catkin_package()


LIST( APPEND COMMON_LINK_LIBRARIES
  ${catkin_LIBRARIES}
)

#----------sample-------------------------------------
set( SAMPLE "sample" )
add_executable( ${SAMPLE}
  "sample_node.cc"
)
target_link_libraries( ${SAMPLE} ${COMMON_LINK_LIBRARIES} )
