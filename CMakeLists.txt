cmake_minimum_required(VERSION 2.8.3)
project(quadruped_simshow)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  nav_msgs
  tf
  std_msgs
  sensor_msgs
  pcl_ros
)

catkin_package(
  INCLUDE_DIRS
#  LIBRARIES
#  ${PROJECT_NAME}_library
#  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  CATKIN_DEPENDS
  roscpp
)


include_directories(
  include ${catkin_INCLUDE_DIRS}
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)




add_executable(quadruped_util
  src/quadruped_util.cpp
)
add_dependencies(quadruped_util ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(quadruped_util
  ${catkin_LIBRARIES}
)

add_executable(loadmap
  src/loadmap.cpp
)
add_dependencies(loadmap ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(loadmap
  ${catkin_LIBRARIES}
)


add_executable(load_raw_map
  src/load_raw_map.cpp
)
add_dependencies(load_raw_map ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(load_raw_map
  ${catkin_LIBRARIES}
)

add_executable(load_legoloam_map
  src/load_legoloam_map.cpp
)
add_dependencies(load_legoloam_map ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(load_legoloam_map
  ${catkin_LIBRARIES}
)

add_executable(write_map
  src/write_map.cpp
)
add_dependencies(write_map ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(write_map
  ${catkin_LIBRARIES}
)