cmake_minimum_required(VERSION 2.8.3)
project(look3d_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cmake_modules
  cv_bridge
  	
)

catkin_package()

set(Look3D_DIR /usr/local/look3d/cmake)
find_package(LooK3D)
#set(Look3D_LIBS /home/sgabello/look3d)
#set(Look3D_EXTERNAL_LIBS /home/sgabello/look3d/external)
#include_directories(${Look3D_LIBS} ${Look3D_EXTERNAL_LIBS})

#find_package(Eigen REQUIRED eigen3)
#pkg_search_module(EIGEN REQUIRED eigen3)
include_directories(${Look3D_INCLUDE_DIRS})
link_directories(${Look3D_LIB_DIR})

include_directories(/usr/local/include/eigen3)
#link_directories(${EIGEN_LIBRARY_DIRS})

include_directories(include ${catkin_INCLUDE_DIRS})

message("${Look3D_EXTERNAL_LIBS}")

add_executable(plane_tracker_ros src/plane_tracker_ros.cpp)
target_link_libraries(plane_tracker_ros ${catkin_LIBRARIES} ${Look3D_LIBRARIES} ${Look3D_EXTERNAL_LIBS} )


