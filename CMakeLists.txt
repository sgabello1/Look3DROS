cmake_minimum_required(VERSION 2.8.3)
<<<<<<< HEAD
project(ros_look3d)
=======
project(look3d_ros)
>>>>>>> 5a52b0cd679e0566eb478fffd6757ffd1daabf3e

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cmake_modules
  cv_bridge
  	
)

<<<<<<< HEAD
=======
catkin_package()

>>>>>>> 5a52b0cd679e0566eb478fffd6757ffd1daabf3e
set(Look3D_DIR /usr/local/look3d/cmake)
find_package(LooK3D)
#set(Look3D_LIBS /home/sgabello/look3d)
#set(Look3D_EXTERNAL_LIBS /home/sgabello/look3d/external)
<<<<<<< HEAD

#include_directories(${Look3D_LIBS} ${Look3D_EXTERNAL_LIBS})

#Slick library
#include_directories(
#  include
#/home/sgabello/look3d/external/slick-1.1/slick
#)


#find_package(Eigen REQUIRED eigen3)
#pkg_search_module(EIGEN REQUIRED eigen3)
#include_directories(${Look3D_INCLUDE_DIRS})
#link_directories(${Look3D_LIB_DIR})
#link_directories(${Boost_LIBRARY_DIR})

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})
include_directories(/usr/include/eigen3)
link_directories(${EIGEN_LIBRARY_DIRS})

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(ros_mt  src/ros_mt.cpp)
target_link_libraries(ros_mt  ${catkin_LIBRARIES} ${Look3D_LIBRARIES} ${Look3D_EXTERNAL_LIBS} )

#message("Look3D_EXTERNAL_LIBS:${Look3D_EXTERNAL_LIBS}")
#message("${Look3D_EXTERNAL_LIBS}") 

#add_executable(plane_tracker_live src/plane_tracker_live.cpp)
#target_link_libraries(plane_tracker_live ${catkin_LIBRARIES} ${Look3D_LIBRARIES} ${Look3D_EXTERNAL_LIBS} )

#add_executable(pano_tracker src/pano_tracker.cpp)
#target_link_libraries(pano_tracker ${catkin_LIBRARIES} ${Look3D_LIBRARIES} ${Look3D_EXTERNAL_LIBS})

#add_executable(look3d_sar src/look3d_sar.cpp)
#target_link_libraries(look3d_sar ${catkin_LIBRARIES} ${Look3D_LIBRARIES} ${Look3D_EXTERNAL_LIBS} )

#add_executable(esm_ros src/esm_ros.cpp)
#target_link_libraries(esm_ros ${catkin_LIBRARIES} ${Look3D_LIBRARIES} ${Look3D_EXTERNAL_LIBS} )
=======
#include_directories(${Look3D_LIBS} ${Look3D_EXTERNAL_LIBS})

#find_package(Eigen REQUIRED eigen3)
#pkg_search_module(EIGEN REQUIRED eigen3)
include_directories(${Look3D_INCLUDE_DIRS})
link_directories(${Look3D_LIB_DIR})
link_directories(${Boost_LIBRARY_DIR})

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})
include_directories(/usr/include/eigen3)
#link_directories(${EIGEN_LIBRARY_DIRS})

include_directories(include ${catkin_INCLUDE_DIRS})

message("Look3D_EXTERNAL_LIBS:${Look3D_EXTERNAL_LIBS}")
message("${Look3D_EXTERNAL_LIBS}") 
add_executable(plane_tracker_live src/plane_tracker_live.cpp)
target_link_libraries(plane_tracker_live ${catkin_LIBRARIES} ${Look3D_LIBRARIES} ${Look3D_EXTERNAL_LIBS} )

add_executable(pano_tracker src/pano_tracker.cpp)
target_link_libraries(pano_tracker ${catkin_LIBRARIES} ${Look3D_LIBRARIES} ${Look3D_EXTERNAL_LIBS})

add_executable(marker_tracker_ros src/marker_tracker_ros.cpp)
target_link_libraries(marker_tracker_ros ${catkin_LIBRARIES} ${Look3D_LIBRARIES} ${Look3D_EXTERNAL_LIBS} )

add_executable(look3d_sar src/look3d_sar.cpp)
target_link_libraries(look3d_sar ${catkin_LIBRARIES} ${Look3D_LIBRARIES} ${Look3D_EXTERNAL_LIBS} )
>>>>>>> 5a52b0cd679e0566eb478fffd6757ffd1daabf3e

