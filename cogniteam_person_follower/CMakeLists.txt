cmake_minimum_required(VERSION 3.0.2)
project(cogniteam_person_follower)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


set(OpenCV_DIR /usr/local/share/OpenCV/)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  image_transport
  vision_msgs
  cv_bridge
  tf
  tf2_geometry_msgs
  tf2_ros
  depth_image_proc
  geometry_msgs
  std_msgs  
  image_geometry      
  visualization_msgs  

  leg_tracker

  pcl_ros
  

)
find_package(realsense2 REQUIRED)
# find_package(OpenCV COMPONENTS tracking REQUIRED PATHS "/usr/local/include/opencv2" "/usr/local/lib")
find_package(OpenCV REQUIRED)

find_package(jetson-utils REQUIRED)
find_package(jetson-inference REQUIRED)
find_package(CUDA REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

# message(STATUS "OpenCV library status:")
# message(STATUS "    config: ${OpenCV_DIR}")
# message(STATUS "    version: ${OpenCV_VERSION}")
# message(STATUS "    libraries: ${OpenCV_LIBS}")
# message(STATUS "    libraries: ${OpenCV_LIBRARIES}")
# message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES cogniteam-person-follower
 CATKIN_DEPENDS roscpp rospy
 tf2_geometry_msgs
 tf2_ros
 DEPENDS OpenCV
#  DEPENDS system_lib
)


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  
  if(COMPLITE_WITH_JETSON_UTILS)
    ${CUDA_INCLUDE_DIRS}
  endif(COMPLITE_WITH_JETSON_UTILS)  
  
)

message(STATUS "${OpenCV_INCLUDE_DIRS},
${catkin_INCLUDE_DIRS}, 
${CUDA_INCLUDE_DIRS}")


add_executable(cogniteam_person_follower_node
      src/image_converter.cpp
      src/detectnet.cpp
      src/cogniteam_person_follower_node.cpp
)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(cogniteam_person_follower_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(cogniteam_person_follower_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${realsense2_LIBRARY}
   jetson-inference  
)