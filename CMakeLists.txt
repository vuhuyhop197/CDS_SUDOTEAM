cmake_minimum_required(VERSION 2.8.3)
project(lane_detect)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
)

set(OpenCV_STATIC OFF)
find_package(  OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES api_lane_detection 
  CATKIN_DEPENDS roscpp
  DEPENDS system_lib
)

include_directories(
  darknet_wrapper/darknet/include
  darknet_wrapper/darknet_wrapper
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(api_lane_detection
  src/detectlane.cpp
  src/carcontrol.cpp
)
link_directories(
	/usr/local/cuda/lib
  /usr/local/cuda/lib64
  src
	darknet_wrapper/darknet
  )
add_executable(
  lane_detect src/main.cpp
  # my_control_car src/my_control_car.cpp
	darknet_wrapper/darknet_wrapper/wrapper_cpp.cpp
  darknet_wrapper/darknet_wrapper/wrapper_cpp.h
  darknet_wrapper/darknet/src/wrapper_c.h
)

add_dependencies(lane_detect ${catkin_EXPORTED_TARGETS})
# add_dependencies(my_control_car lane_detect)


target_link_libraries(
  lane_detect
  api_lane_detection
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  darknet
)

install(TARGETS api_lane_detection ${ADDITIONAL_EXECUTABLES}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

add_executable(my_control_car src/my_control_car.cpp)
target_link_libraries(
  my_control_car
  api_lane_detection
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
)
