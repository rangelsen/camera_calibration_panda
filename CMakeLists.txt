cmake_minimum_required(VERSION 2.8.3)
project(camera_calibration_panda)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    interactive_markers
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    geometric_shapes
    rosbag
    geometry_msgs
)

find_package(Boost REQUIRED system filesystem date_time thread)
find_package(OpenCV REQUIRED core imgproc highgui aruco)

catkin_package(
  LIBRARIES
    interactivity_utils
  INCLUDE_DIRS
    ${THIS_PACKAGE_INCLUDE_DIRS}
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
  DEPENDS
    EIGEN3
)

find_library(REALSENSE2_LIBRARY realsense2 HINTS /usr/lib/x86_64-linux-gnu)
include_directories(SYSTEM ${THIS_PACKAGE_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})

foreach(lib ${REALSENSE2_LIBRARY})
    message(STATUS "lib=${lib}")
endforeach()

add_library(util STATIC src/util.cpp)
# add_library(calibration STATIC src/calibration.cpp)
# add_library(camera STATIC src/camerasensor.cpp)

#Add all executables
add_executable(main src/main.cpp src/camerasensor.cpp src/calibration.cpp src/util.cpp)
add_executable(robot src/robot_commander.cpp)
add_executable(manual_pose_pub src/manual_pose_pub.cpp)

target_link_libraries(robot ${catkin_LIBRARIES})
target_link_libraries(manual_pose_pub ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} util)
# target_link_libraries(camera ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${REALSENSE2_LIBRARY})
# target_link_libraries(calibration ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} camera util)
target_link_libraries(main ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${REALSENSE2_LIBRARY})
