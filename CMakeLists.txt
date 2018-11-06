cmake_minimum_required(VERSION 2.8.3)
project(camera_calibration_panda)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

add_compile_options(-std=c++11 -g)

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
find_package(Franka REQUIRED)

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

find_library(REALSENSE2_LIBRARY realsense2)
include_directories(SYSTEM ${THIS_PACKAGE_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(util STATIC src/util.cpp)
add_library(calibration STATIC src/calibration.cpp)
add_library(camera STATIC src/camerasensor.cpp)

add_executable(calib src/main.cpp)
add_executable(robot src/robot_commander.cpp)
add_executable(manual_pose_pub src/manual_pose_pub.cpp)
add_executable(calib_manual src/calib_manual.cpp)
add_executable(verification src/verification.cpp)
# add_executable(verification_pcloud src/verification_pcloud.cpp)
add_executable(collect-pose src/pose-collection.cpp)
add_executable(capture src/capture.cpp)

target_link_libraries(robot ${catkin_LIBRARIES})
target_link_libraries(manual_pose_pub ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} util)
target_link_libraries(camera ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${REALSENSE2_LIBRARY})
target_link_libraries(calibration ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} camera util)
target_link_libraries(calib ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} util camera calibration)
target_link_libraries(calib_manual ${catkin_LIBRARIES} ${Franka_LIBRARIES} ${OpenCV_LIBRARIES} util camera calibration)
target_link_libraries(verification ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} util camera calibration)
target_link_libraries(collect-pose ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Franka_LIBRARIES} util)
target_link_libraries(capture ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} util camera)
