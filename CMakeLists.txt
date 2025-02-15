cmake_minimum_required(VERSION 2.8.3)
project(camera_calibration_panda)

add_compile_options(-std=c++11 -g)

# find_package(Boost REQUIRED system filesystem date_time thread)
find_package(OpenCV REQUIRED core imgproc highgui aruco)
find_package(Franka REQUIRED)
find_package(PCL REQUIRED common QUIET)

include_directories(SYSTEM ${THIS_PACKAGE_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ..)
include_directories(../rswrapper/src)

link_directories(../common/build)
link_directories(../rswrapper/build)

# add_library(util STATIC src/util.cpp)
add_library(calibration STATIC src/calibration.cpp)
add_library(rswrapper STATIC ../rswrapper/src/camerasensor.cpp)

# add_executable(calib src/main.cpp)
# add_executable(robot src/robot_commander.cpp)
# add_executable(manual_pose_pub src/manual_pose_pub.cpp)
add_executable(calib_manual src/calib_manual.cpp)
# add_executable(verification src/verification.cpp)
# add_executable(verification_pcloud src/verification_pcloud.cpp)
# add_executable(collect-pose src/pose-collection.cpp)
# add_executable(capture src/capture.cpp)

# target_link_libraries(robot ${catkin_LIBRARIES})
# target_link_libraries(manual_pose_pub ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} util)
# target_link_libraries(camera ${OpenCV_LIBRARIES} -lrealsense2)
# target_link_libraries(calibration ${OpenCV_LIBRARIES} camera common)
# target_link_libraries(calib ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} util camera calibration)
target_link_libraries(calib_manual
    ${OpenCV_LIBRARIES}
    ${Franka_LIBRARIES}
    ${PCL_LIBRARIES}
    common
    rswrapper
    calibration
    -lrealsense2
)

#[[
target_link_libraries(verification ${OpenCV_LIBRARIES} common rswrapper calibration)
target_link_libraries(collect-pose ${OpenCV_LIBRARIES} ${Franka_LIBRARIES} common)
target_link_libraries(capture ${OpenCV_LIBRARIES} common rswrapper)
#]]
