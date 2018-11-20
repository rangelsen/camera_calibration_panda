list(APPEND subdirs input graphics common visualization imgui)

foreach(subdir ${subdirs})
	add_subdirectory(${subdir})
endforeach()

set(verification_sources 
	main.cpp
	camerasensor.cpp
)

add_executable(verification ${verification_sources})

foreach(subdir ${subdirs})
    target_link_libraries(verification ${subdir})
endforeach()

find_package(OpenCV REQUIRED core imgproc highgui aruco)

target_link_libraries(verification
	${subdirs}
    GLEW
    GL
    SDL2
	opencv_core
	opencv_highgui
	opencv_imgcodecs
	opencv_imgproc
	realsense2
	${OpenCv_LIBRARIES}
)
