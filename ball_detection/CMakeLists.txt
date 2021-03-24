
#find_package( OpenCV REQUIRED)
#find_package(raspicam)
#include_directories( ${OpenCV_INCLUDE_DIRS} )

#######################################
cmake_minimum_required (VERSION 2.8) 
project( ball_detection )
set(raspicam_DIR "/usr/local/lib/cmake")
find_package(raspicam REQUIRED)
find_package(OpenCV)
IF  ( OpenCV_FOUND AND raspicam_CV_FOUND)
MESSAGE(STATUS "COMPILING OPENCV TESTS")
add_executable( ball_detection ball_detection.cpp )  
target_link_libraries (ball_detection ${raspicam_CV_LIBS})
ELSE()
MESSAGE(FATAL_ERROR "OPENCV NOT FOUND IN YOUR SYSTEM")
ENDIF()
#####################################