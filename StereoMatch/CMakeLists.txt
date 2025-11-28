cmake_minimum_required(VERSION 3.16)
project(StereoMatch)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(OpenCV_DIR "" CACHE PATH "The directory including OpenCVConfig.cmake file. Commonly in opencv-<version>/build/x64/vc16/lib.")
set(CUSTOMLIB_DIR "" CACHE PATH "The directory including custom header and cpp files implement some convinent functions.")
set(PCL_DIR "" CACHE PATH "The directory including pclConfig.cmake file. Commonly in PCL-<version>/cmake.")

if(NOT OpenCV_DIR)
    message(FATAL_ERROR "Variable:OpenCV_DIR is empty, it should be explictly set by user through -DOpenCV_DIR=<directory> while running CMake.")
else()
    message(STATUS OpenCV_DIR:${OpenCV_DIR})
endif()

if(NOT CUSTOMLIB_DIR)
    message(FATAL_ERROR "Variable:CUSTOMLIB_DIR is empty, it should be explictly set by user through -DCUSTOMLIB_DIR=<directory> while running CMake.")
else()
    message(STATUS CUSTOMLIB_DIR:${CUSTOMLIB_DIR})
endif()

if(NOT PCL_DIR)
    message(FATAL_ERROR "Variable:PCL_DIR is empty, it should be explictly set by user through -DCUSTOMLIB_DIR=<directory> while running CMake.")
else()
    message(STATUS PCL_DIR:${PCL_DIR})
endif()

set(SRC_DIR ${CMAKE_CURRENT_LIST_DIR}/src)

if(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    add_compile_options(/wd4996)
endif()

add_subdirectory(3rdparty/elas)
add_subdirectory(src/console)
add_subdirectory(src/GUI)