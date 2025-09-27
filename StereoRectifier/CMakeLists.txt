cmake_minimum_required(VERSION 3.16)
project(stereoRectifier)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CUSTOMLIB_DIR "" CACHE PATH "")
set(OpenCV_DIR "" CACHE PATH "")
set(PCL_DIR "" CACHE PATH "")

if(NOT CUSTOMLIB_DIR)
    message(FATAL_ERROR "Missing CUSTOMLIB_DIR value.")
endif()

if(NOT OpenCV_DIR)
    message(FATAL_ERROR "Missing OpenCV_DIR value.")
endif()

if(NOT PCL_DIR)
    message(FATAL_ERROR "Missing PCL_DIR value.")
endif()

find_package(OpenCV REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core)
find_package(PCL REQUIRED)

add_executable(stereoRectifier main.cpp
    ${CUSTOMLIB_DIR}/StereoMatchingTools.cpp
    ${CUSTOMLIB_DIR}/DirTools.cpp
    ${CUSTOMLIB_DIR}/VisualizationTools.cpp
    window.cpp
    moc_window.cpp
    ui_window.h)

target_link_libraries(stereoRectifier PRIVATE ${OpenCV_LIBRARIES} Qt5::Widgets Qt5::Core ${PCL_LIBRARIES})

target_include_directories(stereoRectifier PRIVATE ${OpenCV_INCLUDE_DIRS} ${CUSTOMLIB_DIR} ${PCL_INCLUDE_DIRS})