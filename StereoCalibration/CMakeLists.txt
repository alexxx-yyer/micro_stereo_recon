cmake_minimum_required(VERSION 3.16)
project(StereoCalibration)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF) 

set(CUSTOMLIB_DIR "" CACHE PATH "")
set(OpenCV_DIR "" CACHE PATH "")

find_package(OpenCV REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Widgets Core5Compat)

add_executable(test
    test.cpp
    ${CUSTOMLIB_DIR}/DirTools.cpp)

target_include_directories(test PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${CUSTOMLIB_DIR})

target_link_libraries(test PRIVATE
    ${OpenCV_LIBRARIES})

qt_standard_project_setup()

qt_add_executable(StereoCalib
    main.cpp
    panel.cpp
    panel.h
    panel.ui
    ${CUSTOMLIB_DIR}/DirTools.cpp
    ${CUSTOMLIB_DIR}/StereoMatchingTools.cpp)

target_link_libraries(StereoCalib PRIVATE Qt6::Widgets Qt6::Core5Compat ${OpenCV_LIBRARIES})

target_include_directories(StereoCalib PRIVATE ${OpenCV_INCLUDE_DIRS} ${CUSTOMLIB_DIR})