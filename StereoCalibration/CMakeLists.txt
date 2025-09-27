cmake_minimum_required(VERSION 3.16)
project(StereoCalibration)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF) 

set(CUSTOMLIB_DIR "" CACHE PATH "")
set(OpenCV_DIR "" CACHE PATH "")

find_package(OpenCV REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core)

# Set Qt5 tools explicitly
set(QT5_UIC_EXECUTABLE /usr/bin/uic)
set(QT5_MOC_EXECUTABLE /usr/bin/moc)

add_executable(test
    test.cpp
    ${CUSTOMLIB_DIR}/DirTools.cpp)

target_include_directories(test PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${CUSTOMLIB_DIR})

target_link_libraries(test PRIVATE
    ${OpenCV_LIBRARIES})

add_executable(StereoCalib
    main.cpp
    panel.cpp
    ui_panel.h
    moc_panel.cpp
    ${CUSTOMLIB_DIR}/DirTools.cpp
    ${CUSTOMLIB_DIR}/StereoMatchingTools.cpp)

target_link_libraries(StereoCalib PRIVATE Qt5::Widgets Qt5::Core ${OpenCV_LIBRARIES})

target_include_directories(StereoCalib PRIVATE ${OpenCV_INCLUDE_DIRS} ${CUSTOMLIB_DIR})