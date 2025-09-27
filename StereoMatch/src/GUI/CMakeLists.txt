cmake_minimum_required(VERSION 3.16)
project(StereoMatchGUI VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

message(STATUS "Project Dir: ${PROJECT_SOURCE_DIR}")

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Widgets Core5Compat)

message(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")

set(CMAKE_AUTOUIC OFF)

qt_standard_project_setup()

qt_add_executable(StereoMatchGUI
    main.cpp
    panel.cpp
    AlgWidget.h
    LoadWidget.h
    ${CUSTOMLIB_DIR}/DirTools.cpp
    ${CUSTOMLIB_DIR}/VisualizationTools.cpp
    ${CUSTOMLIB_DIR}/StereoMatchingTools.cpp
    ${CUSTOMLIB_DIR}/OCV_PCL.cpp
    ${SRC_DIR}/algorithms.cpp
    ${SRC_DIR}/fileIo.cpp)

target_link_libraries(StereoMatchGUI PRIVATE
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
    Qt6::Widgets
    Qt6::Core5Compat
    elaslib)

message(STATUS ELAS_DIR:${ELAS_DIR})

target_include_directories(StereoMatchGUI PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${BOOST_INCLUDE_DIRS}
    ${VTK_PREFIX_PATH}/include/vtk-9.1
    ${CUSTOMLIB_DIR}
    ${SRC_DIR}/include
    ${ELAS_DIR})

set_target_properties(StereoMatchGUI PROPERTIES
    WIN32_EXECUTABLE ON
    MACOSX_BUNDLE ON)