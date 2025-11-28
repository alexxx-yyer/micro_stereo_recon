cmake_minimum_required(VERSION 3.16)
project(StereoMatchConsole)

add_executable(StereoMatchConsole)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

message(STATUS OpenCV_INCLUDE_DIRS:${OpenCV_INCLUDE_DIRS})
message(STATUS OpenCV_LIBRARIES:${OpenCV_LIBRARIES})

target_sources(StereoMatchConsole PRIVATE 
    main.cpp
    ${SRC_DIR}/fileIO.cpp
    interactive.cpp
    ${SRC_DIR}/algorithms.cpp
    interactive.h
    ${SRC_DIR}/include/fileIO.h
    ${SRC_DIR}/include/algorithms.h
    ${CUSTOMLIB_DIR}/VisualizationTools.cpp
    ${CUSTOMLIB_DIR}/DirTools.cpp
    ${CUSTOMLIB_DIR}/StereoMatchingTools.cpp
    ${CUSTOMLIB_DIR}/OCV_PCL.cpp)

target_include_directories(StereoMatchConsole PRIVATE
    ${SRC_DIR}/include
    ${CUSTOMLIB_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${ELAS_DIR})

target_link_libraries(StereoMatchConsole PRIVATE ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} elaslib)