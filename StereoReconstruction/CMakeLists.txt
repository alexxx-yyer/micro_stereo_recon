cmake_minimum_required(VERSION 3.16)
project(StereoReconstruction)

set(OpenCV_DIR "" CACHE PATH "")
set(PCL_DIR "" CACHE PATH "")
set(CUSTOMLIB_DIR "" CACHE PATH "")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

add_executable(StereoReconstruct
    main.cpp
    ${CUSTOMLIB_DIR}/OCV_PCL.cpp
    ${CUSTOMLIB_DIR}/StereoMatchingTools.cpp)

target_include_directories(StereoReconstruct PRIVATE ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${CUSTOMLIB_DIR})
target_link_libraries(StereoReconstruct PRIVATE ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})