cmake_minimum_required(VERSION 3.16)
project(micro_stereo_recon VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set default paths for dependencies
set(OpenCV_DIR "" CACHE PATH "Path to OpenCV installation")
set(PCL_DIR "" CACHE PATH "Path to PCL installation")
set(CUSTOMLIB_DIR "/home/alex/code/micro_stereo_recon/CustomLib-CV" CACHE PATH "Path to custom libraries")

# Find required packages
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core)

# Set Qt5 tools explicitly
set(QT5_UIC_EXECUTABLE /usr/lib/x86_64-linux-gnu/qt5/bin/uic)
set(QT5_MOC_EXECUTABLE /usr/lib/x86_64-linux-gnu/qt5/bin/moc)

# Print configuration info
message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "OpenCV include dirs: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "PCL version: ${PCL_VERSION}")
message(STATUS "Qt5 version: ${Qt5_VERSION}")
message(STATUS "OpenCV_DIR:${OpenCV_DIR}")
message(STATUS "CUSTOMLIB_DIR:${CUSTOMLIB_DIR}")
message(STATUS "PCL_DIR:${PCL_DIR}")
message(STATUS "OpenCV_INCLUDE_DIRS:${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV_LIBRARIES:${OpenCV_LIBRARIES}")

# Set common include directories
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CUSTOMLIB_DIR})
include_directories(${PCL_INCLUDE_DIRS})

# Set common compiler flags
if(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    add_compile_options(/wd4996)
else()
    add_compile_options(-Wall -Wextra)
endif()

# Print build configuration
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Compiler: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")

# Add subdirectories for all projects
add_subdirectory(DualMicroscopeCamera)
add_subdirectory(StereoCalibration)
add_subdirectory(StereoMatch)
add_subdirectory(StereoReconstruction)
add_subdirectory(StereoRectifier)

# Create a custom target to build all executables
add_custom_target(build_all
    DEPENDS 
        DualMicroscopeCamera
        StereoCalib
        test
        StereoMatchConsole
        StereoMatchGUI
        StereoReconstruct
        stereoRectifier
    COMMENT "Building all executables"
)

# Print summary
message(STATUS "")
message(STATUS "=== Build Summary ===")
message(STATUS "Console Programs:")
message(STATUS "  - DualMicroscopeCamera: Dual camera capture")
message(STATUS "  - StereoReconstruct: 3D reconstruction")
message(STATUS "  - StereoMatchConsole: Console stereo matching")
message(STATUS "  - test: Calibration test")
message(STATUS "")
message(STATUS "GUI Programs:")
message(STATUS "  - StereoCalib: Stereo calibration GUI")
message(STATUS "  - StereoMatchGUI: Stereo matching GUI")
message(STATUS "  - stereoRectifier: Stereo rectification GUI")
message(STATUS "")
message(STATUS "Libraries:")
message(STATUS "  - elaslib: ELAS algorithm library")
message(STATUS "")
message(STATUS "To build all: make build_all")
message(STATUS "To build specific: make <target_name>")
message(STATUS "====================")