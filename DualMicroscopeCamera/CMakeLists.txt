cmake_minimum_required(VERSION 3.16)
project(DualMicroscopeCamera)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find required packages
find_package(OpenCV REQUIRED)

# Platform-specific settings
if(WIN32)
    # Windows specific settings
    add_executable(DualMicroscopeCamera
        main.cpp
        analyse.cpp
        analyse.h
        contrast.cpp
        contrast.h
        fitting.cpp
        fitting.h
        fourier.cpp
        fourier.h
        MachineLearningImpl.cpp
        MachineLearningImpl.h
    )
    
    # Windows specific libraries
    target_link_libraries(DualMicroscopeCamera PRIVATE 
        ${OpenCV_LIBRARIES}
        ole32
        oleaut32
        strmiids
    )
    
    # Windows specific includes
    target_include_directories(DualMicroscopeCamera PRIVATE ${OpenCV_INCLUDE_DIRS})
    
elseif(UNIX)
    # Linux specific settings - simplified version without DirectShow
    add_executable(DualMicroscopeCamera
        main_linux.cpp
        analyse.cpp
        analyse.h
        contrast.cpp
        contrast.h
        fitting.cpp
        fitting.h
        fourier.cpp
        fourier.h
        MachineLearningImpl.cpp
        MachineLearningImpl.h
    )
    
    # Linux specific libraries
    target_link_libraries(DualMicroscopeCamera PRIVATE ${OpenCV_LIBRARIES})
    
    # Linux specific includes
    target_include_directories(DualMicroscopeCamera PRIVATE ${OpenCV_INCLUDE_DIRS})
endif()

# Common compiler flags
if(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    add_compile_options(/wd4996)
else()
    add_compile_options(-Wall -Wextra)
endif()