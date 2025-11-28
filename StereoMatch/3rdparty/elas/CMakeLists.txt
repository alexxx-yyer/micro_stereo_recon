cmake_minimum_required(VERSION 3.16)
project(libelas)

if(MinGW)
set(CMAKE_CXX_FLAGS "-msse3")
elseif(MSVC)
set(CMAKE_CXX_FALGS "/arch:SSE2")
endif()

find_package(OpenCV REQUIRED)

file(GLOB LIBELAS_SRC_FILES "${CMAKE_CURRENT_LIST_DIR}/*.cpp")

set(CMAKE_BUILD_TYPE Release)
file(REAL_PATH ${CMAKE_CURRENT_LIST_DIR}/.. ELAS_DIR)
set(ELAS_DIR ${ELAS_DIR} PARENT_SCOPE)

add_library(elaslib STATIC ${LIBELAS_SRC_FILES})

target_include_directories(elaslib PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(elaslib PRIVATE ${OpenCV_LIBRARIES})