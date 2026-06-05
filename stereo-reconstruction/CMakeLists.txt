cmake_minimum_required(VERSION 3.16)
project(StereoReconstruction VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Windows: common OpenCV install locations
if(WIN32 AND NOT DEFINED OpenCV_DIR)
  foreach(_candidate
      "C:/opencv/build"
      "C:/opencv/build/x64/vc16/lib"
      "C:/tools/opencv/build"
      "$ENV{USERPROFILE}/opencv/build")
    if(EXISTS "${_candidate}/OpenCVConfig.cmake" OR
       EXISTS "${_candidate}/opencv-config.cmake")
      set(OpenCV_DIR "${_candidate}")
      message(STATUS "Auto-detected OpenCV at ${OpenCV_DIR}")
      break()
    endif()
  endforeach()
endif()

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
message(STATUS "OpenCV version: ${OpenCV_VERSION}")

find_package(Open3D QUIET)

include_directories(include)

# Individual executables
add_executable(rectification src/rectification.cpp src/utils.cpp)
target_link_libraries(rectification ${OpenCV_LIBS})

add_executable(matching src/matching.cpp src/utils.cpp)
target_link_libraries(matching ${OpenCV_LIBS})

add_executable(depth src/depth.cpp src/utils.cpp)
target_link_libraries(depth ${OpenCV_LIBS})

# Pipeline: compile shared sources with PIPELINE_BUILD so their main() is excluded
add_library(stereo_lib OBJECT
    src/rectification.cpp
    src/matching.cpp
    src/depth.cpp
    src/utils.cpp
)
target_compile_definitions(stereo_lib PRIVATE PIPELINE_BUILD)
target_include_directories(stereo_lib PRIVATE include ${OpenCV_INCLUDE_DIRS})

add_executable(pipeline src/pipeline.cpp)
target_link_libraries(pipeline stereo_lib ${OpenCV_LIBS})
target_compile_definitions(pipeline PRIVATE PIPELINE_BUILD)

if(Open3D_FOUND)
    message(STATUS "Open3D found — mesh reconstruction enabled")
    add_executable(mesh src/mesh.cpp src/utils.cpp)
    target_link_libraries(mesh ${OpenCV_LIBS} Open3D::Open3D)
else()
    message(STATUS "Open3D not found — mesh step will be skipped")
endif()
