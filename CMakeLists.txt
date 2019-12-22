cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(InterPoseLib)

set(CMAKE_CXX_COMPILER g++)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++14 -O2")

set(Pangolin_DIR ./ThirdParty/Pangolin/build/src/)
find_package(Pangolin REQUIRED)

include_directories(
  ./src
  ./viewer
  ${Pangolin_INCLUDE_DIRS}
  ./ThirdParty/eigen
)

link_directories(
  ${Pangolin_LIBRARY_DIRS}
)

add_library(
  ${PROJECT_NAME} STATIC
  src/InterPose.cc
  src/Utils.hpp
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/build/bin)

# Build InterPoseViewer
add_executable(
  exampleInterPoseViewer
  viewer/exampleInterPoseViewer.cc
  viewer/Viewer.cc
)

target_link_libraries(
  exampleInterPoseViewer
  ${PROJECT_NAME}
  ${Pangolin_LIBRARIES}
)

# Build example InterPose
add_executable(
  exampleInterPose
  exampleInterPose.cc
)

target_link_libraries(
  exampleInterPose
  ${PROJECT_NAME}
)
