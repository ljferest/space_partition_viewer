# octree es solo headers: no hay .cpp
add_library(octree INTERFACE)
target_include_directories(octree INTERFACE
  ${CMAKE_SOURCE_DIR}/src/octree
)
