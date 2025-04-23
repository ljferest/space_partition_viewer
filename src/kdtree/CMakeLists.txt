add_library(kdtree STATIC
  KdTree.h
  KdTree.cpp
)

target_include_directories(kdtree PUBLIC
  ${CMAKE_SOURCE_DIR}/src/kdtree
)
