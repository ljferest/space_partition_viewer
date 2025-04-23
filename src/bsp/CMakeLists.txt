add_library(bsp STATIC
  BSPTree.h
  BSPTree.cpp
)

target_include_directories(bsp PUBLIC
  ${CMAKE_SOURCE_DIR}/src/bsp
)