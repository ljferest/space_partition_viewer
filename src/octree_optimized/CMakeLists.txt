add_library(octree_optimized
    Octree.cpp
    CNode.cpp
)

target_include_directories(octree_optimized PUBLIC
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/include/octree_optimized
)
