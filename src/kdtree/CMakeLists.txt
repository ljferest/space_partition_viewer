add_library(kdtree
    KdTree.cpp
)

target_include_directories(kdtree PUBLIC
    ${PROJECT_SOURCE_DIR}/include
)

add_executable(test_kdtree test_kdtree.cpp)
target_link_libraries(test_kdtree
    kdtree
    pointcloud_loader
)
target_include_directories(test_kdtree PRIVATE ${PROJECT_SOURCE_DIR}/include)
