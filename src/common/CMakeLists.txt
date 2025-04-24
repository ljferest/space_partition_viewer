add_library(pointcloud_loader
    PointCloudLoader.cpp
)

target_include_directories(pointcloud_loader PUBLIC
    ${PROJECT_SOURCE_DIR}/include
    ${PCL_INCLUDE_DIRS}
)

find_package(PCL REQUIRED)

target_link_libraries(pointcloud_loader
    ${PCL_LIBRARIES}
)
