# src/bsp/CMakeLists.txt
add_library(bsp
    BSPTree.cpp
)

target_include_directories(bsp PUBLIC
    ${PROJECT_SOURCE_DIR}/include
)
