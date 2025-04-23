add_executable(viewer
    main.cpp
    OctreeRenderer.cpp       # <— nuevo
)

target_link_libraries(viewer
    octree
    kdtree
    bsp
    ${PCL_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${GLUT_LIBRARIES}
    glfw
)

target_include_directories(viewer PUBLIC
    ${CMAKE_SOURCE_DIR}/src       # para que "octree/Octree.h" resuelva a src/octree/Octree.h
    ${CMAKE_SOURCE_DIR}/src/viewer # para encontrar OctreeRenderer.h y main.cpp
)
