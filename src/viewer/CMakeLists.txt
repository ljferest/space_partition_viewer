add_executable(viewer
    main.cpp
)

target_link_libraries(viewer
    octree
    # kdTree    # cuando lo implementes
    # bspTree   # cuando lo implementes
    ${PCL_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${GLUT_LIBRARY}
    glfw
)

target_include_directories(viewer PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)
