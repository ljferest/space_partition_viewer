add_library(octree STATIC
    Vec3.h
    OctreePoint.h
    Octree.h
    Stopwatch.h
    # Octree.cpp   # descomenta si copiaste este fichero
)

target_include_directories(octree PUBLIC
    ${CMAKE_SOURCE_DIR}/src/octree    # para que encuentre "octree/Vec3.h"
)
