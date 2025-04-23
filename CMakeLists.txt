cmake_minimum_required(VERSION 3.10)
project(space_partition VERSION 0.1 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(PCL REQUIRED)
find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS}
                    ${OPENGL_INCLUDE_DIR}
                    ${GLUT_INCLUDE_DIRS}
                    ${CMAKE_SOURCE_DIR}/include)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Módulos
add_subdirectory(src/octree)
add_subdirectory(src/kdtree)
add_subdirectory(src/bsp)
add_subdirectory(src/viewer)
