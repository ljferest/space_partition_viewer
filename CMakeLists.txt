cmake_minimum_required(VERSION 3.10)
project(space_partition VERSION 0.1 LANGUAGES C CXX)

# Estándar C++
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Busca las librerías necesarias
find_package(PCL REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(glfw3 REQUIRED)

# Rutas de include generales
include_directories(
  ${PCL_INCLUDE_DIRS}
  ${OPENGL_INCLUDE_DIR}
  ${GLUT_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/src
  ${CMAKE_SOURCE_DIR}/include
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Activar Octree optimizado (flag de compilación)
option(USE_OPTIMIZED_OCTREE "Use the optimized Octree version" ON)

if(USE_OPTIMIZED_OCTREE)
    add_definitions(-DUSE_OPTIMIZED_OCTREE)
    add_subdirectory(src/octree_optimized)
else()
    add_subdirectory(src/octree)
endif()

# Otros módulos (no dependen del flag)
add_subdirectory(src/kdtree)
add_subdirectory(src/bsp)
add_subdirectory(src/viewer)
