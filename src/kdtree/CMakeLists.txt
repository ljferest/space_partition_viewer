add_library(kdtree STATIC
    Kdtree.h
    Kdtree.cpp          # más adelante, aquí copiarás/pegarás la implementación
)

target_include_directories(kdtree PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)