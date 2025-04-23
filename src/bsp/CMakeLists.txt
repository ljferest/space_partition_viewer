add_library(bsp STATIC
    Bsp.h
    Bsp.cpp          # más adelante, aquí copiarás/pegarás la implementación
)

target_include_directories(bsp PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)
