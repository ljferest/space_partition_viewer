#pragma once
#include <string>
#include <vector>
#include "common/Point3D.h"

// Carga un archivo .pcd y lo convierte a std::vector<Point3D>
std::vector<Point3D> loadFromPCD(const std::string& filename);
