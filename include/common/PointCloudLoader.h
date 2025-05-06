#pragma once
#include <string>
#include <vector>
#include "common/Point3D.h"


// Loads a point cloud from a PCD file and returns a std::vector<Point3D>.
std::vector<Point3D> loadFromPCD(const std::string& filename);
