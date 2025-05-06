#pragma once
#include "Point3D.h"

struct BoundingBox {
    Point3D min;
    Point3D max;

    BoundingBox() {
        min = Point3D(0, 0, 0);
        max = Point3D(0, 0, 0);
    }

    BoundingBox(const Point3D& minPt, const Point3D& maxPt)
        : min(minPt), max(maxPt) {}

    // Utilidad: obtener el tamaño del box
    Point3D size() const {
        return Point3D(max.x - min.x, max.y - min.y, max.z - min.z);
    }

    // Utilidad: centro del bounding box
    Point3D center() const {
        return Point3D((min.x + max.x) / 2.0f,
                       (min.y + max.y) / 2.0f,
                       (min.z + max.z) / 2.0f);
    }
    void expandToInclude(const Point3D& p) {
        min.x = std::min(min.x, p.x);
        min.y = std::min(min.y, p.y);
        min.z = std::min(min.z, p.z);
        max.x = std::max(max.x, p.x);
        max.y = std::max(max.y, p.y);
        max.z = std::max(max.z, p.z);
    }
    
};

