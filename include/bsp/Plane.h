#pragma once

#include "common/Point3D.h"

struct Plane {
    Point3D normal; // Vector normal al plano
    float d;        // Distancia al origen

    Plane() : normal({0, 0, 1}), d(0) {}
    Plane(const Point3D& n, float offset) : normal(n), d(offset) {}

    // Devuelve >0 si el punto está delante, <0 detrás, 0 sobre el plano
    float distanceToPoint(const Point3D& p) const {
        return normal.x * p.x + normal.y * p.y + normal.z * p.z - d;
    }
};
