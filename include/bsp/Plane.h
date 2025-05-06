#pragma once

#include "common/Point3D.h"

struct Plane {
    //Normal from the plane.
    Point3D normal; 
    //Distance from the origin.
    float d;        

    Plane() : normal({0, 0, 1}), d(0) {}
    Plane(const Point3D& n, float offset) : normal(n), d(offset) {}


    float distanceToPoint(const Point3D& p) const {
        return normal.x * p.x + normal.y * p.y + normal.z * p.z - d;
    }
};
