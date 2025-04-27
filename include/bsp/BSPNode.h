#pragma once

#include "Plane.h"
#include <vector>

struct BSPNode {
    Plane dividingPlane;
    std::vector<Point3D*> points;
    BSPNode* front;
    BSPNode* back;

    BSPNode() : front(nullptr), back(nullptr) {}
    ~BSPNode() {
        delete front;
        delete back;
    }
};
