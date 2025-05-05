#pragma once

#include "Plane.h"
#include <vector>

struct BSPNode {
    Plane dividingPlane;
    std::vector<Point3D*> points;
    BSPNode* front;
    BSPNode* back;
    int nodeId;

    BSPNode(int id) : front(nullptr), back(nullptr), nodeId(id) {}
    ~BSPNode() {
        delete front;
        delete back;
    }
};
