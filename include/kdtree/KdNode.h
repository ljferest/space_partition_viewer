#pragma once
#include "common/Point3D.h"

struct KdNode {
    Point3D* point;
    int axis; // 0 = x, 1 = y, 2 = z
    KdNode* left;
    KdNode* right;

    KdNode(Point3D* pt, int ax)
        : point(pt), axis(ax), left(nullptr), right(nullptr) {}
};
