#pragma once
#include "common/Point3D.h"
#include "common/BoundingBox.h"

struct KdNode {
    Point3D* point;
    // 0 = x, 1 = y, 2 = z.
    int axis; 
    KdNode* left;
    KdNode* right;

    BoundingBox bbox;

    KdNode(Point3D* pt, int ax,const BoundingBox& box)
        : point(pt), axis(ax),bbox(box), left(nullptr), right(nullptr) {}
};
