#pragma once
#include "common/Point3D.h"
#include "common/BoundingBox.h"

struct KdNode {
    Point3D* point;
    int axis; // 0 = x, 1 = y, 2 = z
    KdNode* left;
    KdNode* right;

    BoundingBox bbox;     // 📦 Bounding box del subespacio

    KdNode(Point3D* pt, int ax,const BoundingBox& box)
        : point(pt), axis(ax),bbox(box), left(nullptr), right(nullptr) {}
};
