#ifndef OCTREE_H
#define OCTREE_H

#include <vector>
#include "CNode.h"

class Octree {
public:
    std::unique_ptr<CNode> root;
    int maxDepth;

    Octree(float centerX, float centerY, float centerZ, float size, int depth)
        : maxDepth(depth) {
        root = std::make_unique<CNode>(0, centerX, centerY, centerZ, size);
    }

    void build(const std::vector<Point3D>& points);
};

#endif // OCTREE_H
