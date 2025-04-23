#ifndef CNODE_H
#define CNODE_H

#include <vector>
#include <memory>
#include "Point3D.h"

class CNode {
public:
    int depth;
    float centerX, centerY, centerZ, size;
    std::vector<Point3D*> points;
    std::unique_ptr<CNode> children[8];
    bool isLeaf = true;

    CNode(int d, float cx, float cy, float cz, float s)
        : depth(d), centerX(cx), centerY(cy), centerZ(cz), size(s) {}

    void addPoint(Point3D* pt, int maxDepth);
};

#endif // CNODE_H
