#include "CNode.h"

void CNode::addPoint(Point3D* pt, int maxDepth) {
    if (depth == maxDepth) {
        points.push_back(pt);
        return;
    }

    int index = 0;
    if (pt->x > centerX) index |= 1;
    if (pt->y > centerY) index |= 2;
    if (pt->z > centerZ) index |= 4;

    if (!children[index]) {
        float offset = size / 4.0f;
        float childSize = size / 2.0f;
        float childCenterX = centerX + ((index & 1) ? offset : -offset);
        float childCenterY = centerY + ((index & 2) ? offset : -offset);
        float childCenterZ = centerZ + ((index & 4) ? offset : -offset);
        children[index] = std::make_unique<CNode>(depth + 1, childCenterX, childCenterY, childCenterZ, childSize);
    
        isLeaf = false;
    }

    children[index]->addPoint(pt, maxDepth);
}
