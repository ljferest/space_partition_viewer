#include "Octree.h"

void Octree::build(const std::vector<Point3D>& points) {
    for (const auto& pt : points) {
        root->addPoint(const_cast<Point3D*>(&pt), maxDepth);
    }
}
