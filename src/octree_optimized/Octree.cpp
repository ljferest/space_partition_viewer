#include "Octree.h"

void Octree::build(const std::vector<Point3D>& points) {
    for (const auto& pt : points) {
        root->addPoint(const_cast<Point3D*>(&pt), maxDepth);
    }
}

void Octree::queryRegion(float minX, float minY, float minZ,
                         float maxX, float maxY, float maxZ,
                         std::vector<Point3D*>& result) {
    if (root) {
        root->getPointsInBox(minX, minY, minZ, maxX, maxY, maxZ, result);
    }
}

void Octree::traverseLeavesUpToDepth(int renderDepth,
    std::function<void(float, float, float, float, const std::vector<Point3D*>&)> visitor) {
    if (root)
        root->traverseLeavesUpToDepth(renderDepth, visitor);
}

