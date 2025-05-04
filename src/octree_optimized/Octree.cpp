#include "Octree.h"
#include <iostream>

void Octree::build(const std::vector<Point3D*>& points) {
    std::cout << "[DEBUG] build(): puntos recibidos = " << points.size() << std::endl;
    for (auto* pt : points) {
        root->addPoint(pt, maxDepth);
    }
    std::cout << "[DEBUG] build(): root creado = " << (root ? "sí" : "NO") << std::endl;

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

void Octree::movePoint(Point3D* pt, float newX, float newY, float newZ) {
    auto it = pointToNodeMap.find(pt);
    if (it == pointToNodeMap.end()) return;

    CNode* current = it->second;
    current->removePoint(pt);

    pt->x = newX;
    pt->y = newY;
    pt->z = newZ;

    root->insertPointWithMap(pt, maxDepth, pointToNodeMap);
}

void Octree::traverse(std::function<void(bool isLeaf, float cx, float cy, float cz, float size, const std::vector<Point3D*>&)> visitor) {
    std::function<void(CNode*)> recurse = [&](CNode* node) {
        if (!node || node->points.empty()) return;

        bool isLeaf = node->children.empty();
        visitor(isLeaf, node->centerX, node->centerY, node->centerZ, node->size, node->points);

        for (auto& child : node->children) {
            recurse(child.get());
        }
    };

    recurse(root.get());
}
