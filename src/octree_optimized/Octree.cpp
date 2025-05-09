#include "Octree.h"
#include <iostream>
#include <limits>

//To build the octree, we need to define the center and size of the root node.
void Octree::build(const std::vector<Point3D*>& points) {
    std::cout << "[DEBUG] build(): puntos recibidos = " << points.size() << std::endl;
    for (auto* pt : points) {
        root->addPoint(pt, maxDepth, maxPointsPerLeaf);
    }
    std::cout << "[DEBUG] build(): root creado = " << (root ? "sí" : "NO") << std::endl;

}

// The queryRegion function retrieves all points within a specified 3D box.
void Octree::queryRegion(float minX, float minY, float minZ,
                         float maxX, float maxY, float maxZ,
                         std::vector<Point3D*>& result) {
    if (root) {
        root->getPointsInBox(minX, minY, minZ, maxX, maxY, maxZ, result);
    }
}

// The traverseLeavesUpToDepth function allows the user to visit all leaves of the octree up to a specified depth.
void Octree::traverseLeavesUpToDepth(int renderDepth,
    std::function<void(float, float, float, float, const std::vector<Point3D*>&)> visitor) {
    if (root)
        root->traverseLeavesUpToDepth(renderDepth, visitor);
}

// The movePoint function updates the position of a point in the octree.
void Octree::movePoint(Point3D* pt, float newX, float newY, float newZ) {
    auto it = pointToNodeMap.find(pt);
    if (it == pointToNodeMap.end()) return;

    CNode* current = it->second;
    current->removePoint(pt);

    pt->x = newX;
    pt->y = newY;
    pt->z = newZ;

    root->insertPointWithMap(pt, maxDepth, pointToNodeMap, maxPointsPerLeaf);
}

// The traverse function allows the user to visit all nodes in the octree for the wireframe.
void Octree::traverse(std::function<void(bool isLeaf, float cx, float cy, float cz, float size, const std::vector<Point3D*>&)> visitor) {
    std::function<void(CNode*)> recurse = [&](CNode* node) {
        if (!node) return;

        bool isLeaf = node->children.empty();
        visitor(isLeaf, node->centerX, node->centerY, node->centerZ, node->size, node->points);

        for (auto& child : node->children) {
            recurse(child.get());
        }
    };

    recurse(root.get());
}

// The diagnose function provides a summary of the octree's structure, including the number of leaves, non-empty leaves, and their sizes.
void Octree::diagnose(int& maxDepth) const {
    int totalLeaves = 0;
    int nonEmptyLeaves = 0;
    float minSize = 1e6f, maxSize = -1.0f;
    maxDepth = 0;

    //std::cout << "[DEBUG] Entrando a diagnose. Root = " << root.get() << "\n";

    std::function<void(const CNode*, int)> recurse = [&](const CNode* node, int depth) {
        if (!node) return;

        bool isLeaf = true;
        for (const auto& child : node->children) {
            if (child) {
                isLeaf = false;
                break;
            }
        }

        if (isLeaf) {
            totalLeaves++;
            if (!node->points.empty()) {
                nonEmptyLeaves++;
                minSize = std::min(minSize, node->size);
                maxSize = std::max(maxSize, node->size);
            }
            maxDepth = std::max(maxDepth, depth);
        }

        for (const auto& child : node->children) {
            if (child) recurse(child.get(), depth + 1);
        }
    };

    recurse(root.get(), 0);

    std::cout << "------ Diagnóstico Octree ------\n";
    std::cout << "Total de hojas:       " << totalLeaves << "\n";
    std::cout << "Hojas con puntos:     " << nonEmptyLeaves << "\n";
    std::cout << "Tamaño mínimo hoja:   " << (nonEmptyLeaves > 0 ? minSize : 0.0f) << "\n";
    std::cout << "Tamaño máximo hoja:   " << maxSize << "\n";
    std::cout << "Resolución pedida:    " << maxDepth << "\n";
    std::cout << "--------------------------------\n";
}
