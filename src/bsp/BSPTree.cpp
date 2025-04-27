#include "bsp/BSPTree.h"
#include <algorithm>

BSPTree::BSPTree() : root(nullptr) {}

BSPTree::~BSPTree() {
    delete root;
}

void BSPTree::build(std::vector<Point3D*>& points, int maxDepth) {
    delete root;
    root = buildRecursive(points, 0, maxDepth);
}

BSPNode* BSPTree::getRoot() const {
    return root;
}

BSPNode* BSPTree::buildRecursive(std::vector<Point3D*>& points, int depth, int maxDepth) {
    if (points.empty() || depth >= maxDepth) {
        BSPNode* leaf = new BSPNode();
        leaf->points = points;
        return leaf;
    }

    int axis = depth % 3;

    // Ordenar puntos en el eje actual
    std::sort(points.begin(), points.end(), [axis](Point3D* a, Point3D* b) {
        if (axis == 0) return a->x < b->x;
        if (axis == 1) return a->y < b->y;
        return a->z < b->z;
    });

    // Elegir la mediana
    size_t midIndex = points.size() / 2;
    Point3D* medianPoint = points[midIndex];
    float splitValue = (axis == 0) ? medianPoint->x : (axis == 1) ? medianPoint->y : medianPoint->z;

    // Crear plano de corte
    Point3D normal = {0, 0, 0};
    if (axis == 0) normal.x = 1.0f;
    else if (axis == 1) normal.y = 1.0f;
    else normal.z = 1.0f;
    
    Plane divider(normal, splitValue);

    // Particionar
    std::vector<Point3D*> frontPoints(points.begin() + midIndex, points.end());
    std::vector<Point3D*> backPoints(points.begin(), points.begin() + midIndex);

    BSPNode* node = new BSPNode();
    node->dividingPlane = divider;
    node->front = buildRecursive(frontPoints, depth + 1, maxDepth);
    node->back = buildRecursive(backPoints, depth + 1, maxDepth);

    return node;
}
