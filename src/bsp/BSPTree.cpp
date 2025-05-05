#include "bsp/BSPTree.h"
#include <algorithm>

BSPTree::BSPTree() : root(nullptr) {}

BSPTree::~BSPTree() {
    delete root;
}

void BSPTree::build(std::vector<Point3D*>& points, int maxDepth) {
    delete root;
    nextNodeId = 0;
    root = buildRecursive(points, 0, maxDepth);
}

BSPNode* BSPTree::getRoot() const {
    return root;
}

BSPNode* BSPTree::buildRecursive(std::vector<Point3D*>& points, int depth, int maxDepth) {
    constexpr int minPoints = 50; // Puedes ajustar este valor

    if (points.empty() || depth >= maxDepth || points.size() <= minPoints) {
        BSPNode* leaf = new BSPNode(nextNodeId++);
        leaf->points = points;
        return leaf;
    }

    // Calcular medias
    float meanX = 0, meanY = 0, meanZ = 0;
    for (auto p : points) {
        meanX += p->x;
        meanY += p->y;
        meanZ += p->z;
    }
    meanX /= points.size();
    meanY /= points.size();
    meanZ /= points.size();

    // Calcular varianzas
    float varX = 0, varY = 0, varZ = 0;
    for (auto p : points) {
        varX += (p->x - meanX) * (p->x - meanX);
        varY += (p->y - meanY) * (p->y - meanY);
        varZ += (p->z - meanZ) * (p->z - meanZ);
    }

    // Elegir eje de mayor varianza
    int axis = 0; // 0->X, 1->Y, 2->Z
    if (varY > varX && varY >= varZ) axis = 1;
    else if (varZ > varX && varZ >= varY) axis = 2;

    // Encontrar mediana por nth_element (O(n))
    size_t midIndex = points.size() / 2;
    std::nth_element(points.begin(), points.begin() + midIndex, points.end(),
        [axis](Point3D* a, Point3D* b) {
            if (axis == 0) return a->x < b->x;
            if (axis == 1) return a->y < b->y;
            return a->z < b->z;
        });

    Point3D* medianPoint = points[midIndex];
    float splitValue = (axis == 0) ? medianPoint->x : (axis == 1) ? medianPoint->y : medianPoint->z;

    // Crear plano de corte
    Point3D normal = {0, 0, 0};
    if (axis == 0) normal.x = 1.0f;
    else if (axis == 1) normal.y = 1.0f;
    else normal.z = 1.0f;
    
    Plane divider(normal, splitValue);

    // Separar en dos sin copiar innecesariamente
    std::vector<Point3D*> frontPoints;
    std::vector<Point3D*> backPoints;
    frontPoints.reserve(points.size() / 2);
    backPoints.reserve(points.size() / 2);

    for (auto p : points) {
        float coord = (axis == 0) ? p->x : (axis == 1) ? p->y : p->z;
        if (coord >= splitValue)
            frontPoints.push_back(p);
        else
            backPoints.push_back(p);
    }

    BSPNode* node = new BSPNode(nextNodeId++);
    node->dividingPlane = divider;
    node->front = buildRecursive(frontPoints, depth + 1, maxDepth);
    node->back = buildRecursive(backPoints, depth + 1, maxDepth);

    return node;
}

