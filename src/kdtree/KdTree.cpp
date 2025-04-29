#include "kdtree/KdTree.h"
#include <algorithm>
#include <iostream>
#include <functional>


// Referencia al paper de Bentley (1975)
// https://doi.org/10.1145/361002.361007

void KdTree::build(std::vector<Point3D*>& pts) {
    if (root) {
        delete root;
        root = nullptr;
    }

    root = buildRecursive(pts, 0);
}

KdNode* KdTree::getRoot() const {
    return root;
}

KdNode* KdTree::buildRecursive(std::vector<Point3D*>& pts, int depth) {
    if (pts.empty()) return nullptr;

    // 🛑 Si no hay suficientes puntos, no seguimos dividiendo
    if (pts.size() <= MAX_POINTS_PER_LEAF)
        return new KdNode(pts[0], depth % 3); // o promedio si querés precisión

    int axis = depth % 3;
    size_t mid = pts.size() / 2;

    std::nth_element(pts.begin(), pts.begin() + mid, pts.end(),
        [axis](Point3D* a, Point3D* b) {
            if (axis == 0) return a->x < b->x;
            if (axis == 1) return a->y < b->y;
            return a->z < b->z;
        });

    KdNode* node = new KdNode(pts[mid], axis);

    std::vector<Point3D*> left(pts.begin(), pts.begin() + mid);
    std::vector<Point3D*> right(pts.begin() + mid + 1, pts.end());

    node->left = buildRecursive(left, depth + 1);
    node->right = buildRecursive(right, depth + 1);

    return node;
}

void KdTree::diagnose() const {
    int totalNodes = 0;
    int leafNodes = 0;
    int maxDepth = 0;

    std::function<void(KdNode*, int)> traverse;
    traverse = [&](KdNode* node, int depth) {
        if (!node) return;
        totalNodes++;
        if (!node->left && !node->right)
            leafNodes++;
        if (depth > maxDepth)
            maxDepth = depth;
        traverse(node->left, depth + 1);
        traverse(node->right, depth + 1);
    };

    traverse(root, 0);

    std::cout << "------ Diagnóstico KdTree ------\n";
    std::cout << "Total de nodos:     " << totalNodes << "\n";
    std::cout << "Nodos hoja:         " << leafNodes << "\n";
    std::cout << "Profundidad máxima: " << maxDepth << "\n";
    std::cout << "--------------------------------\n";
}

void KdTree::queryRegion(float cx, float cy, float cz, float size, std::vector<Point3D*>& result) const {
    float half = size / 2.0f;

    std::function<void(KdNode*)> traverse;
    traverse = [&](KdNode* node) {
        if (!node || !node->point) return;

        float x = node->point->x;
        float y = node->point->y;
        float z = node->point->z;

        // ✅ Chequeo de inclusión
        if (x >= cx - half && x <= cx + half &&
            y >= cy - half && y <= cy + half &&
            z >= cz - half && z <= cz + half) {
            result.push_back(node->point);
        }

        // ✅ Poda inteligente según eje de división
        int axis = node->axis;
        float coord = (axis == 0) ? x : (axis == 1) ? y : z;
        float minBound = (axis == 0) ? cx - half : (axis == 1) ? cy - half : cz - half;
        float maxBound = (axis == 0) ? cx + half : (axis == 1) ? cy + half : cz + half;

        if (minBound <= coord && node->left)
            traverse(node->left);
        if (maxBound >= coord && node->right)
            traverse(node->right);
    };

    traverse(root);
}

void KdTree::buildFromPoints(const std::vector<Point3D>& points) {
    // Limpia árbol anterior si lo hubiera
    if (root) {
        delete root;
        root = nullptr;
    }

    std::vector<Point3D*> pointPtrs;
    pointPtrs.reserve(points.size());
    for (const auto& p : points) {
        pointPtrs.push_back(new Point3D(p));
    }

    root = buildRecursive(pointPtrs, 0); // Tu función recursiva interna
}

