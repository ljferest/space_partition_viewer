#include "kdtree/KdTree.h"
#include <algorithm>
#include <iostream>
#include <functional>
#include <limits>
#include <cmath>


// Referencia al paper de Bentley (1975)
// https://doi.org/10.1145/361002.361007

void KdTree::build(std::vector<Point3D*>& pts) {
    this->points = &pts;

    BoundingBox fullBox;
    for (auto* p : *points) {
        fullBox.expandToInclude(*p);
    }    

    int N = static_cast<int>(points->size());

    // 📈 Cálculo dinámico de parámetros
    int maxDepth = std::ceil(std::log2(N));
    int minPointsPerLeaf = std::max(2, N / (1 << (maxDepth - 1)));

    std::cout << "[KDTree] Construyendo con N=" << N
              << ", maxDepth=" << maxDepth
              << ", minPointsPerLeaf=" << minPointsPerLeaf << "\n";

    root = buildRecursive(*points, 0, fullBox, maxDepth, minPointsPerLeaf);
}

KdNode* KdTree::getRoot() const {
    return root;
}

KdNode* KdTree::buildRecursive(std::vector<Point3D*>& pts, int depth,
    const BoundingBox& box,
    int maxDepth, int minPointsPerLeaf) {
if (pts.empty()) return nullptr;

if (depth >= maxDepth || pts.size() <= minPointsPerLeaf) {
Point3D* median_point = pts[pts.size() / 2];
return new KdNode(median_point, depth % 3, box);
}

int axis = depth % 3;

std::nth_element(pts.begin(), pts.begin() + pts.size() / 2, pts.end(),
[axis](Point3D* a, Point3D* b) {
if (axis == 0) return a->x < b->x;
if (axis == 1) return a->y < b->y;
return a->z < b->z;
});

size_t median_idx = pts.size() / 2;
Point3D* median_point = pts[median_idx];

std::vector<Point3D*> left_pts(pts.begin(), pts.begin() + median_idx);
std::vector<Point3D*> right_pts(pts.begin() + median_idx + 1, pts.end());

BoundingBox left_box = box;
BoundingBox right_box = box;

float split_value = (axis == 0 ? median_point->x :
axis == 1 ? median_point->y :
          median_point->z);

if (axis == 0) {
left_box.max.x = split_value;
right_box.min.x = split_value;
} else if (axis == 1) {
left_box.max.y = split_value;
right_box.min.y = split_value;
} else {
left_box.max.z = split_value;
right_box.min.z = split_value;
}

KdNode* node = new KdNode(median_point, axis, box);
node->left = buildRecursive(left_pts, depth + 1, left_box, maxDepth, minPointsPerLeaf);
node->right = buildRecursive(right_pts, depth + 1, right_box, maxDepth, minPointsPerLeaf);

return node;
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
/*
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
}*/

BoundingBox computeBoundingBoxFromPoints(const std::vector<Point3D*>& pts) {
    BoundingBox box;

    if (pts.empty()) {
        std::cerr << "[ERROR] computeBoundingBoxFromPoints(): No hay puntos.\n";
        box.min = Point3D(0, 0, 0);
        box.max = Point3D(1, 1, 1);  // fallback
        return box;
    }

    float minX = std::numeric_limits<float>::max();
    float minY = std::numeric_limits<float>::max();
    float minZ = std::numeric_limits<float>::max();
    float maxX = std::numeric_limits<float>::lowest();
    float maxY = std::numeric_limits<float>::lowest();
    float maxZ = std::numeric_limits<float>::lowest();

    for (const auto* p : pts) {
        if (!p) continue;
        minX = std::min(minX, p->x);
        minY = std::min(minY, p->y);
        minZ = std::min(minZ, p->z);
        maxX = std::max(maxX, p->x);
        maxY = std::max(maxY, p->y);
        maxZ = std::max(maxZ, p->z);
    }

    // 🧱 Asegurar volumen mínimo si está colapsado
    const float minLen = 0.01f;
    if (maxX - minX < minLen) maxX = minX + minLen;
    if (maxY - minY < minLen) maxY = minY + minLen;
    if (maxZ - minZ < minLen) maxZ = minZ + minLen;

    box.min = Point3D(minX, minY, minZ);
    box.max = Point3D(maxX, maxY, maxZ);
    return box;
}

void KdTree::diagnose(int& outMaxDepth) const {
    int totalNodes = 0;
    int leafNodes = 0;
    int maxDepth = 0;
    double totalLeafVolume = 0.0;

    std::function<void(KdNode*, int)> traverse;
    traverse = [&](KdNode* node, int depth) {
        if (!node) return;

        totalNodes++;
        if (!node->left && !node->right) {
            leafNodes++;

            const auto& box = node->bbox;
            float dx = box.max.x - box.min.x;
            float dy = box.max.y - box.min.y;
            float dz = box.max.z - box.min.z;
            float volume = dx * dy * dz;
            totalLeafVolume += volume;
        }

        if (depth > maxDepth)
            maxDepth = depth;

        traverse(node->left, depth + 1);
        traverse(node->right, depth + 1);
    };

    traverse(root, 0);

    double avgLeafVolume = (leafNodes > 0) ? totalLeafVolume / leafNodes : 0.0;

    std::cout << "------ Diagnóstico KdTree ------\n";
    std::cout << "Total de nodos:         " << totalNodes << "\n";
    std::cout << "Nodos hoja:             " << leafNodes << "\n";
    std::cout << "Profundidad máxima:     " << maxDepth << "\n";
    std::cout << "Volumen promedio hojas: " << avgLeafVolume << "\n";
    std::cout << "--------------------------------\n";

    outMaxDepth = maxDepth;  // devolver profundidad calculada
}