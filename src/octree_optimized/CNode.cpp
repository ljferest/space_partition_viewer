#include "CNode.h"
#include <cmath>
#include <iostream>

// To get the child index based on the point's position relative to the center.
int CNode::getChildIndex(Point3D* pt) const {
    return (pt->x > centerX) * 4 + (pt->y > centerY) * 2 + (pt->z > centerZ);
}

// Add a point to the node, subdividing if necessary with the subdivide method.
void CNode::addPoint(Point3D* pt, int maxDepth, int maxPointsPerLeaf) {
    if (depth == maxDepth || (isLeaf && points.size() < maxPointsPerLeaf)) {
        points.push_back(pt);
        return;
    }

    if (isLeaf) {
        subdivide(maxDepth);
        for (auto* p : points) {
            int idx = getChildIndex(p);
            children[idx]->addPoint(p, maxDepth, maxPointsPerLeaf);
        }
        points.clear();
        isLeaf = false;
    }

    int idx = getChildIndex(pt);
    children[idx]->addPoint(pt, maxDepth, maxPointsPerLeaf);
}

// Subdivide the node into 8 children.
void CNode::subdivide(int maxDepth) {
    float h = size / 2.0f;
    for (int i = 0; i < 8; ++i) {
        float dx = (i & 4) ? h : -h;
        float dy = (i & 2) ? h : -h;
        float dz = (i & 1) ? h : -h;
        children[i] = std::make_unique<CNode>(
            depth + 1,
            centerX + dx / 2.0f,
            centerY + dy / 2.0f,
            centerZ + dz / 2.0f,
            h
        );
    }
}

// To get the index of the child node where the point belongs.
void CNode::getPointsInBox(float minX, float minY, float minZ,
                            float maxX, float maxY, float maxZ,
                            std::vector<Point3D*>& result) {
    float nodeMinX = centerX - size / 2, nodeMaxX = centerX + size / 2;
    float nodeMinY = centerY - size / 2, nodeMaxY = centerY + size / 2;
    float nodeMinZ = centerZ - size / 2, nodeMaxZ = centerZ + size / 2;

    if (nodeMaxX < minX || nodeMinX > maxX ||
        nodeMaxY < minY || nodeMinY > maxY ||
        nodeMaxZ < minZ || nodeMinZ > maxZ)
        return;

    if (isLeaf) {
        for (auto* pt : points) {
            if (pt->x >= minX && pt->x <= maxX &&
                pt->y >= minY && pt->y <= maxY &&
                pt->z >= minZ && pt->z <= maxZ)
                result.push_back(pt);
        }
    } else {
        for (auto& child : children)
            if (child) child->getPointsInBox(minX, minY, minZ, maxX, maxY, maxZ, result);
    }
}

// Traverse the leaves of the octree and apply the visitor function.
void CNode::traverseLeaves(std::function<void(float, float, float, float, const std::vector<Point3D*>&)> visitor) {
    if (isLeaf) {
        visitor(centerX, centerY, centerZ, size, points);
    } else {
        for (auto& child : children) {
            if (child) child->traverseLeaves(visitor);
        }
    }
}

// Traverse the leaves of the octree up to a certain depth and apply the visitor function.
void CNode::traverseLeavesUpToDepth(int maxRenderDepth,
    std::function<void(float, float, float, float, const std::vector<Point3D*>&)> visitor) {

    /*
    std::cout << "[TRACE] Nodo depth=" << depth
              << ", isLeaf=" << (isLeaf ? "true" : "false")
              << ", size=" << size
              << ", puntos=" << points.size() << "\n";
    */
    if (depth == maxRenderDepth || isLeaf) {
        if(isLeaf && points.empty()) {
            //std::cout << "  >> NO HAY PUNTOS en este nodo\n";
            return;

        }
        else{
        //std::cout << "  >> DIBUJANDO este nodo\n";
        visitor(centerX, centerY, centerZ, size, points);
        }
    } else {
        //std::cout << "  >> PROFUNDIZANDO en hijos\n";
        for (auto& child : children)
            if (child) child->traverseLeavesUpToDepth(maxRenderDepth, visitor);
    }
}

// Remove a point from the node.
void CNode::removePoint(Point3D* pt) {
    auto it = std::find(points.begin(), points.end(), pt);
    if (it != points.end()) {
        points.erase(it);
    }
}

// To insert a point into the octree and map it to the node.
void CNode::insertPointWithMap(Point3D* pt, int maxDepth, std::unordered_map<Point3D*, CNode*>& map, int maxPointsPerLeaf) {
    if (depth == maxDepth || (isLeaf && points.size() < maxPointsPerLeaf)) {
        points.push_back(pt);
        map[pt] = this;
        return;
    }

    if (isLeaf) {
        subdivide(maxDepth);
    }

    int index = getChildIndex(pt);
    children[index]->insertPointWithMap(pt, maxDepth, map, maxPointsPerLeaf);
}
