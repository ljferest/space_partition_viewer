#ifndef OCTREE_H
#define OCTREE_H

#include <vector>
#include <memory>
#include "CNode.h"
#include "common/Point3D.h"

class Octree {
public:
    std::unique_ptr<CNode> root;
    int maxDepth;

    Octree(float centerX, float centerY, float centerZ, float size, int depth)
        : maxDepth(depth) {
        root = std::make_unique<CNode>(0, centerX, centerY, centerZ, size);
    }

    void build(const std::vector<Point3D>& points);
    
    void queryRegion(float minX, float minY, float minZ,
                     float maxX, float maxY, float maxZ,
                     std::vector<Point3D*>& result);

    void traverseLeavesUpToDepth(int renderDepth,
        std::function<void(float, float, float, float, const std::vector<Point3D*>&)> visitor);
                    
};

#endif // OCTREE_H

