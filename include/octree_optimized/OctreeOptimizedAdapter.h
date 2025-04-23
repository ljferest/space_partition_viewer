#pragma once
#include "IOctreeAdapter.h"
#include "octree_optimized/Octree.h" // tu implementación optimizada
#include "octree_optimized/Point3D.h"

class OctreeOptimizedAdapter : public IOctreeAdapter {
    Octree* tree;
    std::vector<Point3D> pointStorage;

public:
    OctreeOptimizedAdapter(const Vec3& center, const Vec3& halfSize) {
        float size = std::max({halfSize.x, halfSize.y, halfSize.z}) * 2.0f;
        tree = new Octree(center.x, center.y, center.z, size, 5); // maxDepth = 5
    }

    ~OctreeOptimizedAdapter() {
        delete tree;
    }

    void insertPoint(float x, float y, float z) override {
        pointStorage.push_back({x, y, z});
    }

    std::vector<OctreeDrawableNode> getDrawableNodes(int level) const override {
        std::vector<OctreeDrawableNode> nodes;
        collectLeafNodes(tree->root.get(), nodes, level, 0);
        return nodes;
    }
    void build();
    
    void collectLeafNodes(CNode* node, std::vector<OctreeDrawableNode>& out, int targetLevel, int currentLevel) const {
        if (!node) return;
    
        if (node->isLeaf || currentLevel == targetLevel) {
            out.push_back({
                Vec3(node->centerX, node->centerY, node->centerZ),
                Vec3(node->size / 2.0f, node->size / 2.0f, node->size / 2.0f)
            });
            return;
        }
    
        for (const auto& child : node->children) {
            if (child) {
                collectLeafNodes(child.get(), out, targetLevel, currentLevel + 1);
            }
        }
    }
    
};
