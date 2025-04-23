#pragma once
#include "IOctreeAdapter.h"
#include "octree/Octree.h"
#include "octree/OctreePoint.h"
#include "octree/Vec3.h"

class OctreeBasicAdapter : public IOctreeAdapter {
    Octree* tree;
    std::vector<OctreePoint*> insertedPoints;
    Vec3 center;
    Vec3 halfSize;

public:
    OctreeBasicAdapter(const Vec3& c, const Vec3& h) : center(c), halfSize(h) {
        tree = new Octree(center, halfSize);
    }

    ~OctreeBasicAdapter() {
        for (auto p : insertedPoints)
            delete p;
        delete tree;
    }

    void insertPoint(float x, float y, float z) override {
        OctreePoint* pt = new OctreePoint(Vec3(x, y, z));
        insertedPoints.push_back(pt);
        tree->insert(pt);
    }

    std::vector<OctreeDrawableNode> getDrawableNodes(int level) const override {
        std::vector<OctreeDrawableNode> nodes;
        // Por ahora, devolvemos solo la raíz
        nodes.push_back({center, halfSize});
        // En el futuro: filtrar por nivel si Octree lo permite
        return nodes;
    }
};
