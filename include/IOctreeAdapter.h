#pragma once

#include <vector>
#include "octree_basic/Vec3.h"

struct OctreeDrawableNode {
    Vec3 center;
    Vec3 halfSize;
};

class IOctreeAdapter {
public:
    virtual ~IOctreeAdapter() = default;
    virtual void insertPoint(float x, float y, float z) = 0;
    virtual std::vector<OctreeDrawableNode> getDrawableNodes(int level) const = 0;
};
