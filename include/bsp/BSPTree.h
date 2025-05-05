#pragma once

#include "BSPNode.h"
#include <vector>

class BSPTree {
public:
    BSPTree();
    ~BSPTree();

    void build(std::vector<Point3D*>& points, int maxDepth = 10);
    BSPNode* getRoot() const;
    int nextNodeId;

private:
    BSPNode* root;
    BSPNode* buildRecursive(std::vector<Point3D*>& points, int depth, int maxDepth);
    
};
