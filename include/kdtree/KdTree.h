#pragma once
#include <vector>
#include "KdNode.h"

class KdTree {
public:
    void build(std::vector<Point3D>& pts);
    KdNode* getRoot() const;

private:
    KdNode* root = nullptr;
    KdNode* buildRecursive(std::vector<Point3D*>& pts, int depth);
};
