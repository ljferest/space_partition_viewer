#pragma once
#include <vector>
#include "KdNode.h"

class KdTree {
public:
    static constexpr int MAX_POINTS_PER_LEAF = 50;
    KdNode* getRoot() const;

    void build(std::vector<Point3D>& pts);
    void diagnose() const;
    void queryRegion(float cx, float cy, float cz, float size, std::vector<Point3D*>& result) const;
    void buildFromPoints(const std::vector<Point3D>& points);



    

private:
    KdNode* root = nullptr;
    KdNode* buildRecursive(std::vector<Point3D*>& pts, int depth);
};
