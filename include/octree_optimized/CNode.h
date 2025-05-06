#ifndef CNODE_H
#define CNODE_H

#include <vector>
#include <memory>
#include <array>
#include "common/Point3D.h"
#include <functional>

class CNode {
public:
    int depth;
    float centerX, centerY, centerZ, size;
    std::vector<Point3D*> points;
    std::array<std::unique_ptr<CNode>, 8> children;
    bool isLeaf = true;

    static constexpr int MAX_POINTS = 100;

    CNode(int d, float cx, float cy, float cz, float s)
        : depth(d), centerX(cx), centerY(cy), centerZ(cz), size(s) {}

        void addPoint(Point3D* pt, int maxDepth, int maxPointsPerLeaf);
    void subdivide(int maxDepth);
    void getPointsInBox(float minX, float minY, float minZ,
                        float maxX, float maxY, float maxZ,
                        std::vector<Point3D*>& result);
    void traverseLeaves(std::function<void(float, float, float, float, const std::vector<Point3D*>&)> visitor);
    void traverseLeavesUpToDepth(int maxRenderDepth,
        std::function<void(float, float, float, float, const std::vector<Point3D*>&)> visitor);
    void removePoint(Point3D* pt);
    void insertPointWithMap(Point3D* pt, int maxDepth, std::unordered_map<Point3D*, CNode*>& map, int maxPointsPerLeaf);
        
        
    

private:
    int getChildIndex(Point3D* pt) const;
};

#endif
