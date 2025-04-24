#ifndef OCTREERENDERER_H
#define OCTREERENDERER_H

#include "Octree.h"
#include "Point3D.h"
#include <vector>
#include <memory>

class OctreeRenderer {
public:
    OctreeRenderer();
    void loadPointCloud(const std::vector<Point3D>& pts);
    void setResolution(int depth);
    void render(bool wireframe = false);
    float getCenterX() const { return centerX; }
    float getCenterY() const { return centerY; }
    float getCenterZ() const { return centerZ; }
    float getSceneSize() const { return sceneSize; }
    void diagnoseOctree();
    int renderDepth = 6;
    void increaseResolution(); // + tecla
    void decreaseResolution(); // - tecla




private:
    std::vector<Point3D> points;
    std::unique_ptr<Octree> tree;
    float minZ = 0.0f, maxZ = 1.0f;
    float centerX = 0, centerY = 0, centerZ = 0;
    float sceneSize = 100;


    void computeZRange();
};

#endif
