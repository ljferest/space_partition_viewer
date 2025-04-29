#ifndef OCTREERENDERER_H
#define OCTREERENDERER_H

#include "Octree.h"
#include "common/Point3D.h"
#include <vector>
#include <memory>
#include "kdtree/KdTree.h"
#include "common/BoundingBox.h"
#include "bsp/BSPTree.h"  






class PartitionRenderer {
public:
    enum class RenderMode {
        Octree,
        KdTree,
        BSP
    };
    PartitionRenderer();
    void loadPointCloud(const std::vector<Point3D>& pts);
    float getCenterX() const { return centerX; }
    float getCenterY() const { return centerY; }
    float getCenterZ() const { return centerZ; }
    float getSceneSize() const { return sceneSize; }
    void diagnoseOctree();
    void increaseResolution(); // + tecla
    void decreaseResolution(); // - tecla
    void render(bool wireframe = false);
    void handleKeyboard(unsigned char key);
    void setRenderMode(RenderMode mode);
    void renderKdTreePartitioning(KdNode* node,
        float xmin, float xmax,
        float ymin, float ymax,
        float zmin, float zmax,
        int depth); // ✅ ahora coincide con el .cpp

    void computeBoundingBox();
    void renderBSPPartitioning(BSPNode* node, float xmin, float xmax, float ymin, float ymax, float zmin, float zmax, int depth = 0);

    struct Color {
        float r, g, b;
    };
    
    Color getRandomColor();
    void drawPoint(float x, float y, float z, const Color& color);
    void drawBSPRecursive(BSPNode* node);
    
    
    int renderDepth = 4;
    int maxRenderDepth = 10;



private:
    std::vector<Point3D> points;
    std::vector<Point3D*> pointPtrs;
    std::unique_ptr<Octree> tree;
    float minZ = 0.0f, maxZ = 1.0f;
    float centerX = 0, centerY = 0, centerZ = 0;
    float sceneSize = 100;
    RenderMode currentMode = RenderMode::BSP;
    BoundingBox bbox;
    KdTree kdtree;
    BSPTree bspTree;
    




    void computeZRange();
};

// Declaración de utilidad para medir tiempo
template <typename Func>
void measureExecutionTime(const std::string& label, Func functionToMeasure);


#endif
