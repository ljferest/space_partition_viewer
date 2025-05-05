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
    void renderKdTreePartitioning(KdNode* node, int depth, int max);

    void computeBoundingBox();
    void renderBSPPartitioning(BSPNode* node, float xmin, float xmax, float ymin, float ymax, float zmin, float zmax, int depth = 0);

    struct Color {
        float r, g, b;
    };
    
    Color getRandomColor();
    void drawPoint(float x, float y, float z, const Color& color);
    void drawBSPRecursive(BSPNode* node);
    
    
    int renderDepth = 4;
    int renderDepthKD;
    int maxRenderDepth = 10;
    
    void setCameraTarget(float x, float y, float z);
    void setCameraDistance(float d);
    void setWireframeMode(bool enabled);
    void mouseMotionCallback(int x, int y);
    void specialCallback(int key, int x, int y);
    void updateCameraDirection();

    bool flyMode = false;
    float camPosX = 0.0f, camPosY = 0.0f, camPosZ = 5.0f;
    float yaw = -90.0f;     // ángulo horizontal
    float pitch = 20.0f;     // ángulo vertical
    float flySpeed = 0.5f;
    float sensitivity = 0.1f;
    float camDirX = 0, camDirY = 0, camDirZ = -1;
    int lastMouseX = 0, lastMouseY = 0;
    bool firstMouse = true;

    


private:
    std::vector<Point3D> points;
    std::vector<Point3D*> pointPtrs;
    std::unique_ptr<Octree> tree;
    float minZ = 0.0f, maxZ = 1.0f;
    float centerX = 0, centerY = 0, centerZ = 0;
    float sceneSize = 100;
    RenderMode currentMode = RenderMode::Octree;
    BoundingBox bbox;
    KdTree kdtree;
    BSPTree bspTree;
    float cameraTargetX = 0.0f;
    float cameraTargetY = 0.0f;
    float cameraTargetZ = 0.0f;
    float cameraDistance = 3.0f;
    bool wireframeMode = false;





    void computeZRange();
};

// Declaración de utilidad para medir tiempo
template <typename Func>
void measureExecutionTime(const std::string& label, Func functionToMeasure);


#endif
