#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <GLFW/glfw3.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include "octree/Octree.h"
#include "octree/OctreePoint.h"
#include "octree/Vec3.h"

// variables globales
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
Octree* octreePtr = nullptr;
int currentLevel = 0;

void drawPartition() {
    if (octreePtr) {
        drawOctree(*octreePtr, currentLevel);
    }
}

void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (action == GLFW_PRESS) {
        if (key == GLFW_KEY_KP_ADD)    currentLevel++;
        else if (key == GLFW_KEY_KP_SUBTRACT && currentLevel > 0) currentLevel--;
        // aquí podrías llamar a un método que “activa” solo los nodos hasta currentLevel
    }
}

int main() {
    // 1) Carga PCD
    cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("data/corridor_telin.pcd", *cloud) == -1) {
        std::cerr << "No puedo leer data/corridor_telin.pcd\n";
        return -1;
    }

    // 2) Calcula bounding box
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(*cloud, minPt, maxPt);
    Vec3 center((minPt.x+maxPt.x)/2, (minPt.y+maxPt.y)/2, (minPt.z+maxPt.z)/2);
    Vec3 halfDim((maxPt.x-minPt.x)/2, (maxPt.y-minPt.y)/2, (maxPt.z-minPt.z)/2);

    // 3) Construye octree
    octreePtr = new Octree(center, halfDim);
    for (auto& pt : cloud->points) {
        OctreePoint* p = new OctreePoint(Vec3(pt.x,pt.y,pt.z));
        octreePtr->insert(p);
    }

    // 4) Inicializa GLFW + GLUT
    glfwInit();
    GLFWwindow* window = glfwCreateWindow(800, 600, "Viewer", nullptr, nullptr);
    glfwMakeContextCurrent(window);
    glfwSetKeyCallback(window, keyCallback);
    glutInit(nullptr, nullptr);

    // Loop principal
    while (!glfwWindowShouldClose(window)) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        drawPartition();
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // Cleanup
    delete octreePtr;
    glfwTerminate();
    return 0;
}
