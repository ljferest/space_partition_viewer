#include <iostream>

// PCL: IO y tipos
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
// Para getMinMax3D()
#include <pcl/common/common.h>

// OpenGL + GLFW + GLUT
#include <GLFW/glfw3.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

// Tus módulos header-only
#include "octree/Vec3.h"
#include "octree/OctreePoint.h"
#include "octree/Octree.h"
#include "OctreeRenderer.h"
// (más tarde) #include "kdtree/KdTree.h"
//             #include "bsp/BSPTree.h"

using namespace brandonpelfrey;

// variables globales
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
Octree* octreePtr = nullptr;
int currentLevel = -1;

Vec3 cameraCenter;
float cameraDistance;




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

int main(int argc, char** argv) {
    // 1) Carga PCD
    cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../data/corridor_telin.pcd", *cloud) == -1) {
        std::cerr << "No puedo leer data/corridor_telin.pcd\n";
        return -1;
    }

    // 2) Calcula bounding box
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(*cloud, minPt, maxPt);

    cameraCenter = Vec3(
        (minPt.x + maxPt.x) * 0.5f,
        (minPt.y + maxPt.y) * 0.5f,
        (minPt.z + maxPt.z) * 0.5f
      );
      Vec3 bbHalfDim = Vec3(
        (maxPt.x - minPt.x) * 0.5f,
        (maxPt.y - minPt.y) * 0.5f,
        (maxPt.z - minPt.z) * 0.5f
      );
      float maxHalf = std::max({bbHalfDim.x, bbHalfDim.y, bbHalfDim.z});
      cameraDistance = maxHalf * 5.0f;

    Vec3 center((minPt.x+maxPt.x)/2, (minPt.y+maxPt.y)/2, (minPt.z+maxPt.z)/2);
    Vec3 halfDim(maxHalf, maxHalf, maxHalf);

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
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);

    // Habilita el test de profundidad y el color de fondo
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);

    // ← Y justo después de ese glClearColor, pega esto:
    int w, h;
    glfwGetFramebufferSize(window, &w, &h);
    glViewport(0, 0, w, h);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    // Campo de visión 45°, aspecto w/h, near=0.1, far=cameraDistance*5
    gluPerspective(45.0, (double)w / h, 0.1, cameraDistance * 10.0);

    glMatrixMode(GL_MODELVIEW);

    // Loop principal
    while (!glfwWindowShouldClose(window)) {
        // 1) Limpia buffers
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
        // 2) Coloca la cámara
        glLoadIdentity();
        gluLookAt(
            cameraCenter.x,                   // eye.x
            cameraCenter.y + cameraDistance,  // eye.y
            cameraCenter.z + cameraDistance,  // eye.z
            cameraCenter.x,                   // center.x
            cameraCenter.y,                   // center.y
            cameraCenter.z,                   // center.z
            0.0, 1.0, 0.0                     // up vector
        );
    
        // — Dibuja la nube de puntos en blanco —
        glPointSize(2.0f);
        glColor3f(1.0f, 1.0f, 1.0f);
        glBegin(GL_POINTS);
        for (auto& pt : cloud->points) {
            glVertex3f(pt.x, pt.y, pt.z);
        }
        glEnd();
    
        // — Dibuja la caja raíz en rojo —
        {
          Vec3 c = center;    // el mismo center que usas para construir el octree
          Vec3 h = halfDim;   // el mismo halfDim
          glColor3f(1.0f, 0.0f, 0.0f);
          glPushMatrix();
          glTranslatef(c.x, c.y, c.z);
          glScalef(h.x * 2, h.y * 2, h.z * 2);
          glutWireCube(1.0f);
          glPopMatrix();
        }
    
        // — Dibuja el Octree en verde —
        drawPartition();
    
        // 4) Swap buffers y procesa eventos
        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    

    // Cleanup
    delete octreePtr;
    glfwTerminate();
    return 0;
}
