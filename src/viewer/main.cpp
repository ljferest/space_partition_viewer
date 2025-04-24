#include <GL/glut.h>
#include <vector>
#include <iostream>
#include <stdexcept>
#include "OctreeRenderer.h"
#include "Point3D.h"

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// Globals
OctreeRenderer renderer;
int currentResolution = 10;
bool showWireframe = false;
bool wireframeMode = false;

// Cargar nube de puntos desde .pcd
std::vector<Point3D> loadFromPCD(const std::string& path) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud) == -1) {
        throw std::runtime_error("No se pudo cargar el archivo PCD: " + path);
    }

    std::vector<Point3D> result;
    result.reserve(cloud->size());
    for (const auto& p : cloud->points) {
        result.push_back({p.x, p.y, p.z});
    }
    return result;
}

void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glPushMatrix();
    renderer.render(showWireframe);
    glPopMatrix();
    glutSwapBuffers();
}

void reshape(int w, int h) {
    if (h == 0) h = 1;
    float ratio = 1.0f * w / h;
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45, ratio, 0.1, 1000);
    glMatrixMode(GL_MODELVIEW);
    glViewport(0, 0, w, h);
}

void keyboard(unsigned char key, int x, int y) {
    switch (key) {
    case 27: // ESC
        exit(0);
        break;

    case '+':
        renderer.increaseResolution();
        glutPostRedisplay();
        break;

    case '-':
        renderer.decreaseResolution();
        glutPostRedisplay();
        break;

    case 'w':
    case 'W':
        wireframeMode = !wireframeMode;
        glutPostRedisplay();
        break;

    default:
        break;
    }
}

void initGL() {
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    float cx = renderer.getCenterX();
    float cy = renderer.getCenterY();
    float cz = renderer.getCenterZ();
    float size = renderer.getSceneSize();

    gluLookAt(
        cx + size, cy + size, cz + size,  // cámara alejada en diagonal
        cx, cy, cz,                       // mirando al centro de la escena
        0, 0, 1                           // eje Z como "arriba"
    );
}




int main(int argc, char** argv) {
    try {
        glutInit(&argc, argv);
        glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
        glutInitWindowSize(800, 600);
        glutCreateWindow("Octree Viewer");

        initGL();

        std::vector<Point3D> cloud = loadFromPCD("../data/ufo.pcd");
        std::cout << "[INFO] Puntos cargados: " << cloud.size() << std::endl;
        for (int i = 0; i < 10 && i < cloud.size(); ++i) {
            std::cout << "  P[" << i << "]: " << cloud[i].x << ", " << cloud[i].y << ", " << cloud[i].z << std::endl;
        }

        renderer.loadPointCloud(cloud);
        renderer.setResolution(currentResolution);
        renderer.diagnoseOctree();
        

        glutDisplayFunc(display);
        glutReshapeFunc(reshape);
        glutKeyboardFunc(keyboard);
        glutMainLoop();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}