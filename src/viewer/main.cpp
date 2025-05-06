#include <GL/glut.h>
#include <vector>
#include <iostream>
#include <stdexcept>
#include "PartitionRenderer.h"
#include "common/Point3D.h"
#include "common/PointCloudLoader.h"

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// Globals
PartitionRenderer renderer;
bool showWireframe = false;
bool wireframeMode = false;

// Load cloud from PCD file
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
    float min_x = std::numeric_limits<float>::max(), max_x = -std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max(), max_y = -std::numeric_limits<float>::max();
    float min_z = std::numeric_limits<float>::max(), max_z = -std::numeric_limits<float>::max();

    for (const auto& pt : result) {
        min_x = std::min(min_x, pt.x);
        max_x = std::max(max_x, pt.x);
        min_y = std::min(min_y, pt.y);
        max_y = std::max(max_y, pt.y);
        min_z = std::min(min_z, pt.z);
        max_z = std::max(max_z, pt.z);
    }

    float center_x = (min_x + max_x) / 2.0f;
    float center_y = (min_y + max_y) / 2.0f;
    float center_z = (min_z + max_z) / 2.0f;
    float radius = std::max({max_x - min_x, max_y - min_y, max_z - min_z}) / 2.0f;

    renderer.setCameraTarget(center_x, center_y, center_z);
    renderer.setCameraDistance(radius * 3.0f);

    return result;
}

// to display the scene
// This function is called every time the window needs to be redrawn
void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glPolygonMode(GL_FRONT_AND_BACK, wireframeMode ? GL_LINE : GL_FILL);
    glPushMatrix();
    renderer.render(showWireframe);
    glPopMatrix();
    glutSwapBuffers();
}

// to reshape the window
void reshape(int w, int h) {
    if (h == 0) h = 1;
    float ratio = 1.0f * w / h;
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45, ratio, 0.1, 1000);
    glMatrixMode(GL_MODELVIEW);
    glViewport(0, 0, w, h);
}

// to handle keyboard input
// This function is called every time a key is pressed
void keyboard(unsigned char key, int x, int y) {
    switch (key) {
    case 27: 
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
        renderer.setWireframeMode(wireframeMode);
        glutPostRedisplay();
        break;

    default:
        renderer.handleKeyboard(key); 
        break;
    }
}

// to initialize OpenGL settings
// This function is called once at the beginning of the program
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
        cx + size, cy + size, cz + size,  
        cx, cy, cz,                       
        0, 0, 1                           
    );
}

// Main function where the program starts and initializes everything
int main(int argc, char** argv) {
    try {
        glutInit(&argc, argv);
        glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
        glutInitWindowSize(800, 600);
        glutCreateWindow("Viewer");

        initGL();

        //Download and load the point cloud
        std::vector<Point3D> cloud = loadFromPCD("../data/bunny.pcd");
        renderer.loadPointCloud(cloud);

        /*
        std::cout << "[INFO] Puntos cargados: " << cloud.size() << std::endl;
        for (int i = 0; i < 10 && i < cloud.size(); ++i) {
            std::cout << "  P[" << i << "]: " << cloud[i].x << ", " << cloud[i].y << ", " << cloud[i].z << std::endl;
        }
        */

        glutDisplayFunc(display);
        glutReshapeFunc(reshape);
        glutKeyboardFunc(keyboard);
        glutSpecialFunc([](int key, int x, int y) {
            renderer.specialCallback(key, x, y);
        });        
        glutPassiveMotionFunc([](int x, int y) {
            renderer.mouseMotionCallback(x, y);
        });        
        glutMainLoop();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
