#include "PartitionRenderer.h"
#include <GL/glut.h>
#include <algorithm>
#include <iostream>
#include <chrono>
#include <cmath>


PartitionRenderer::PartitionRenderer() {
    // Posición inicial de la cámara (centrada en escena luego en loadPointCloud)
    camPosX = 0.0f;
    camPosY = 0.0f;
    camPosZ = 5.0f;

    // Dirección inicial mirando al -Z
    camDirX = 0.0f;
    camDirY = 0.0f;
    camDirZ = -1.0f;

    // Ángulos yaw/pitch coherentes con esa dirección
    yaw = -90.0f;
    pitch = 0.0f;

    sensitivity = 0.1f;
    flySpeed = 0.1f;

    firstMouse = true;
}

void PartitionRenderer::loadPointCloud(const std::vector<Point3D>& pts) {
    points = pts;
    pointPtrs.clear();
    for (auto& p : points) {
        pointPtrs.push_back(&p);
    }

    computeBoundingBox();

    // 🔄 Primero calcula dimensiones del bounding box
    float dx = bbox.max.x - bbox.min.x;
    float dy = bbox.max.y - bbox.min.y;
    float dz = bbox.max.z - bbox.min.z;

    // 🔐 Asegurar que no sean 0
    if (dx < 1e-6f) dx = 1.0f;
    if (dy < 1e-6f) dy = 1.0f;
    if (dz < 1e-6f) dz = 1.0f;

    // ✅ Calcular sceneSize correctamente antes de usarlo
    sceneSize = std::max({ dx, dy, dz }) * 1.1f;

    // 📌 Centro de la escena
    centerX = (bbox.min.x + bbox.max.x) / 2.0f;
    centerY = (bbox.min.y + bbox.max.y) / 2.0f;
    centerZ = (bbox.min.z + bbox.max.z) / 2.0f;

    // 🎯 Posicionar cámara
    cameraTargetX = centerX;
    cameraTargetY = centerY;
    cameraTargetZ = centerZ;
    cameraDistance = sceneSize * 1.5f;  // Alejado para ver todo

    std::cout << "[INFO] Bounding Box: [" << dx << " x " << dy << " x " << dz << "]\n";
    std::cout << "[INFO] Centro: (" << centerX << ", " << centerY << ", " << centerZ << ") Tamaño: " << sceneSize << "\n";
    std::cout << "[INFO] CameraDistance: " << cameraDistance << "\n";

    int maxTreeDepth = 10;
    renderDepth = 4;

    measureExecutionTime("Construcción Octree", [this, maxTreeDepth]() {
        tree = std::make_unique<Octree>(centerX, centerY, centerZ, sceneSize, maxTreeDepth);
        tree->build(pointPtrs);
        std::cout << "[POST-BUILD] tree = " << tree.get()
                  << ", root = " << (tree ? tree->root.get() : nullptr) << std::endl;
    });

    measureExecutionTime("Construcción KD-Tree", [this]() {
        kdtree.build(pointPtrs);
        kdtree.diagnose(maxRenderDepth); //Show stats od kdtree
        renderDepthKD = maxRenderDepth/2;
    });

    measureExecutionTime("Construcción BSP-Tree", [this]() {
        bspTree.build(pointPtrs, 10);
    });

    computeZRange(); // Para colorear por altura
}

void PartitionRenderer::computeZRange() {
    if (points.empty()) return;
    minZ = maxZ = points[0].z;
    for (const auto& p : points) {
        minZ = std::min(minZ, p.z);
        maxZ = std::max(maxZ, p.z);
    }
}

void PartitionRenderer::render(bool wireframe) {
    if (!tree) {
        std::cout << "[DEBUG] Árbol aún no cargado en renderer.\n";
    } else {
        std::cout << "[DEBUG] Árbol listo, renderizando...\n";
    }

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    if (flyMode) {
        float radYaw = yaw * M_PI / 180.0f;
        float radPitch = pitch * M_PI / 180.0f;

        float eyeX = centerX + cameraDistance * cos(radPitch) * sin(radYaw);
        float eyeY = centerY + cameraDistance * sin(radPitch);
        float eyeZ = centerZ + cameraDistance * cos(radPitch) * cos(radYaw);

        gluLookAt(eyeX, eyeY, eyeZ,
                centerX, centerY, centerZ,
                0.0f, 1.0f, 0.0f);

    }
     
    else {
        // Tu modo cámara orbital normal
        gluLookAt(
            cameraTargetX, cameraTargetY, cameraTargetZ + cameraDistance,
            cameraTargetX, cameraTargetY, cameraTargetZ,
            0.0f, 1.0f, 0.0f
        );
    }

    if (currentMode == RenderMode::Octree) {
        std::cout << "Render mode: Octree, wireframeMode = " << wireframeMode << std::endl;
        std::cout << "[RENDER] tree = " << tree.get()
          << ", root = " << (tree ? tree->root.get() : nullptr) << std::endl;
          if (!tree) {
            std::cout << "[FAIL] tree == nullptr" << std::endl;
            return;
        }
        
        if (!tree->root) {
            std::cout << "[FAIL] tree->root == nullptr" << std::endl;
            return;
        }
        std::cout << "[RENDER] tree = " << tree.get() << ", root = " << tree->root.get() << std::endl;
    
        if (wireframeMode) {
            // Wireframe → recorrer todos los nodos ocupados (incluye padres con hijos)
            tree->traverse([=](bool isLeaf, float cx, float cy, float cz, float size, const std::vector<Point3D*>& pts) {
                std::cout << "[WIRE] Nodo: center=(" << cx << "," << cy << "," << cz << "), size=" << size << ", puntos=" << pts.size() << ", hoja=" << isLeaf << std::endl;
                if (pts.empty()) return;
    
                float dz = maxZ - minZ;
                float colorZ = (dz > 0.0f) ? (cz - minZ) / dz : 0.5f;
                glColor3f(1.0f, 1.0f, 1.0f);  // wireframe en blanco neutro

                std::cout << "Wireframe cube at: (" << cx << ", " << cy << ", " << cz << "), size: " << size << ", pts: " << pts.size() << std::endl;

                glPushMatrix();
                glTranslatef(cx, cy, cz);
                glutWireCube(size);
                glPopMatrix();
            });
        } else {
            // Render sólido → solo hojas ocupadas
            tree->traverseLeavesUpToDepth(renderDepth, [=](float cx, float cy, float cz, float size, const std::vector<Point3D*>& pts) {
                if (pts.empty()) return;
    
                float dz = maxZ - minZ;
                float colorZ = (dz > 0.0f) ? (cz - minZ) / dz : 0.5f;
                glColor3f(1.0f - colorZ, 0.0f, colorZ);  // color por altura   
                
                glPushMatrix();
                glTranslatef(cx, cy, cz);
                glutSolidCube(size);
                glPopMatrix();
            });
        }
        
    }
    else if (currentMode == RenderMode::KdTree) {
        renderKdTreePartitioning(kdtree.getRoot(),0, renderDepthKD);
    }
    else if (currentMode == RenderMode::BSP) {
        glPointSize(3.0f);
        drawBSPRecursive(bspTree.getRoot());
    }
}

void PartitionRenderer::diagnoseOctree() {
    if (!tree || !tree->root) {
        std::cout << "[ERROR] No hay árbol cargado.\n";
        return;
    }

    int totalLeaves = 0;
    int nonEmptyLeaves = 0;
    int maxDepth = 0;
    float minSize = 1e6, maxSize = -1;

    tree->root->traverseLeaves([&](float cx, float cy, float cz, float size, const std::vector<Point3D*>& leafPoints) {
        totalLeaves++;
        if (!leafPoints.empty()) nonEmptyLeaves++;
        if (size < minSize) minSize = size;
        if (size > maxSize) maxSize = size;
    });

    std::cout << "------ Diagnóstico Octree ------\n";
    std::cout << "Total de hojas:       " << totalLeaves << "\n";
    std::cout << "Hojas con puntos:     " << nonEmptyLeaves << "\n";
    std::cout << "Tamaño mínimo hoja:   " << minSize << "\n";
    std::cout << "Tamaño máximo hoja:   " << maxSize << "\n";
    std::cout << "Resolución pedida:    " << tree->maxDepth << "\n";
    std::cout << "--------------------------------\n";
}

void PartitionRenderer::increaseResolution() {
    if (currentMode == RenderMode::Octree) {
        if (tree && renderDepth < tree->maxDepth) {
            renderDepth++;
            std::cout << "[RES] Nueva resolución Octree: " << renderDepth << "\n";
        }
    } else if (currentMode == RenderMode::KdTree) {
        if (renderDepthKD < maxRenderDepth) {
            renderDepthKD++;
            std::cout << "[RES] Nueva profundidad KdTree: " << renderDepthKD << "\n";
        }
    }
}

void PartitionRenderer::decreaseResolution() {
    if (currentMode == RenderMode::Octree) {
        if (renderDepth > 1) {
            renderDepth--;
            std::cout << "[RES] Resolución reducida: " << renderDepth << "\n";
        }
    } else if (currentMode == RenderMode::KdTree) {
        if (renderDepthKD > 1) {
            renderDepthKD--;
            std::cout << "[RES] Nueva profundidad KdTree: " << renderDepthKD << "\n";
        }
    }
}

void PartitionRenderer::renderKdTreePartitioning(KdNode* node, int depth, int max) {
    if (!node || depth > max)
        return;

    const auto& box = node->bbox;

    // 🧹 No renderizar cuboids invisibles
    if ((box.max.x - box.min.x) < 1e-3f ||
        (box.max.y - box.min.y) < 1e-3f ||
        (box.max.z - box.min.z) < 1e-3f) {
        return;
    }

    // 🎨 Colorear por altura Z (puedes cambiar por eje o profundidad)
    float t = (node->point->z - minZ) / (maxZ - minZ);
    glColor3f(t, 0.2f * (1 - t), 1.0f - t);

    // 📦 Dibujar cuboid como líneas (wireframe)
    glLineWidth(1.0f);
    glBegin(GL_LINES);
    float x[] = { box.min.x, box.max.x };
    float y[] = { box.min.y, box.max.y };
    float z[] = { box.min.z, box.max.z };

    for (int xi : {0, 1}) for (int yi : {0, 1}) {
        glVertex3f(x[xi], y[yi], z[0]);
        glVertex3f(x[xi], y[yi], z[1]);
    }
    for (int xi : {0, 1}) for (int zi : {0, 1}) {
        glVertex3f(x[xi], y[0], z[zi]);
        glVertex3f(x[xi], y[1], z[zi]);
    }
    for (int yi : {0, 1}) for (int zi : {0, 1}) {
        glVertex3f(x[0], y[yi], z[zi]);
        glVertex3f(x[1], y[yi], z[zi]);
    }
    glEnd();

    // 🔁 Recursión
    renderKdTreePartitioning(node->left, depth + 1,max);
    renderKdTreePartitioning(node->right, depth + 1, max);
}


void PartitionRenderer::handleKeyboard(unsigned char key) {
    if (key == '1') {
        setRenderMode(RenderMode::Octree);
        std::cout << "[MODE] Octree\n";
    }
    else if (key == '2') {
        setRenderMode(RenderMode::KdTree);
        std::cout << "[MODE] KdTree\n";
    }
    else if (key == '3') {
        setRenderMode(RenderMode::BSP);
        std::cout << "[MODE] BSP \n";
    }

    if (key == '+' && (currentMode == RenderMode::Octree || currentMode == RenderMode::KdTree)) {
        increaseResolution();
    }
    else if (key == '-' && (currentMode == RenderMode::Octree || currentMode == RenderMode::KdTree)) {
        decreaseResolution();
    }

    if (key == 'f') {
        flyMode = !flyMode;
        firstMouse = true;
        std::cout << "[FLY MODE] " << (flyMode ? "Activado" : "Desactivado") << "\n";
    }

    glutPostRedisplay();
}

void PartitionRenderer::setRenderMode(RenderMode mode) {
    currentMode = mode;
}

void PartitionRenderer::computeBoundingBox() {
    if (points.empty()) return;
    bbox.min = bbox.max = points[0];
    for (const auto& p : points) {
        bbox.min.x = std::min(bbox.min.x, p.x);
        bbox.min.y = std::min(bbox.min.y, p.y);
        bbox.min.z = std::min(bbox.min.z, p.z);
        bbox.max.x = std::max(bbox.max.x, p.x);
        bbox.max.y = std::max(bbox.max.y, p.y);
        bbox.max.z = std::max(bbox.max.z, p.z);
    }
}

void PartitionRenderer::renderBSPPartitioning(BSPNode* node,
    float xmin, float xmax,
    float ymin, float ymax,
    float zmin, float zmax,
    int depth) {

    if (!node) return;

    glLineWidth(1.0f);
    glColor3f(1.0f, 1.0f, 0.0f); // Amarillo para los planos

    glBegin(GL_LINES);
    const Plane& plane = node->dividingPlane;
    
    if (plane.normal.x == 1.0f) { // Plano perpendicular a X
        glVertex3f(plane.d, ymin, zmin);
        glVertex3f(plane.d, ymax, zmax);
    } else if (plane.normal.y == 1.0f) { // Plano perpendicular a Y
        glVertex3f(xmin, plane.d, zmin);
        glVertex3f(xmax, plane.d, zmax);
    } else { // Plano perpendicular a Z
        glVertex3f(xmin, ymin, plane.d);
        glVertex3f(xmax, ymax, plane.d);
    }
    glEnd();

    // Recursión en subespacios
    if (plane.normal.x == 1.0f) {
        renderBSPPartitioning(node->front, plane.d, xmax, ymin, ymax, zmin, zmax, depth + 1);
        renderBSPPartitioning(node->back, xmin, plane.d, ymin, ymax, zmin, zmax, depth + 1);
    } else if (plane.normal.y == 1.0f) {
        renderBSPPartitioning(node->front, xmin, xmax, plane.d, ymax, zmin, zmax, depth + 1);
        renderBSPPartitioning(node->back, xmin, xmax, ymin, plane.d, zmin, zmax, depth + 1);
    } else {
        renderBSPPartitioning(node->front, xmin, xmax, ymin, ymax, plane.d, zmax, depth + 1);
        renderBSPPartitioning(node->back, xmin, xmax, ymin, ymax, zmin, plane.d, depth + 1);
    }
}

PartitionRenderer::Color PartitionRenderer::getRandomColor() {
    static std::vector<Color> palette = {
        {1.0f, 0.0f, 0.0f},  // Rojo
        {0.0f, 1.0f, 0.0f},  // Verde
        {0.0f, 0.0f, 1.0f},  // Azul
        {1.0f, 1.0f, 0.0f},  // Amarillo
        {1.0f, 0.0f, 1.0f},  // Magenta
        {0.0f, 1.0f, 1.0f},  // Cian
        {1.0f, 0.5f, 0.0f},  // Naranja
        {0.5f, 0.0f, 1.0f},  // Violeta
        {0.0f, 0.5f, 1.0f},  // Azul claro
        {0.5f, 1.0f, 0.0f}   // Lima
    };

    int index = rand() % palette.size();  // Elegir un color de la paleta
    return palette[index];
}


void PartitionRenderer::drawPoint(float x, float y, float z, const Color& color) {
    glColor3f(color.r, color.g, color.b);
    glVertex3f(x, y, z);
}

void PartitionRenderer::drawBSPRecursive(BSPNode* node) {
    if (!node) return;

    if (!node->front && !node->back) {
        Color color = getRandomColor();

        glBegin(GL_POINTS);
        for (auto* pt : node->points) {
            drawPoint(pt->x, pt->y, pt->z, color);
        }
        glEnd();
    } else {
        drawBSPRecursive(node->front);
        drawBSPRecursive(node->back);
    }
}

template <typename Func>
void measureExecutionTime(const std::string& label, Func functionToMeasure) {
    auto start = std::chrono::high_resolution_clock::now();

    functionToMeasure(); // Ejecuta lo que quieras medir

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;

    std::cout << "[TIMER] " << label << " tomó " << elapsed.count() << " segundos.\n";
}

void PartitionRenderer::setCameraTarget(float x, float y, float z) {
    cameraTargetX = x;
    cameraTargetY = y;
    cameraTargetZ = z;
}

void PartitionRenderer::setCameraDistance(float d) {
    cameraDistance = d;
}

void PartitionRenderer::setWireframeMode(bool enabled) {
    wireframeMode = enabled;
}

void PartitionRenderer::mouseMotionCallback(int x, int y) {
    if (!flyMode) return;

    std::cout << "mouse moved: " << x << ", " << y << "\n";

    if (firstMouse) {
        lastMouseX = x;
        lastMouseY = y;
        firstMouse = false;
        return;
    }

    float xoffset = x - lastMouseX;
    float yoffset = lastMouseY - y; // Invertido porque Y crece hacia abajo
    lastMouseX = x;
    lastMouseY = y;

    xoffset *= sensitivity;
    yoffset *= sensitivity;

    if (fabs(xoffset) < 0.01f && fabs(yoffset) < 0.01f) return;

    yaw += xoffset;
    pitch += yoffset;

    if (pitch > 89.0f) pitch = 89.0f;
    if (pitch < -89.0f) pitch = -89.0f;

    float radYaw = yaw * M_PI / 180.0f;
    float radPitch = pitch * M_PI / 180.0f;

    camDirX = cos(radYaw) * cos(radPitch);
    camDirY = sin(radPitch);
    camDirZ = sin(radYaw) * cos(radPitch);

    std::cout << "yaw: " << yaw << " pitch: " << pitch << "\n";
    std::cout << "dir: " << camDirX << ", " << camDirY << ", " << camDirZ << "\n";


    float len = sqrt(camDirX * camDirX + camDirY * camDirY + camDirZ * camDirZ);
    camDirX /= len;
    camDirY /= len;
    camDirZ /= len;

    glutPostRedisplay();
}

void PartitionRenderer::specialCallback(int key, int x, int y) {
    if (!flyMode) return;

    float angleStep = 2.5f;  // grados por pulsación

    switch (key) {
        case GLUT_KEY_LEFT:
            yaw -= angleStep; // rotar a la izquierda
            break;
        case GLUT_KEY_RIGHT:
            yaw += angleStep; // rotar a la derecha
            break;
        case GLUT_KEY_UP:
            pitch += angleStep; // mirar hacia arriba
            if (pitch > 89.0f) pitch = 89.0f;
            break;
        case GLUT_KEY_DOWN:
            pitch -= angleStep; // mirar hacia abajo
            if (pitch < -89.0f) pitch = -89.0f;
            break;
    }

    updateCameraDirection();
    glutPostRedisplay();
}

void PartitionRenderer::updateCameraDirection() {
    float radYaw = yaw * M_PI / 180.0f;
    float radPitch = pitch * M_PI / 180.0f;

    camDirX = cos(radYaw) * cos(radPitch);
    camDirY = sin(radPitch);
    camDirZ = sin(radYaw) * cos(radPitch);

    float len = sqrt(camDirX * camDirX + camDirY * camDirY + camDirZ * camDirZ);
    if (len != 0.0f) {
        camDirX /= len;
        camDirY /= len;
        camDirZ /= len;
    }
}