#include "PartitionRenderer.h"
#include <GL/glut.h>
#include <algorithm>
#include <iostream>
#include <chrono>
#include <cmath>
#include <cstdlib>  // para std::srand, std::rand

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

    int maxTreeDepth = std::min(15, static_cast<int>(std::ceil(std::log2(pointPtrs.size()))));
    renderDepth = maxTreeDepth/2;
    int maxPointsPerLeaf = 10;


    measureExecutionTime("Construcción Octree", [this, maxTreeDepth, maxPointsPerLeaf]() {
        tree = std::make_unique<Octree>(centerX, centerY, centerZ, sceneSize, maxTreeDepth, maxPointsPerLeaf);
        tree->build(pointPtrs);
        std::cout << "[DEBUG] Diagnosing Octree instance: " << tree.get() << "\n";
        tree->diagnose(maxRenderTreeDepth);
    });

    measureExecutionTime("Construcción KD-Tree", [this]() {
        kdtree = std::make_unique<KdTree>();
        kdtree->build(pointPtrs);
        kdtree->diagnose(maxRenderDepth); //Show stats od kdtree
        renderDepthKD = maxRenderDepth/2;
    });

    measureExecutionTime("Construcción BSP-Tree", [this]() {
        bspTree = std::make_unique<BSPTree>();
        bspTree->build(pointPtrs, 10);

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
    
    /*
    if (!tree) {
        std::cout << "[DEBUG] Árbol aún no cargado en renderer.\n";
    } else {
        std::cout << "[DEBUG] Árbol listo, renderizando...\n";
    }
    */

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
        renderOctreePartitioning(wireframeMode);
    }
    else if (currentMode == RenderMode::KdTree) {
        renderKdTreePartitioning(kdtree->getRoot(),0, renderDepthKD);
    }
    else if (currentMode == RenderMode::BSP) {
        glPointSize(3.0f);
        drawBSPRecursive(bspTree->getRoot());
    }
}

void PartitionRenderer::increaseResolution() {
    if (currentMode == RenderMode::Octree) {
        if (renderDepth < maxRenderTreeDepth){
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

void PartitionRenderer::renderOctreePartitioning(bool wireframe) {
    if (!tree || !tree->root) return;

    if (!tree) {
        //std::cout << "[FAIL] tree == nullptr" << std::endl;
        return;
    }
    
    if (!tree->root) {
        //std::cout << "[FAIL] tree->root == nullptr" << std::endl;
        return;
    }
    //std::cout << "[RENDER] tree = " << tree.get() << ", root = " << tree->root.get() << std::endl;

    if (wireframeMode) {
        // 🔲 Mostrar contornos (todos los nodos)
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glLineWidth(1.0f);
        tree->traverse([&](bool isLeaf, float cx, float cy, float cz, float size, const std::vector<Point3D*>& pts) {
            glColor3f(1.0f, 1.0f, 1.0f);
            glPushMatrix();
            glTranslatef(cx, cy, cz);
            glutWireCube(size);
            glPopMatrix();
        });
    
        // 🟥 Mostrar hojas con puntos (sólido y color)
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        tree->traverseLeavesUpToDepth(renderDepth, [&](float cx, float cy, float cz, float size, const std::vector<Point3D*>& pts) {
            /*std::cout << "[TRACE] Nodo recibido: center=(" << cx << ", " << cy << ", " << cz 
                      << "), size=" << size << ", puntos=" << pts.size() << std::endl;
        
            
        
            std::cout << "  >> DIBUJANDO este nodo\n";
            */
            float dz = maxZ - minZ;
            float colorZ = (dz > 0.0f) ? (cz - minZ) / dz : 0.5f;
            glColor3f(1.0f - colorZ, 0.0f, colorZ);
        
            glPushMatrix();
            glTranslatef(cx, cy, cz);
            glutSolidCube(size);
            glPopMatrix();
        });            
    }
    else{
        // Render sólido → solo hojas ocupadas
        tree->traverseLeavesUpToDepth(renderDepth, [=](float cx, float cy, float cz, float size, const std::vector<Point3D*>& pts) {

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

void PartitionRenderer::renderKdTreePartitioning(KdNode* node, int depth, int max) {
    if (!node || depth > max) return;

    bool reachedLimit = (depth == max);
    bool isLeaf = !node->left && !node->right;

    // ❌ Si el subárbol no contiene puntos, no renderizar
    int numPts = countPointsInSubtree(node);
    if (numPts == 0) return;

    // 🔁 Si no es hoja y no llegó al límite de profundidad, seguir bajando
    if (!isLeaf && !reachedLimit) {
        renderKdTreePartitioning(node->left, depth + 1, max);
        renderKdTreePartitioning(node->right, depth + 1, max);
        return;
    }

    const auto& box = node->bbox;

    // Evitar cajas planas o invisibles
    if ((box.max.x - box.min.x) < 1e-3f ||
        (box.max.y - box.min.y) < 1e-3f ||
        (box.max.z - box.min.z) < 1e-3f) return;

    // 🎨 Color por altura Z
    float t = (node->point->z - minZ) / (maxZ - minZ + 1e-6f);
    glColor3f(1.0f - t, 0.0f, t);

    // 📦 Dibujo de cubo opaco
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glBegin(GL_QUADS);

    // Cara inferior (Z-)
    glVertex3f(box.min.x, box.min.y, box.min.z);
    glVertex3f(box.max.x, box.min.y, box.min.z);
    glVertex3f(box.max.x, box.max.y, box.min.z);
    glVertex3f(box.min.x, box.max.y, box.min.z);

    // Cara superior (Z+)
    glVertex3f(box.min.x, box.min.y, box.max.z);
    glVertex3f(box.max.x, box.min.y, box.max.z);
    glVertex3f(box.max.x, box.max.y, box.max.z);
    glVertex3f(box.min.x, box.max.y, box.max.z);

    // Cara frontal (Y+)
    glVertex3f(box.min.x, box.max.y, box.min.z);
    glVertex3f(box.max.x, box.max.y, box.min.z);
    glVertex3f(box.max.x, box.max.y, box.max.z);
    glVertex3f(box.min.x, box.max.y, box.max.z);

    // Cara trasera (Y-)
    glVertex3f(box.min.x, box.min.y, box.min.z);
    glVertex3f(box.max.x, box.min.y, box.min.z);
    glVertex3f(box.max.x, box.min.y, box.max.z);
    glVertex3f(box.min.x, box.min.y, box.max.z);

    // Cara izquierda (X-)
    glVertex3f(box.min.x, box.min.y, box.min.z);
    glVertex3f(box.min.x, box.max.y, box.min.z);
    glVertex3f(box.min.x, box.max.y, box.max.z);
    glVertex3f(box.min.x, box.min.y, box.max.z);

    // Cara derecha (X+)
    glVertex3f(box.max.x, box.min.y, box.min.z);
    glVertex3f(box.max.x, box.max.y, box.min.z);
    glVertex3f(box.max.x, box.max.y, box.max.z);
    glVertex3f(box.max.x, box.min.y, box.max.z);

    glEnd();
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
        float r, g, b;
        getColorFromId(node->nodeId, r, g, b);
        Color color = {r,g,b};
        /*
        std::cout << "Node ID: " << node->nodeId 
          << " | Color RGB: (" << r << ", " << g << ", " << b << ")" << std::endl;
        */
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

    //std::cout << "mouse moved: " << x << ", " << y << "\n";

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

    /*
    std::cout << "yaw: " << yaw << " pitch: " << pitch << "\n";
    std::cout << "dir: " << camDirX << ", " << camDirY << ", " << camDirZ << "\n";
    */

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

void PartitionRenderer::getColorFromId(int id, float& r, float& g, float& b) {
    unsigned int hash = static_cast<unsigned int>(id) * 2654435761u;

    r = ((hash >> 16) & 0xFF) / 255.0f;
    g = ((hash >> 8) & 0xFF) / 255.0f;
    b = (hash & 0xFF) / 255.0f;

    // Evita colores muy oscuros
    if (r + g + b < 0.3f) {
        r += 0.3f;
        g += 0.3f;
    }

    // Recorta si supera 1.0
    r = std::min(r, 1.0f);
    g = std::min(g, 1.0f);
    b = std::min(b, 1.0f);
}

int PartitionRenderer::countPointsInSubtree(KdNode* node) {
    if (!node) return 0;
    if (!node->left && !node->right && node->point) return 1;
    return countPointsInSubtree(node->left) + countPointsInSubtree(node->right);
}