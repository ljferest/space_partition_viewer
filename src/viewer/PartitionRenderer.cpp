#include "PartitionRenderer.h"
#include <GL/glut.h>
#include <algorithm>
#include <iostream>
#include <chrono>


PartitionRenderer::PartitionRenderer() {}

void PartitionRenderer::loadPointCloud(const std::vector<Point3D>& pts) {
    points = pts;
    pointPtrs.clear();
    for (auto& p : points) {
        pointPtrs.push_back(&p);
    }

    computeBoundingBox();

    centerX = (bbox.min.x + bbox.max.x) / 2.0f;
    centerY = (bbox.min.y + bbox.max.y) / 2.0f;
    centerZ = (bbox.min.z + bbox.max.z) / 2.0f;
    // Centrar la cámara en el modelo
    cameraTargetX = centerX;
    cameraTargetY = centerY;
    cameraTargetZ = centerZ;
    cameraDistance = sceneSize * 1.5f;  // Alejado para ver todo


    float dx = bbox.max.x - bbox.min.x;
    float dy = bbox.max.y - bbox.min.y;
    float dz = bbox.max.z - bbox.min.z;
    sceneSize = std::max({ dx, dy, dz }) * 1.1f;

    std::cout << "[INFO] Bounding Box: [" << dx << " x " << dy << " x " << dz << "]\n";
    std::cout << "[INFO] Centro: (" << centerX << ", " << centerY << ", " << centerZ << ") Tamaño: " << sceneSize << "\n";

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
    });

    measureExecutionTime("Construcción BSP-Tree", [this]() {
        bspTree.build(pointPtrs, 10);
    });

    computeZRange(); // Opcional si quieres seguir normalizando colores
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

    glLoadIdentity();
    gluLookAt(
        cameraTargetX, cameraTargetY, cameraTargetZ + cameraDistance,
        cameraTargetX, cameraTargetY, cameraTargetZ,
        0.0f, 1.0f, 0.0f
    );

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

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
        renderKdTreePartitioning(kdtree.getRoot(),
                                 bbox.min.x, bbox.max.x,
                                 bbox.min.y, bbox.max.y,
                                 bbox.min.z, bbox.max.z,
                                 0);
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
    if (tree && renderDepth < tree->maxDepth) {
        renderDepth++;
        std::cout << "[RES] Nueva resolución: " << renderDepth << "\n";
    }
}

void PartitionRenderer::decreaseResolution() {
    if (renderDepth > 1) {
        renderDepth--;
        std::cout << "[RES] Resolución reducida: " << renderDepth << "\n";
    }
}

void PartitionRenderer::renderKdTreePartitioning(KdNode* node,
    float xmin, float xmax,
    float ymin, float ymax,
    float zmin, float zmax,
    int depth) {

    if (!node || !node->point || depth > maxRenderDepth)
        return;

    // Colorear por altura z
    float nodeZ = node->point->z;
    float t = (nodeZ - minZ) / (maxZ - minZ); // Normalizar
    float r = t;
    float g = 0.2f * (1 - t);
    float b = 1.0f - t;
    glColor3f(r, g, b);

    glLineWidth(1.0f);
    glBegin(GL_LINES);

    float x[] = { xmin, xmax };
    float y[] = { ymin, ymax };
    float z[] = { zmin, zmax };

    for (int xi : {0, 1}) {
        for (int yi : {0, 1}) {
            glVertex3f(x[xi], y[yi], z[0]);
            glVertex3f(x[xi], y[yi], z[1]);
        }
    }

    for (int xi : {0, 1}) {
        for (int zi : {0, 1}) {
            glVertex3f(x[xi], y[0], z[zi]);
            glVertex3f(x[xi], y[1], z[zi]);
        }
    }

    for (int yi : {0, 1}) {
        for (int zi : {0, 1}) {
            glVertex3f(x[0], y[yi], z[zi]);
            glVertex3f(x[1], y[yi], z[zi]);
        }
    }

    glEnd();

    // Recursión
    if (node->axis == 0) {
        renderKdTreePartitioning(node->left, xmin, node->point->x, ymin, ymax, zmin, zmax, depth + 1);
        renderKdTreePartitioning(node->right, node->point->x, xmax, ymin, ymax, zmin, zmax, depth + 1);
    } else if (node->axis == 1) {
        renderKdTreePartitioning(node->left, xmin, xmax, ymin, node->point->y, zmin, zmax, depth + 1);
        renderKdTreePartitioning(node->right, xmin, xmax, node->point->y, ymax, zmin, zmax, depth + 1);
    } else {
        renderKdTreePartitioning(node->left, xmin, xmax, ymin, ymax, zmin, node->point->z, depth + 1);
        renderKdTreePartitioning(node->right, xmin, xmax, ymin, ymax, node->point->z, zmax, depth + 1);
    }
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

    if (key == '+' && currentMode == RenderMode::Octree) {
        increaseResolution();
    }
    else if (key == '-' && currentMode == RenderMode::Octree) {
        decreaseResolution();
    }

    if (key == ']' && currentMode == RenderMode::KdTree) {
        maxRenderDepth++;
        std::cout << "[VISUAL] Profundidad KdTree + → " << maxRenderDepth << "\n";
    }
    else if (key == '[' && currentMode == RenderMode::KdTree && maxRenderDepth > 0) {
        maxRenderDepth--;
        std::cout << "[VISUAL] Profundidad KdTree - → " << maxRenderDepth << "\n";
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
