#include "PartitionRenderer.h"
#include <GL/glut.h>
#include <algorithm>
#include <iostream>


PartitionRenderer::PartitionRenderer() {}

void PartitionRenderer::loadPointCloud(const std::vector<Point3D>& pts) {
    points = pts;
    computeZRange();
}

void PartitionRenderer::setResolution(int depth) {
    if (points.empty()) return;

    float minX = points[0].x, maxX = points[0].x;
    float minY = points[0].y, maxY = points[0].y;
    float minZ = points[0].z, maxZ = points[0].z;

    for (const auto& p : points) {
        if (p.x < minX) minX = p.x;
        if (p.x > maxX) maxX = p.x;
        if (p.y < minY) minY = p.y;
        if (p.y > maxY) maxY = p.y;
        if (p.z < minZ) minZ = p.z;
        if (p.z > maxZ) maxZ = p.z;
    }

    centerX = (minX + maxX) / 2.0f;
    centerY = (minY + maxY) / 2.0f;
    centerZ = (minZ + maxZ) / 2.0f;

    float dx = maxX - minX;
    float dy = maxY - minY;
    float dz = maxZ - minZ;
    sceneSize = std::max({dx, dy, dz}) * 1.1f;

    std::cout << "[INFO] Bounding Box: [" << dx << " x " << dy << " x " << dz << "]\n";
    std::cout << "[INFO] Centro: (" << centerX << ", " << centerY << ", " << centerZ << ") Tamaño: " << sceneSize << "\n";

    int maxTreeDepth = 10;  // ⬅️ construimos una vez a profundidad 10
    tree = std::make_unique<Octree>(centerX, centerY, centerZ, sceneSize, maxTreeDepth);
    tree->build(points);

    renderDepth = 4;  // ⬅️ empezamos renderizando a nivel 4
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
    if (!tree || !tree->root) return;

    
   
    // 🔹 Primero, dibujamos los puntos originales como GL_POINTS
    glPointSize(1.0f); // puedes aumentar si querés más visibles
    glBegin(GL_POINTS);
    glColor3f(0, 1, 1); // cian
    for (const auto& p : points) {
        glVertex3f(p.x, p.y, p.z);
    }
    glEnd();
    if (currentMode == RenderMode::KdTree) {
        renderKdTreePartitioning(kdtree.getRoot(),
                                 bbox.min.x, bbox.max.x,
                                 bbox.min.y, bbox.max.y,
                                 bbox.min.z, bbox.max.z,
                                0);
    }
    else{
    // 🔹 Después, dibujamos los cubos del Octree
    tree->traverseLeavesUpToDepth(renderDepth, [=](float cx, float cy, float cz, float size, const std::vector<Point3D*>& pts) {
        if (pts.empty()) return;
        float colorZ = (cz - minZ) / (maxZ - minZ);
        glColor3f(1.0f - colorZ, 0.0f, colorZ);
    
        glPushMatrix();
        glTranslatef(cx, cy, cz);
        wireframe ? glutWireCube(size) : glutSolidCube(size);
        glPopMatrix();
    });}
    
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
if (!node || !node->point || depth > maxRenderDepth) return;

glLineWidth(1.5f);
glBegin(GL_LINES);

// Color por eje: X=Rojo, Y=Verde, Z=Azul
if (node->axis == 0) {
glColor3f(1.0f, 0.0f, 0.0f); // Rojo (X)
glVertex3f(node->point->x, ymin, zmin);
glVertex3f(node->point->x, ymax, zmax);
} else if (node->axis == 1) {
glColor3f(0.0f, 1.0f, 0.0f); // Verde (Y)
glVertex3f(xmin, node->point->y, zmin);
glVertex3f(xmax, node->point->y, zmax);
} else {
glColor3f(0.0f, 0.0f, 1.0f); // Azul (Z)
glVertex3f(xmin, ymin, node->point->z);
glVertex3f(xmax, ymax, node->point->z);
}

glEnd();

// Recursión adaptativa
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
        std::cout << "[MODE] BSP (no implementado aún)\n";
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

void PartitionRenderer::buildKdTree() {
    kdtree.buildFromPoints(points); // Asegurate de tener este método en tu clase KdTree
}

void PartitionRenderer::setPoints(const std::vector<Point3D>& pts) {
    points = pts;
}



