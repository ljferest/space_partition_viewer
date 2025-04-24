#include "OctreeRenderer.h"
#include <GL/glut.h>
#include <algorithm>
#include <iostream>

OctreeRenderer::OctreeRenderer() {}

void OctreeRenderer::loadPointCloud(const std::vector<Point3D>& pts) {
    points = pts;
    computeZRange();
}

void OctreeRenderer::setResolution(int depth) {
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

    // Guardamos el centro y el tamaño de la escena para usar en la cámara
    centerX = (minX + maxX) / 2.0f;
    centerY = (minY + maxY) / 2.0f;
    centerZ = (minZ + maxZ) / 2.0f;

    float dx = maxX - minX;
    float dy = maxY - minY;
    float dz = maxZ - minZ;
    sceneSize = std::max({dx, dy, dz}) * 1.1f; // un margen extra

    std::cout << "[INFO] Bounding Box: [" << dx << " x " << dy << " x " << dz << "]\n";
    std::cout << "[INFO] Centro: (" << centerX << ", " << centerY << ", " << centerZ << ") Tamaño: " << sceneSize << "\n";

    tree = std::make_unique<Octree>(centerX, centerY, centerZ, sceneSize, depth);
    tree->build(points);
}



void OctreeRenderer::computeZRange() {
    if (points.empty()) return;
    minZ = maxZ = points[0].z;
    for (const auto& p : points) {
        minZ = std::min(minZ, p.z);
        maxZ = std::max(maxZ, p.z);
    }
}

void OctreeRenderer::render(bool wireframe) {
    if (!tree || !tree->root) return;

    // 🔹 Primero, dibujamos los puntos originales como GL_POINTS
    glPointSize(1.0f); // puedes aumentar si querés más visibles
    glBegin(GL_POINTS);
    glColor3f(0, 1, 1); // cian
    for (const auto& p : points) {
        glVertex3f(p.x, p.y, p.z);
    }
    glEnd();

    // 🔹 Después, dibujamos los cubos del Octree
    tree->traverseLeavesUpToDepth(renderDepth, [=](float cx, float cy, float cz, float size, const std::vector<Point3D*>& pts) {
        if (pts.empty()) return;
        float colorZ = (cz - minZ) / (maxZ - minZ);
        glColor3f(1.0f - colorZ, 0.0f, colorZ);
    
        glPushMatrix();
        glTranslatef(cx, cy, cz);
        wireframe ? glutWireCube(size) : glutSolidCube(size);
        glPopMatrix();
    });
    
}


void OctreeRenderer::diagnoseOctree() {
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

void OctreeRenderer::increaseResolution() {
    if (renderDepth < tree->maxDepth) renderDepth++;
}
void OctreeRenderer::decreaseResolution() {
    if (renderDepth > 1) renderDepth--;
}



