#include "common/PointCloudLoader.h"
#include "kdtree/KdTree.h"
#include "common/Point3D.h"
#include <iostream>

int main() {
    std::vector<Point3D> cloud = loadFromPCD("../data/hasselt.pcd");
    std::cout << "[INFO] Cargados: " << cloud.size() << " puntos\n";

    KdTree tree;
    std::vector<Point3D*> pointPtrs;
    for (auto& pt : cloud) {
        pointPtrs.push_back(&pt);
    }

    tree.build(pointPtrs); // Ahora correcto

    tree.diagnose();

    // 🧪 Paso 3: probar queryRegion
    float cx = 0.0f, cy = 0.0f, cz = 0.0f, size = 10.0f;
    std::vector<Point3D*> encontrados;

    tree.queryRegion(cx, cy, cz, size, encontrados);

    std::cout << "[INFO] Encontrados en región: " << encontrados.size() << " puntos\n";

    return 0;
}
