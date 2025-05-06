#include "common/PointCloudLoader.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>

// This function loads a PCD file and returns a vector of Point3D objects.
std::vector<Point3D> loadFromPCD(const std::string& filename) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == -1) {
        std::cerr << "[ERROR] No se pudo cargar el archivo: " << filename << "\n";
        return {};
    }

    std::vector<Point3D> result;
    result.reserve(cloud->points.size());

    for (const auto& pt : cloud->points) {
        result.emplace_back(pt.x, pt.y, pt.z);
    }

    return result;
}
