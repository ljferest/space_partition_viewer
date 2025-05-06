#ifndef POINT3D_H
#define POINT3D_H

struct Point3D {
    float x, y, z;
    
    Point3D() : x(0), y(0), z(0) {}
    Point3D(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
};

#endif 
