
#include <GL/glut.h>   // glutWireCube, glTranslatef,


using namespace brandonpelfrey;

void drawOctree(const Octree& node, int depth) {
    // 1) Dibuja el cubo del nodo actual
    Vec3 c = node.getOrigin();
    Vec3 h = node.getHalfDimension();
    glColor3f(0.0f, 1.0f, 0.0f);
    glPushMatrix();
      glTranslatef(c.x, c.y, c.z);
      // Escala a tamaño real del nodo
      glScalef(h.x * 2, h.y * 2, h.z * 2);
      glutWireCube(1.0f);
    glPopMatrix();

    // 2) Siempre recurre a los hijos si depth != 0
    if (depth != 0) {
        int nextDepth = (depth > 0 ? depth - 1 : -1);
        for (int i = 0; i < 8; ++i) {
            Octree* child = node.getChild(i);
            if (child) {
                drawOctree(*child, nextDepth);
            }
        }
    }
}
