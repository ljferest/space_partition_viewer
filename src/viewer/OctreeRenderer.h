#pragma once

#include "octree/Octree.h"
#include "octree/Vec3.h"
#include "octree/OctreePoint.h"

using namespace brandonpelfrey;

// Dibuja un Octree hasta el nivel 'depth'.
// depth<0 = dibuja todo; depth=0 = solo la raíz.
void drawOctree(const Octree& node, int depth);
