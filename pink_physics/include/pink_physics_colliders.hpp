#pragma once

#include <vector>
#include "pink_structs.hpp"
#include "glm/glm.hpp"

struct Manifold {
    int count;
    ps::pp::Sphere sphere;
    float penetration;
    std::vector<glm::vec2> pointsOfContact;
};

typedef void CollisionFunction(void*, void*, Manifold*) ;

CollisionFunction sphereToSphere;
CollisionFunction capsuleToCapsule;
CollisionFunction cylinderToCylinder;

CollisionFunction cylinderToHeightMap;
CollisionFunction capsuleToHeightMap;

