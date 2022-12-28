#pragma once
#include <vector>
#include "pink_structs.hpp"

namespace ps::pp {
    
    typedef void CollisionFunction(Rigidbody*, Rigidbody*, Manifold*);

    // CollisionFunction sphereToSphere;
    // CollisionFunction capsuleToCapsule;
    // CollisionFunction cylinderToCylinder;

    // CollisionFunction cylinderToHeightMap;
    // CollisionFunction capsuleToHeightMap;
    
    void sphereToSphere(Rigidbody* sp1, Rigidbody* sp2, Manifold* m);
    void sphereToPlane(Rigidbody* sphere, Rigidbody* plane, Manifold* m);
    void boxToPlane(Rigidbody* box, Rigidbody* plane, Manifold* m);
    
}//namespace ps::pp