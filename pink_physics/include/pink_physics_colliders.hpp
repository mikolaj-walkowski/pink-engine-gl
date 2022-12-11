#include <vector>
#include "pink_structs.hpp"

namespace ps::pp {
    
    typedef void CollisionFunction(void*, void*, Manifold*);

    // CollisionFunction sphereToSphere;
    // CollisionFunction capsuleToCapsule;
    // CollisionFunction cylinderToCylinder;

    // CollisionFunction cylinderToHeightMap;
    // CollisionFunction capsuleToHeightMap;

    void sphereToPlane(void* sphere, void* plane, Manifold* m);
}//namespace ps::pp