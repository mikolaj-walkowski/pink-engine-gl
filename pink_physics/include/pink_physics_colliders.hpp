#pragma once
#include <vector>
#include "pink_structs.hpp"

namespace ps::pp {
    
    typedef void CollisionFunction(Rigidbody*, Rigidbody*, Manifold*);



    //TODO box v box
    void sphereToSphere(Rigidbody* b1, Rigidbody* b2, Manifold* m);

    //TODO cylinder v plane
    

    //TODO cylinder v cylinder

    //TODO box v cylinder

    //TODO any v system
    //TODO system v system

    void sphereToSphere(Rigidbody* sp1, Rigidbody* sp2, Manifold* m);
    void sphereToPlane(Rigidbody* sphere, Rigidbody* plane, Manifold* m);
    void boxToPlane(Rigidbody* box, Rigidbody* plane, Manifold* m);
    bool eCmp(float, float );
}//namespace ps::pp