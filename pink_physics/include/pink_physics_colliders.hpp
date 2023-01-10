#pragma once
#include <vector>
#include "pink_structs.hpp"
#include "pink_physics_shapes.hpp"
namespace ps::pp {

    typedef bool (*CollisionFunction)(BaseShape*, kln::motor*, BaseShape*, kln::motor*, Manifold*);

    bool collide(Rigidbody*, Rigidbody*, Manifold*);

    bool defaultCollider(BaseShape*, kln::motor*, BaseShape*, kln::motor*, Manifold*);
    //TODO box v box
    bool sphereToSphere(BaseShape*, kln::motor*, BaseShape*, kln::motor*, Manifold*);
    bool boxToBox(BaseShape*, kln::motor*, BaseShape*, kln::motor*, Manifold*);

    //TODO cylinder v plane


    //TODO cylinder v cylinder

    //TODO box v cylinder

    // bool shapeToComposite(BaseShape*, kln::motor*, BaseShape*, kln::motor*, Manifold*);

    bool sphereToSphere(BaseShape*, kln::motor*, BaseShape*, kln::motor*, Manifold*);
    bool sphereToPlane(BaseShape*, kln::motor*, BaseShape*, kln::motor*, Manifold*);
    bool boxToPlane(BaseShape*, kln::motor*, BaseShape*, kln::motor*, Manifold*);

    //CollisionFunction jumpTable[BM(ST_SIZE)];

    bool eCmp(float, float);
}//namespace ps::pp