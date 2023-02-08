#pragma once
#include <vector>
#include "pink_structs.hpp"
#include "pink_physics_shapes.hpp"
namespace ps::pp {

    struct Interval {
        float min;
        float max;
    };

    Interval GetInterval(const kln::point* verts, const vec3& axis);
    bool OverlapOnAxis(ps::pp::Rigidbody* rb1, ps::pp::Rigidbody* rb2, const vec3& axis);
    void kln_calcBoxOrientation(ps::pp::Rigidbody* rb, kln::line* orientation);
    bool PointInBox(kln::point& inp_point, ps::pp::Rigidbody* rb);
    bool ClipToPlane(const kln::plane& plane, std::pair<kln::point, kln::point>& line, kln::point* outPoint);
    std::vector<kln::point> ClipEdgesToOBB(std::pair<kln::point, kln::point>* edges, ps::pp::Rigidbody* rb);
    float PenetrationDepth(ps::pp::Rigidbody* rb1, ps::pp::Rigidbody* rb2, const vec3& axis, bool* outShouldFlip = nullptr);

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