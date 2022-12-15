#include "pink_physics_colliders.hpp"

void ps::pp::sphereToPlane(Rigidbody* _sphere, Rigidbody* _plane, ps::pp::Manifold* m) {
    auto plane = _plane->M(((ps::pp::Plane*)_plane->shape)->plane).normalized();
    auto sphere = (ps::pp::Sphere*)_sphere->shape;
    
    auto center = _sphere->M(sphere->center);

    auto line = plane | center;

    auto distance = abs((center.normalized() & plane).scalar());
    if (distance < sphere->radius) {
        m->count = 1;
        m->pointsOfContact[0] = line ^ plane;
        m->normal = line.normalized();
        m->penetration = sphere->radius - distance;
    }
    else {
        m->count = 0;
    }
}

void ps::pp::cubeToPlane(Rigidbody* box, Rigidbody* plane, Manifold* m) {

    
}