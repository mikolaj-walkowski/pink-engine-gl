#include "pink_physics_colliders.hpp"

void ps::pp::sphereToPlane(void* _sphere, void* _plane, ps::pp::Manifold* m) {
    auto plane = ((ps::pp::Plane*)_plane)->plane.normalized();
    auto sphere = (ps::pp::Sphere*)_sphere;
    
    auto center = sphere->center.normalized();

    auto line = plane | center;

    auto distance = (plane & center).norm();

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