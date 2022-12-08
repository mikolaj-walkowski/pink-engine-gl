#include "pink_physics_colliders.hpp"

void ps::pp::sphereToPlane(void* _sphere, void* _plane, ps::pp::Manifold* m) {
    auto plane = (ps::pp::Plane*)_plane;
    auto sphere = (ps::pp::Sphere*)_sphere;
    auto distance = (plane->plane & sphere->center).scalar();

    if (distance < sphere->radius) {
        m->count = 1;
        m->pointsOfContact = { (plane->plane | sphere->center) ^ plane->plane };
        m->penetration = sphere->radius - distance;
    }
    else {
        m->count = 0;
    }
}