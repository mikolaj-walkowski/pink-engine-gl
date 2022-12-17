#include "pink_physics_colliders.hpp"

bool eCmp(float a, float b) {
    float epsilon = 0.000001f;
    return abs(a - b) < epsilon;
}

void ps::pp::sphereToPlane(Rigidbody* _sphere, Rigidbody* _plane, ps::pp::Manifold* m) {
    auto plane = _plane->M(((ps::pp::Plane*)_plane->shape)->plane).normalized();
    auto sphere = (ps::pp::Sphere*)_sphere->shape;
    m->count = 0;

    auto center = _sphere->M(sphere->center);

    auto line = plane | center;

    auto distance = abs((center.normalized() & plane).scalar());
    if (distance < sphere->radius) {
        m->count = 1;
        m->pointsOfContact[0] = line ^ plane;
        m->normal = line.normalized();
        m->penetration = sphere->radius - distance;
    }
}

void* find(void* arr, void* arr_end, void* data, int size) {
    for (void* i = arr; i < arr_end; i = static_cast<char*>(i) + size) {
        if (memcmp(i, data, size) == 0)
            return i;
    }
    return NULL;
}

float sumCoords(kln::point a) {
    return a.e013() + a.e021() + a.e032();
}

bool between(float max, float min, float a) {
    return min <= a && a <= max;
}

// Points of contact already in box space
void ps::pp::boxToPlane(Rigidbody* _box, Rigidbody* _plane, Manifold* m) {
    auto box = (ps::pp::Box*)(_box->shape);
    auto plane = _plane->M(((ps::pp::Plane*)(_plane->shape))->plane);

    m->count = 0;

    for (auto p : box->edges) {
        auto i = _box->M(box->verts[p.first]);
        auto j = _box->M(box->verts[p.second]);

        auto line = i & j;
        //line.normalize();

        m->normal = plane | kln::origin();
        m->normal.normalize();

        auto point = line ^ plane;
        point.normalize();
        //point.e013() == 0 && point.e021() == 0 && point.e032() == 0 && 
        // printf("Point: %f,%f,%f,%f\n", point.e013(), point.e021(), point.e032(), point.e123());

        //Line parallel
        if (point.e123() == 0.f) {
            if (eCmp(point.e013(), 0) && eCmp(point.e021(), 0) && eCmp(point.e032(), 0)) {
                kln::point* p = (kln::point*)find(m->pointsOfContact, m->pointsOfContact + m->count, &i, sizeof(kln::point));
                if (p == NULL && m->count < m->maxContactPoints) {
                    m->pointsOfContact[m->count] = i;
                    ++(m->count);
                }
                p = (kln::point*)find(m->pointsOfContact, m->pointsOfContact + m->count, &j, sizeof(kln::point));
                if (p == NULL && m->count < m->maxContactPoints) {
                    m->pointsOfContact[m->count] = j;
                    ++(m->count);
                }
            }
        }
        else {
            bool x = between(
                std::max(i.x(), j.x()),
                std::min(i.x(), j.x()),
                point.x()
            );
            bool y = between(
                std::max(i.y(), j.y()),
                std::min(i.y(), j.y()),
                point.y()
            );
            bool z = between(
                std::max(i.z(), j.z()),
                std::min(i.z(), j.z()),
                point.z()
            );

            // printf("%d, %d , %d\n", x, y, z);
            // printf("I: %f,%f,%f,%f\n", i.e013(), i.e021(), i.e032(), i.e123());
            // printf("J: %f,%f,%f,%f\n", j.e013(), j.e021(), j.e032(), j.e123());
            // printf("Point: %f,%f,%f,%f\n", point.e013(), point.e021(), point.e032(), point.e123());
            if (x && y && z) {
                kln::point* p = (kln::point*)find(m->pointsOfContact, m->pointsOfContact + m->count, &point, sizeof(kln::point));
                if (p == NULL && m->count < m->maxContactPoints) {
                    m->pointsOfContact[m->count] = point;
                    ++(m->count);
                }
            }

            // bool min = std::min(sumCoords(i), sumCoords(j));
            // bool max = std::max(sumCoords(i), sumCoords(j));
            // bool p = sumCoords(point);

            //  if (!between(max,min,p)) {
            //     kln::point* p = (kln::point*)find(m->pointsOfContact, m->pointsOfContact + m->count, &point, sizeof(kln::point));
            //     if (p == NULL && m->count < m->maxContactPoints) {
            //         m->pointsOfContact[m->count] = point;
            //         ++(m->count);
            //     }
            // }

        }

    }


}