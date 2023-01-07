#include "pink_physics_colliders.hpp"

bool ps::pp::eCmp(float a, float b) {
    float epsilon = 0.0001f;
    return abs(a - b) < epsilon;
}

void ps::pp::sphereToPlane(Rigidbody* _sphere, Rigidbody* _plane, ps::pp::Manifold* m) {
    auto plane = _plane->M(((ps::pp::Plane*)_plane->shape)->plane);
    auto sphere = (ps::pp::Sphere*)_sphere->shape;
    m->count = 0;

    auto center = _sphere->M(sphere->center).normalized();

    auto line = plane | center;

    auto distance = abs((center & plane).scalar());
    if (distance < sphere->radius) {
        m->count = 1;
        m->pointsOfContact[0] = (line ^ plane);
        m->normal = plane ;
        m->penetration = sphere->radius - distance;
    }
}

void ps::pp::sphereToSphere(Rigidbody* sp1, Rigidbody* sp2, Manifold* m) {
    auto s1 = (ps::pp::Sphere*)sp1->shape;
    auto s2 = (ps::pp::Sphere*)sp2->shape;
    m->count = 0;

    auto c1 = sp1->M(s1->center).normalized();
    auto c2 = sp2->M(s2->center).normalized();
    auto normal = c1 & c2;
    if (normal.norm() <= s1->radius + s2->radius) {
        float b = s2->radius/(s1->radius + s2->radius);
        auto t = c1 * c2;
        t = kln::sqrt(t) * b;
        m->count = 1;
        m->pointsOfContact[0] = t(c2);
        auto plane = normal.normalized() | m->pointsOfContact[0];
        m->normal = plane ;
        m->penetration = s1->radius + s2->radius - normal.norm();
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
    float epsilon = 0.0001f;
    return min - epsilon <= a && a <= max + epsilon;
}

// Points of contact already in box space
void ps::pp::boxToPlane(Rigidbody* _box, Rigidbody* _plane, Manifold* m) {
    auto box = (ps::pp::Box*)(_box->shape);
    auto boxCenter = _box->M(_box->centerOfMass);
    auto plane = _plane->M(((ps::pp::Plane*)(_plane->shape))->plane);

    m->count = 0;
    m->normal = plane;
    //m->normal.normalize();

    for (auto p : box->edges) {
        auto i = _box->M(box->verts[p.first]);
        auto j = _box->M(box->verts[p.second]);

        auto line = i & j;
        line.normalize();

        auto point = line ^ plane;
        
        //point.e013() == 0 && point.e021() == 0 && point.e032() == 0 && 
        // printf("Point: %f,%f,%f,%f\n", point.e013(), point.e021(), point.e032(), point.e123());

        //Line parallel
        auto helper = (plane & i).scalar();
        if (point.e123() == 0.f) {
            if (ps::pp::eCmp((plane & i).scalar(), 0.0f)) {
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
            point.normalize();
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

void ps::pp::boxToBox(Rigidbody* ap_box1, Rigidbody* ap_box2, Manifold* m) {
    auto boxOne = (ps::pp::Box*)(ap_box1->shape);
    auto boxOneCenter = ap_box1->M(ap_box1->centerOfMass);

    auto boxTwo = (ps::pp::Box*)(ap_box2->shape);
    auto boxTwoCenter = ap_box2->M(ap_box2->centerOfMass);

    m->count = 0;

    for (auto p1 : boxOne->faces) {
        auto vert1 = ap_box1->M(boxOne->verts[p1[0]]);
        auto vert2 = ap_box1->M(boxOne->verts[p1[1]]);
        auto vert3 = ap_box1->M(boxOne->verts[p1[2]]);
        auto vert4 = ap_box1->M(boxOne->verts[p1[3]]);

        auto plane = vert1 & vert2 & vert3;
        plane.normalize();
        auto normal = plane;

        for (auto p2 : boxTwo->edges) {
            
            auto i = ap_box2->M(boxTwo->verts[p2.first]);
            auto j = ap_box2->M(boxTwo->verts[p2.second]);

            auto line = i & j;
            line.normalize();

            auto point = plane ^ line;

            point.normalize();

            // ugly as hell, but works
            bool x = between(
                std::max(i.x(), j.x()),
                std::min(i.x(), j.x()),
                point.x()
            ) & between(
                std::max(std::max(vert1.x(), vert2.x()), std::max(vert3.x(), vert4.x())),
                std::min(std::min(vert1.x(), vert2.x()), std::min(vert3.x(), vert4.x())),
                point.x()
            );

            bool y = between(
                std::max(i.y(), j.y()),
                std::min(i.y(), j.y()),
                point.y()
            ) & between(
                std::max(std::max(vert1.y(), vert2.y()), std::max(vert3.y(), vert4.y())),
                std::min(std::min(vert1.y(), vert2.y()), std::min(vert3.y(), vert4.y())),
                point.y()
            );

            bool z = between(
                std::max(i.z(), j.z()),
                std::min(i.z(), j.z()),
                point.z()
            ) & between(
                std::max(std::max(vert1.z(), vert2.z()), std::max(vert3.z(), vert4.z())),
                std::min(std::min(vert1.z(), vert2.z()), std::min(vert3.z(), vert4.z())),
                point.z()
            );
            
            if (x && y && z) {
                kln::point* p = (kln::point*)find(m->pointsOfContact, m->pointsOfContact + m->count, &point, sizeof(kln::point));
                if (p == NULL && m->count < m->maxContactPoints) {
                    m->normal = normal;
                    m->pointsOfContact[m->count] = point;
                    ++(m->count);
                }
            }

        }

    }


}