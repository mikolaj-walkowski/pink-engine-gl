#include "pink_physics_colliders.hpp"

bool ps::pp::eCmp(float a, float b) {
    float epsilon = 0.0001f;
    return abs(a - b) < epsilon;
}


// void initJmpTable() {
//     ps::pp::jumpTable[BM(ps::pp::ST_SPHERE)] = ps::pp::sphereToSphere;
//     ps::pp::jumpTable[BM(ps::pp::ST_PLANE) | BM(ps::pp::ST_SPHERE)] = ps::pp::sphereToPlane;
//     ps::pp::jumpTable[BM(ps::pp::ST_PLANE) | BM(ps::pp::ST_BOX)] = ps::pp::boxToPlane;
//     ps::pp::jumpTable[BM(ps::pp)]
// }
bool ps::pp::collide(ps::pp::Rigidbody* rb1, ps::pp::Rigidbody* rb2, ps::pp::Manifold* m) {
    if (rb1->shape->type > rb2->shape->type) {
        m->rb1 = rb2;
        m->rb2 = rb1;
    }
    else {
        m->rb1 = rb1;
        m->rb2 = rb2;
    }

    CollisionFunction f = defaultCollider;
    auto type = BM(m->rb1->shape->type) | BM(m->rb2->shape->type);
    // if (type & BM(ps::pp::ST_COMPOSITE)) {
    //     f = shapeToComposite;
    // }
    // else {
    switch (type)
    {
    case (BM(ps::pp::ST_SPHERE) | BM(ps::pp::ST_PLANE)): {
        f = sphereToPlane;
        break;
    }
    case (BM(ps::pp::ST_BOX) | BM(ps::pp::ST_PLANE)): {
        f = boxToPlane;
        break;
    }
    case (BM(ps::pp::ST_SPHERE)): {
        f = sphereToSphere;
        break;
    }
    case (BM(ps::pp::ST_BOX)): {
        f = boxToBox;
        break;
    }
    }

    // }
    bool out = f(m->rb1->shape, &m->rb1->M, m->rb2->shape, &m->rb2->M, m);
    return out;
}

bool ps::pp::defaultCollider(BaseShape* b1, kln::motor* m1, BaseShape* b2, kln::motor* m2, Manifold* m) {
    return false;
}

bool ps::pp::sphereToPlane(BaseShape* _plane, kln::motor* m1, BaseShape* _sphere, kln::motor* m2, ps::pp::Manifold* m) {
    auto plane = (*m1)(((ps::pp::Plane*)_plane)->plane);
    auto sphere = (ps::pp::Sphere*)_sphere;

    auto center = (*m2)(sphere->center).normalized();

    auto line = plane | center;

    auto distance = abs((center & plane).scalar());
    if (distance < sphere->radius) {
        m->count = 1;
        m->pointsOfContact[0] = (line ^ plane);
        m->normal = plane * -1.f;
        m->penetration = sphere->radius - distance;
        return true;
    }
    return false;
}

bool ps::pp::sphereToSphere(BaseShape* sp1, kln::motor* m1, BaseShape* sp2, kln::motor* m2, Manifold* m) {
    auto s1 = (ps::pp::Sphere*)sp1;
    auto s2 = (ps::pp::Sphere*)sp2;

    auto c1 = (*m1)(s1->center).normalized();
    auto c2 = (*m2)(s2->center).normalized();
    auto normal = c1 & c2;
    if (normal.norm() <= s1->radius + s2->radius) {
        float b = s2->radius / (s1->radius + s2->radius);
        auto t = c1 * c2;
        t = kln::sqrt(t) * b;
        m->count = 1;
        m->pointsOfContact[0] = t(c2);
        auto plane = normal.normalized() | m->pointsOfContact[0];
        m->normal = plane;
        m->penetration = s1->radius + s2->radius - normal.norm();
        return true;
    }
    return false;
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
bool ps::pp::boxToPlane(BaseShape* _plane, kln::motor* m1, BaseShape* _box, kln::motor* m2, Manifold* m) {
    auto box = (ps::pp::Box*)_box;
    auto boxCenter = (*m2)(m->rb2->centerOfMass);
    auto plane = (*m1)(((ps::pp::Plane*)_plane)->plane);
    bool out = false;
    m->normal = plane * -1;
    m->count = 0;
    //m->normal.normalize();

    for (auto p : box->edges) {
        auto i = (*m2)(box->verts[p.first]);
        auto j = (*m2)(box->verts[p.second]);

        auto line = i & j;
        line.normalize();

        auto point = line ^ plane;

        //point.e013() == 0 && point.e021() == 0 && point.e032() == 0 && 
        // printf("Point: %f,%f,%f,%f\n", point.e013(), point.e021(), point.e032(), point.e123());

        //Line parallel
        if (point.e123() == 0.f) {
            if (ps::pp::eCmp((plane & i).scalar(), 0.0f)) {
                kln::point* p = (kln::point*)find(m->pointsOfContact, m->pointsOfContact + m->count, &i, sizeof(kln::point));
                if (p == NULL && m->count < m->maxContactPoints) {
                    m->pointsOfContact[m->count] = i;
                    ++(m->count);
                    out = true;
                }
                p = (kln::point*)find(m->pointsOfContact, m->pointsOfContact + m->count, &j, sizeof(kln::point));
                if (p == NULL && m->count < m->maxContactPoints) {
                    m->pointsOfContact[m->count] = j;
                    ++(m->count);
                    out = true;

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
                    out = true;
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

    return out;
}

// bool ps::pp::shapeToComposite(BaseShape* sh, kln::motor* m1, BaseShape* com, kln::motor* m2, Manifold* m) {
//     auto c = (Composite*)com;
//     bool out = false;
//     for (int i = 0; i < c->children.size(); ++i) {
//         auto ch = c->children[i];
//         out |= collide(sh, m1, ch.first, &((*m2) * ch.second), m);
//     }
//     return out;
// }

bool ps::pp::boxToBox(BaseShape* ap_box1_, kln::motor* m1, BaseShape* ap_box2_, kln::motor* m2, Manifold* m) {
    auto ap_box1 = m->rb1;
    auto ap_box2 = m->rb2;
    bool out = false;

    auto boxOne = (ps::pp::Box*)(ap_box1->shape);
    auto boxOneCenter = ap_box1->M(ap_box1->centerOfMass);

    auto boxTwo = (ps::pp::Box*)(ap_box2->shape);
    auto boxTwoCenter = ap_box2->M(ap_box2->centerOfMass);

    m->normal = (boxOneCenter & boxTwoCenter) | boxOneCenter;
    m->normal.normalize();
    // TODO really bad tmp fix
    m->count = 0;
    //PROBLEM what if edge and line are parallel
    //PROBLEM multiple faces registering collision override normal

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
                    m->pointsOfContact[m->count] = point;
                    // m->normal += normal;
                    ++(m->count);
                    out = true;
                }
            }

        }

    }

    // m->normal /= m->count;
    // m->normal.normalize();

    return out;
}
