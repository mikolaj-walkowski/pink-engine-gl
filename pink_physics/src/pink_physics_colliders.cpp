#include "pink_physics_colliders.hpp"

float Dot(const vec3& l, const vec3& r) {
    return l.x * r.x + l.y * r.y + l.z * r.z;
}

vec3 Cross(const vec3& l, const vec3& r) {
    vec3 result;
    result.x = l.y * r.z - l.z * r.y;
    result.y = l.z * r.x - l.x * r.z;
    result.z = l.x * r.y - l.y * r.x;
    return result; // Done
}

ps::pp::Interval ps::pp::GetInterval(const kln::point* verts, const vec3& axis) {
    Interval result;
    result.min = result.max = Dot(axis, vec3(verts[0].x(), verts[0].y(), verts[0].z()));
    for (int i = 1; i < 8; ++i) {
        float projection = Dot(axis, vec3(verts[i].x(), verts[i].y(), verts[i].z()));
        result.min = (projection < result.min) ?
            projection : result.min;
        result.max = (projection > result.max) ?
            projection : result.max;
    }
    return result;
}

ps::pp::Interval::Interval(float a, float b) {
    min = a < b ? a : b;
    max = a > b ? a : b;
    len = max - min;
}

bool ps::pp::Interval::overlap(const Interval& oth, Interval& res) {

    res.min = fmaxf(res.min, min);
    res.max = fminf(res.max, max);
    res.len = res.max - res.min;

    return (min <= max);
}


bool ps::pp::Interval::operator[](float& a) {
    return this->min <= a && this->max >= a;
}
bool ps::pp::Interval::operator()(float& a) {
    return this->min < a&& this->max > a;
}

bool ps::pp::OverlapOnAxis(ps::pp::Rigidbody* rb1, ps::pp::Rigidbody* rb2, const vec3& axis) {
    kln::point* verts1 = ((ps::pp::Box*)rb1->shape)->verts;
    kln::point* verts2 = ((ps::pp::Box*)rb2->shape)->verts;

    Interval a = GetInterval(verts1, axis);
    Interval b = GetInterval(verts2, axis);
    return ((b.min <= a.max) && (a.min <= b.max));
}

void calcBoxOrientation(ps::pp::Rigidbody* rb, vec3* orientation) {
    ps::pp::Box* box = (ps::pp::Box*)rb->shape;

    uint8_t i = 0;

    for (auto& edge : { box->edges[0], box->edges[4], box->edges[1] })
    {
        auto a = box->verts[edge.first];
        auto b = box->verts[edge.second];

        // Not sure if this really works, to mia by dot, bo jak tak to git? 
        orientation[i++] = { a.x() - b.x(), a.y() - b.y(), a.z() - b.z() };
    }
}

bool ps::pp::eCmp(float a, float b) {
    float epsilon = 0.0001f;
    return abs(a - b) < epsilon;
}


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
    bool out = f(m->rb1->shape, &m->rb1->M, m->rb2->shape, &m->rb2->M, m);
    return out;
}

bool ps::pp::defaultCollider(BaseShape* b1, kln::motor* m1, BaseShape* b2, kln::motor* m2, Manifold* m) {
    return false;
}

bool ps::pp::sphereToPlane(BaseShape* _plane, kln::motor* m1, BaseShape* _sphere, kln::motor* m2, ps::pp::Manifold* m) {
    auto plane = (((ps::pp::Plane*)_plane)->plane);
    auto sphere = (ps::pp::Sphere*)_sphere;

    auto center = (sphere->center);

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

    auto c1 = (s1->center);
    auto c2 = (s2->center);
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


bool eCmp(float a, float b, float epsilon) {
    return abs(a - b) < epsilon;
}

kln::point* findApprox(kln::point* arr, int size, kln::point& p) {
    float f = 0.1f;
    for (int i = 0; i < size; i++)
    {
        auto& a = arr[i];
        if (
            eCmp(a.e013(), p.e013(), f) &&
            eCmp(a.e021(), p.e021(), f) &&
            eCmp(a.e032(), p.e032(), f) &&
            eCmp(a.e123(), p.e123(), f)
            ) {
            return &a;
        }
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
    auto boxCenter = box->center;
    auto plane = (((ps::pp::Plane*)_plane)->plane);
    bool out = false;
    m->normal = plane * -1;
    m->count = 0;
    //m->normal.normalize();

    for (auto p : box->edges) {
        auto i = (box->verts[p.first]);
        auto j = (box->verts[p.second]);

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

void ps::pp::kln_calcBoxOrientation(ps::pp::Rigidbody* rb, kln::line* orientation) {
    ps::pp::Box* box = (ps::pp::Box*)rb->shape;
    uint8_t i = 0;

    for (auto& edge : { box->edges[0], box->edges[4], box->edges[1] })
    {
        auto a = box->verts[edge.first];
        auto b = box->verts[edge.second];

        orientation[i++] = a & b;
    }

}

// inp_point must be in word space 
bool ps::pp::PointInBox(kln::point& inp_point, ps::pp::Rigidbody* rb) {
    kln::point p_Rel = (~rb->M)(inp_point);

    vec3 point(p_Rel.x(), p_Rel.y(), p_Rel.z());

    //dynamic_cast byłby bezpieczniejszy ale my nie jesteśmy bezpieczni
    auto OBB = (ps::pp::Box*)rb->shape;

    kln::point v[8];
    OBB->getBodyVerts(v);

    for (int i = 0; i < 3; ++i) {
        vec3 axis(0.f, 0.f, 0.f);
        axis[i] = 1.f;

        ps::pp::Interval interval = ps::pp::GetInterval(v, axis);
        float projection = Dot(axis, point);

        if (!interval[projection]) {
            return false;
        }
    }
    return true;
}

int ps::pp::ClipToPlane(const kln::plane& plane, std::pair<kln::point, kln::point>& line, kln::point* outPoint, kln::point* outPoint2) {
    kln::line l = line.first & line.second;
    kln::point intersection = (l) ^ plane;
    intersection.normalize();
    if (!ps::pp::eCmp(intersection.e123(), 0.f)) {
        float i_val = (l | intersection).e0();
        Interval interval((l | line.first).e0(), (l | line.second).e0());

        if (!interval[i_val])
            return 0;

        *outPoint = intersection;
        return 1;
    }
    else {
        if (ps::pp::eCmp((plane & line.first).scalar(), 0.0f)) {
            *outPoint = kln::project(line.first, plane).normalized();
            *outPoint2 = kln::project(line.second, plane).normalized();
            return 2;
        }
    }
    return 0;
}

std::vector<kln::point> ps::pp::ClipEdgesToOBB(std::pair<kln::point, kln::point>* edges, ps::pp::Rigidbody* rb) {

    ps::pp::Box* OBB = (ps::pp::Box*)rb->shape;
    std::vector<kln::point> result;
    kln::point intersection;
    kln::point intersection2;

    for (auto face : OBB->faces) {
        auto vert1 = OBB->verts[face[0]];
        auto vert2 = OBB->verts[face[1]];
        auto vert3 = OBB->verts[face[2]];
        auto vert4 = OBB->verts[face[3]];

        auto plane = vert1 & vert2 & vert3;

        for (int j = 0; j < 12; ++j) {
            switch (ClipToPlane(plane, edges[j], &intersection, &intersection2))
            {
            case 0:
                break;
            case 2:
                if (PointInBox(intersection2, rb)) {
                    result.push_back(intersection2);
                }
            case 1:// fallthrough
                if (PointInBox(intersection, rb)) {
                    result.push_back(intersection);
                }
                break;
            }
        }
    }
    return result;
}

float ps::pp::PenetrationDepth(ps::pp::Rigidbody* rb1, ps::pp::Rigidbody* rb2, const vec3& axis, bool* outShouldFlip) {
    kln::point* a = ((ps::pp::Box*)rb1->shape)->verts;
    kln::point* b = ((ps::pp::Box*)rb2->shape)->verts;

    ps::pp::Interval i1 = ps::pp::GetInterval(a, axis);
    ps::pp::Interval i2 = ps::pp::GetInterval(b, axis);

    // ps::pp::Interval res;
    // if (!i1.overlap(i2, res)) {
    //     return 0.0f; // No penetration
    // }

    // if (outShouldFlip != nullptr) {
    //     *outShouldFlip = (i2.min < i1.min);
    // }

    // return res.len;
    if (!((i2.min <= i1.max) && (i1.min <= i2.max))) {
        return 0.0f; // No penetration
    }

    // Co  tu właściwie robisz: Rozmiar przecięcia interwałów? Rozmiar różnicy ?  
    float len1 = i1.max - i1.min;
    float len2 = i2.max - i2.min;
    float min = fminf(i1.min, i2.min);
    float max = fmaxf(i1.max, i2.max);
    float length = max - min;

    if (outShouldFlip != nullptr) {
        *outShouldFlip = (i2.min < i1.min);
    }

    return (len1 + len2) - length;
}

bool ps::pp::boxToBox(BaseShape* ap_box1_, kln::motor* m1, BaseShape* ap_box2_, kln::motor* m2, Manifold* m) {
    bool out = false;

    auto box1 = (ps::pp::Box*)(m->rb1->shape);
    auto box2 = (ps::pp::Box*)(m->rb2->shape);

    kln::line kln_test[15] = {};

    ps::pp::kln_calcBoxOrientation(m->rb1, kln_test);
    ps::pp::kln_calcBoxOrientation(m->rb2, kln_test + 3);

    for (int i = 0; i < 3; ++i) { // Fill out rest of axes
        kln_test[6 + i * 3 + 0] = kln_test[i] * kln_test[0];
        kln_test[6 + i * 3 + 1] = kln_test[i] * kln_test[1];
        kln_test[6 + i * 3 + 2] = kln_test[i] * kln_test[2];
    }

    vec3 test[15] = {};
    calcBoxOrientation(m->rb1, test);
    calcBoxOrientation(m->rb2, test + 3);

    for (int i = 0; i < 3; ++i) { // Fill out rest of axes
        test[6 + i * 3 + 0] = Cross(test[i], test[0]);
        test[6 + i * 3 + 1] = Cross(test[i], test[1]);
        test[6 + i * 3 + 2] = Cross(test[i], test[2]);
    }

    bool shouldFlip;
    float depth;
    kln::line normal_line;
    m->count = 0;
    m->penetration = 0;
    bool overlap = false;
    vec3 axis(0, 0, 0);

    for (int i = 0; i < 15; ++i) {
        if (abs(kln_test[i] | kln_test[i]) < 0.001f) {
            continue;
        }

        depth = ps::pp::PenetrationDepth(m->rb1, m->rb2, test[i], &shouldFlip);
        if (!OverlapOnAxis(m->rb1, m->rb2, test[i])) {
            return false; // Seperating axis found
        }
        else if (depth < m->penetration || m->penetration == 0) {
            if (shouldFlip) {
                kln_test[i] = ~kln_test[i];
                test[i] = test[i] * -1.0f;
            }
            normal_line = kln_test[i];
            m->penetration = depth;
            axis = test[i];
        }
    }

    if (!axis[0] && !axis[1] && !axis[2])
    {
        return false; // Coś nie pykło
    }

    m->penetration /= 2.0f; // PenetrationDepth zwraca chyba tak naprawdę dwukrotność głębokości penetracji

    Rigidbody* rbs[2] = { m->rb1, m->rb2 };
    for (int i = 0; i < 2; i++) {
        auto rb = rbs[i];
        auto box = (ps::pp::Box*)(rb->shape);
        std::pair<kln::point, kln::point> edges[12];

        for (int i = 0; i < 12; ++i)
        {
            kln::point p1 = box->verts[box->edges[i].first];
            kln::point p2 = box->verts[box->edges[i].second];
            edges[i] = std::make_pair(p1, p2);
        }

        std::vector<kln::point> c = ps::pp::ClipEdgesToOBB(edges, rbs[1 - i]);

        for (int i = 0; m->count < m->maxContactPoints && i < (int)c.size(); ++i)
        {
            kln::point* p = findApprox(m->pointsOfContact, m->count, c[i]);
            if (p == NULL) {
                m->pointsOfContact[m->count] = c[i];
                ++(m->count);
            }
        }
    }

    m->normal = kln::plane(-axis[0], -axis[1], -axis[2], 0.f);

    return true;
}
