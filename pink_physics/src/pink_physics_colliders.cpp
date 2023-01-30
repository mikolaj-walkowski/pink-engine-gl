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

ps::pp::Interval ps::pp::GetInterval(const kln::point* verts, const vec3& axis){
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

bool ps::pp::OverlapOnAxis(ps::pp::Rigidbody* rb1, ps::pp::Rigidbody* rb2, const vec3& axis) {
    kln::point* verts1 = ((ps::pp::Box*) rb1->moved)->verts;
    kln::point* verts2 = ((ps::pp::Box*) rb2->moved)->verts;

    Interval a = GetInterval(verts1, axis);
    Interval b = GetInterval(verts2, axis);
    return ((b.min <= a.max) && (a.min <= b.max));
}

const float* calcBoxOrientation(ps::pp::Rigidbody* rb){
    ps::pp::Box* box = (ps::pp::Box*) rb->moved;

    // TODO chyba lepiej byłoby bez malloc, ale nie ma czasu
    float* orientation = (float*) malloc(sizeof(float)*9);
    uint8_t i = 0;

    for (auto& edge: {box->edges[0], box->edges[4], box->edges[1]})
    {
        auto a = box->verts[edge.first];
        auto b = box->verts[edge.second];

        // Not sure if this really works

        vec3 orientationAxis = {a.x() - b.x(), a.y() - b.y(), a.z() - b.z()};
        
        for (uint8_t j = 0; j<3; ++j) orientation[i++] = orientationAxis[j];
    }

    return orientation;
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
    bool out = f(m->rb1->moved, &m->rb1->M, m->rb2->moved, &m->rb2->M, m);
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

const kln::line* ps::pp::kln_calcBoxOrientation(ps::pp::Rigidbody* rb){
    ps::pp::Box* box = (ps::pp::Box*) rb->shape;

    // TODO chyba lepiej byłoby bez malloc, ale nie ma czasu
    kln::line* orientation = (kln::line*) malloc(sizeof(kln::line)*3);
    uint8_t i = 0;

    for (auto& edge: {box->edges[0], box->edges[4], box->edges[1]})
    {
        auto a = rb->M(box->verts[edge.first]);
        auto b = rb->M(box->verts[edge.second]);
        
        orientation[i++] = a & b;
    }

    return orientation;
}
bool ps::pp::PointInBox(kln::point& inp_point, ps::pp::Rigidbody* rb) {
    bool output = true;

    vec3 point(inp_point.x(), inp_point.y(), inp_point.z());

    //dynamic_cast byłby bezpieczniejszy ale my nie jesteśmy bezpieczni
    auto OBB = (ps::pp::Box*) rb->shape;

    kln::point center_point = rb->M(OBB->center);
    vec3 center(center_point.x(), center_point.y(), center_point.z());

    point -= center;

    float orientation[3][3] = {
        {1.f, 0.f, 0.f},
        {0.f, 1.f, 0.f},
        {0.f, 0.f, 1.f}
    };
    
    for (int i = 0; i < 3; ++i) {
        vec3 axis(
            orientation[i][0],
            orientation[i][1],
            orientation[i][2]);

        //temp solution - TODO
        ps::pp::Interval interval = ps::pp::GetInterval(OBB->verts, axis);
        float projection = Dot(axis, vec3(point[0], point[1], point[2]));

        if (projection > interval.max  || projection < interval.min) {
            output = false;
            break;
        }
    }
    return output;
}

bool ps::pp::ClipToPlane(const kln::plane& plane, std::pair<kln::point, kln::point>& line, kln::point* outPoint) {
    kln::point intersection = (line.first & line.second) ^ plane;

    // bool x = between(
    //     std::max(line.first.x(), line.second.x()),
    //     std::min(line.first.x(), line.second.x()),
    //     intersection.x()
    // );
    // bool y = between(
    //     std::max(line.first.y(), line.second.y()),
    //     std::min(line.first.y(), line.second.y()),
    //     intersection.y()
    // );
    // bool z = between(
    //     std::max(line.first.z(), line.second.z()),
    //     std::min(line.first.z(), line.second.z()),
    //     intersection.z()
    // );
    // // printf("\n\tx: %f, %f, %f\n", line.first.x(), line.second.x(), intersection.x());
    // // printf("\ty: %f, %f, %f\n", line.first.y(), line.second.y(), intersection.y());
    // // printf("\tz: %f, %f, %f\n\n", line.first.z(), line.second.z(), intersection.z());

    // if (x && y && z) {
    *outPoint = intersection;
    return true;
    // }

    // return false;
}

std::vector<kln::point> ps::pp::ClipEdgesToOBB(std::pair<kln::point, kln::point>* edges, ps::pp::Rigidbody* rb) {

    ps::pp::Box* OBB = (ps::pp::Box*) rb->moved;
    std::vector<kln::point> result;
    kln::point intersection;

    for (auto face : OBB->faces){
        auto vert1 = OBB->verts[face[0]];
        auto vert2 = OBB->verts[face[1]];
        auto vert3 = OBB->verts[face[2]];
        auto vert4 = OBB->verts[face[3]];

        auto plane = vert1 & vert2 & vert3;


        for (int j = 0; j < 12; ++j) {
            if (ClipToPlane(plane, edges[j], &intersection)) {
                if (PointInBox(intersection, rb)) {
                    result.push_back(intersection);
                }
            }
        }
    }
    return result;
}

float ps::pp::PenetrationDepth(ps::pp::Rigidbody* rb1, ps::pp::Rigidbody* rb2, const vec3& axis, bool* outShouldFlip) {
    kln::point* a = ((ps::pp::Box*) rb1->moved)->verts;
    kln::point* b = ((ps::pp::Box*) rb2->moved)->verts;

    ps::pp::Interval i1 = ps::pp::GetInterval(a, axis);
    ps::pp::Interval i2 = ps::pp::GetInterval(b, axis);
    
    if (!((i2.min <= i1.max) && (i1.min <= i2.max))) {
        return 0.0f; // No penetration
    }
    
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

    auto box1 = (ps::pp::Box*)(m->rb1->moved);
    auto box2 = (ps::pp::Box*)(m->rb2->moved);

    // I know, it's terrible
    const kln::line* kln_o1 = ps::pp::kln_calcBoxOrientation(m->rb1);
    const kln::line* kln_o2 = ps::pp::kln_calcBoxOrientation(m->rb2);

    kln::line kln_test[15] = {
        kln_o1[0],
        kln_o1[1],
        kln_o1[2],
        kln_o2[0],
        kln_o2[1],
        kln_o2[2]
    };
    
    // TODO jeszcze ta pamięć którą trzeba zwolnić samemu
    // Mało inżynierskie jak na inżynierkę...
    free((void*) kln_o1);
    free((void*) kln_o2);

    for (int i = 0; i < 3; ++i) { // Fill out rest of axes
        kln_test[6 + i * 3 + 0] = kln_test[i] * kln_test[0];
        kln_test[6 + i * 3 + 1] = kln_test[i] * kln_test[1];
        kln_test[6 + i * 3 + 2] = kln_test[i] * kln_test[2];
    }

    const float* o1 = calcBoxOrientation(m->rb1);
    const float* o2 = calcBoxOrientation(m->rb2);
    vec3 test[15] = {
        vec3(o1[0], o1[1], o1[2]),
        vec3(o1[3], o1[4], o1[5]),
        vec3(o1[6], o1[7], o1[8]),
        vec3(o2[0], o2[1], o2[2]),
        vec3(o2[3], o2[4], o2[5]),
        vec3(o2[6], o2[7], o2[8])
    };

    free((void*) o1);
    free((void*) o2);


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
    vec3 axis(0,0,0);

    for (int i = 0; i < 15; ++i) {
        if (abs(kln_test[i] | kln_test[i]) < 0.001f){
            continue;
        }

        depth = ps::pp::PenetrationDepth(m->rb1, m->rb2, test[i], &shouldFlip);
        if (!OverlapOnAxis(m->rb1, m->rb2, test[i])) {
            return false; // Seperating axis found
        }
        else if (depth < m->penetration || m->penetration == 0){
            if (shouldFlip) {
                kln_test[i] = ~kln_test[i];
                test[i] = test[i] * -1.0f;
            }
            normal_line = kln_test[i];
            m->penetration = depth;
            axis = test[i];
        }
    }

    if(!axis[0] && !axis[1] && !axis[2])
    {
        return false; // Coś nie pykło
    }
    
    m->penetration /= 2.0f; // PenetrationDepth zwraca chyba tak naprawdę dwukrotność głębokości penetracji

    std::pair<kln::point, kln::point> edges1[12];
    for(int i = 0; i < 12; ++i)
    {
        // kln::point p1 =  m->rb1->M(box1->verts[box1->edges[i].first]);
        // kln::point p2 =  m->rb1->M(box1->verts[box1->edges[i].second]);
        kln::point p1 =  box1->verts[box1->edges[i].first];
        kln::point p2 =  box1->verts[box1->edges[i].second];
        edges1[i] = std::make_pair(p1, p2);
    }
    std::vector<kln::point> c1 = ps::pp::ClipEdgesToOBB(edges1, m->rb2);

    for(int i = 0; i < 12; ++i)
    {
        // kln::point p1 = m->rb2->M(box2->verts[box2->edges[i].first]);
        // kln::point p2 = m->rb2->M(box2->verts[box2->edges[i].second]);
        kln::point p1 = box2->verts[box2->edges[i].first];
        kln::point p2 = box2->verts[box2->edges[i].second];
        edges1[i] = std::make_pair(p1, p2);
    }
    std::vector<kln::point> c2 = ps::pp::ClipEdgesToOBB(edges1, m->rb1);
    
    for(int i = 0; i < std::min(m->maxContactPoints, (int) c1.size()); ++i)
    {
        m->pointsOfContact[m->count] = c1[m->count];
        ++(m->count);
    }

    for(int i = 0; i < std::min(m->maxContactPoints - m->count, (int) c2.size()); ++i)
    {
        m->pointsOfContact[m->count] = c2[i];
        ++(m->count);
    }

    // int points = 0;
    // for(auto vert : box1->verts)
    // {
    //     if(PointInBox(vert, m->rb2)){
    //         ++points;
    //     }
    // }
    // for(auto vert : box2->verts)
    // {
    //     if(PointInBox(vert, m->rb1))
    //     {
    //         ++points;
    //     }
    // }

    // if(points) printf("points = %d\n", points);

    // Interval interval = GetInterval(box1->verts, axis);
    // float distance = (interval.max - interval.min)* 0.5f - m->penetration * 0.5f;

    // vec3 center = {
    //     box1->center.x(),
    //     box1->center.y(),
    //     box1->center.z()};

    // vec3 pointOnPlane = center + axis * distance;

    // kln::point p1;

    // for (int i = m->count - 1; i>= 0; --i) {
    //     p1 = m->pointsOfContact[i];
    //     vec3 contact = {p1.x(), p1.y(), p1.z()};
    //     contact = contact + (axis * Dot(axis, pointOnPlane - contact));
    //     m->pointsOfContact[i] = kln::point(contact[0], contact[1], contact[2]);
    // }

    //really bad
    // kln::plane normal = normal_line.normalized() | box1->center;

    // m->normal = normal;
    m->normal = (box1->center & box2->center) | box1->center;

    return true;

}
