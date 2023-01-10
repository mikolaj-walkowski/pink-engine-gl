#include "pink_physics_shapes.hpp"


ps::pp::Box::Box(float x, float y, float z, float mass, kln::motor offset) {
    type = ST_BOX;

    float a = mass / 12.f;
    inertia = offset(kln::motor(
        1.f, 1.f, 1.f, 1.f,
        a * (y * y + z * z),
        a * (z * z + x * x),
        a * (x * x + y * y),
        1.f
    ));

    x /= 2.f;
    y /= 2.f;
    z /= 2.f;
    verts[0] = (kln::point(-x, y, z));
    verts[1] = (kln::point(x, y, z));
    verts[2] = (kln::point(x, y, -z));
    verts[3] = (kln::point(-x, y, -z));
    verts[4] = (kln::point(-x, -y, z));
    verts[5] = (kln::point(x, -y, z));
    verts[6] = (kln::point(x, -y, -z));
    verts[7] = (kln::point(-x, -y, -z));


    // for (int i = 0; i < 8; i++) {
    //     auto p = verts[i];
    //     printf("Point: %f,%f,%f,%f\n", p.e013(), p.e021(), p.e032(), p.e123());
    // }
}

ps::pp::Sphere::Sphere(float r, float mass, kln::motor offset) {
    type = ST_SPHERE;

    float a = 0.4f * mass;

    inertia = kln::motor(
        1.f, 1.f, 1.f, 1.f,
        a * (r * r),
        a * (r * r),
        a * (r * r),
        1.f
    );
    radius = r;
    center = kln::point(0, 0, 0);
}

ps::pp::Plane::Plane(kln::plane p) {
    type = ST_PLANE;

    plane = p;
    inertia = kln::motor(1.f, 1.f, 1.f, 1.f, 1.f, 1.f, 1.f, 1.f);
}

ps::pp::Cylinder::Cylinder(float len, float r, float m, kln::motor offset) {
    type = ST_CYLINDER;
    inertia = offset(kln::motor(
        1.f, 1.f, 1.f, 1.f,
        (1.f / 12.f) * m * (3 * r * r + len * len),
        (1.f / 12.f) * m * (3 * r * r + len * len),
        0.5f * m * (r * r),
        1.f
    ));

    r /= 2.f;

    auto t = kln::translator(r, 0.f, 1.f, 0.f);
    caps[0] = t(kln::origin());

    t = kln::translator(-r, 0.f, 1.f, 0.f);
    caps[1] = t(kln::origin());

    centerLine = (caps[0] & caps[1]).normalized();

}

ps::pp::Composite::Composite(std::vector < std::pair<BaseShape*, kln::motor>> data) {
    children = data;
}

ps::pp::Composite::~Composite() {
    for (int i = 0; i < children.size(); ++i) {
        delete children[i].first;
    }
}