#include "pink_physics_shapes.hpp"


ps::pp::Box::Box(float x, float y, float z, float _mass, kln::motor offset) {
    type = ST_BOX;
    mass = _mass;

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
    mult = kln::point(x, y, z);
    move(kln::uMotor());
}

void ps::pp::Box::move(const kln::motor& M) {
    center = M(kln::origin());
    
    for (int i = 0; i < 8; i++)
    {
        verts[i] = M(_verts[i].mult(mult));
    }
}


ps::pp::Sphere::Sphere(float r, float _mass, kln::motor offset) {
    type = ST_SPHERE;
    mass = _mass;

    float a = 0.4f * mass;

    inertia = kln::motor(
        1.f, 1.f, 1.f, 1.f,
        a * (r * r),
        a * (r * r),
        a * (r * r),
        1.f
    );
    mult = kln::point(r, r, r);
    radius = r;
    move(kln::uMotor());
}

void ps::pp::Sphere::move(const kln::motor& M) {
    center = M(kln::origin());
}

ps::pp::Plane::Plane(kln::plane p) {
    type = ST_PLANE;
    mass = 1.f;
    mult = kln::point(1.f, 1.f, 1.f);
    _plane = p;
    inertia = kln::motor(1.f, 1.f, 1.f, 1.f, 1.f, 1.f, 1.f, 1.f);
    move(kln::uMotor());
}

void ps::pp::Plane::move(const kln::motor& M) {
    center = M(kln::origin());
    plane = M(_plane);
}

ps::pp::Cylinder::Cylinder(float len, float r, float _mass, kln::motor offset) {
    type = ST_CYLINDER;
    mass = _mass;

    inertia = offset(kln::motor(
        1.f, 1.f, 1.f, 1.f,
        (1.f / 12.f) * mass * (3 * r * r + len * len),
        (1.f / 12.f) * mass * (3 * r * r + len * len),
        0.5f * mass * (r * r),
        1.f
    ));

    r /= 2.f;
    mult = kln::point(r, len / 2.f, r);
    move(kln::uMotor());
}

void ps::pp::Cylinder::move(const kln::motor& M) {
    center = M(kln::origin());
    
    for (int i = 0; i < 2; i++)
    {
        caps[i] = M(_caps[i].mult(mult));
    }
}

ps::pp::Composite::Composite(std::vector < std::pair<BaseShape*, kln::motor>> data) {
    children = data;
}

void ps::pp::Composite::move(const kln::motor& M) {
    center = M(kln::origin());

    for (int i = 0; i < children.size(); ++i) {
        children[i].first->move(M*children[i].second);
    }
}

ps::pp::Composite::~Composite() {
    for (int i = 0; i < children.size(); ++i) {
        delete children[i].first;
    }
}