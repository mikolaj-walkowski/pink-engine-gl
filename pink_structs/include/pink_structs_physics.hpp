#pragma once 
#include <klein/klein.hpp>
#include "nvmath/nvmath.h"
#include "pink_structs.hpp"

#define BM(x) (int)(1<<((int)(x)))

namespace ps::pp {
    struct Rigidbody;

    typedef struct Manifold {
        static const int maxContactPoints = 10;
        int count = 0;
        float penetration;
        kln::plane normal;
        kln::point pointsOfContact[maxContactPoints];
        Rigidbody* rb1;
        Rigidbody* rb2;
    } Manifold;

    struct Spring {

        int rb1;  //TODO Bad
        int rb2; //TODO

        float restingLength;
        float k;
    };

    struct WheelJoin {
        int body; //TODO Bad
        int wheel;

        float travel;
        kln::point Att[2];
        kln::line constraint;
    };

    enum ShapeType {
        ST_PLANE,
        ST_SPHERE,
        ST_BOX,
        ST_CYLINDER,
        ST_COMPOSITE,
        ST_SIZE
    };
    enum BodyType {
        BT_STATIC = 0,
        BT_DYNAMIC = 1
    };

    class BaseShape {
    public:
        kln::line inertia;
        kln::point center;
        ShapeType type;
        float mass;

        virtual void move(const kln::motor& M, const BaseShape* og) = 0;
    };

    struct Rigidbody {
        kln::motor M;
        kln::line B;

        kln::motor dM;
        kln::line dB;

        kln::line F;

        kln::point centerOfMass;

        BodyType bodyType;

        BaseShape* shape;
        BaseShape* moved;
    };

    static char* shapeName[ST_SIZE] = { "plane","sphere","cube_multi","cylinder", "composite" };
} //namespace ps::pp