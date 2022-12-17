#pragma once 
#include <klein/klein.hpp>
#include "nvmath/nvmath.h"
#include "pink_structs.hpp"

#define BM(x) (int)(1<<((int)(x)))

namespace ps::pp {

    typedef struct Manifold {
        static const int maxContactPoints = 10;
        int count = 0;
        float penetration;
        kln::line normal;
        kln::point pointsOfContact[maxContactPoints];
    } Manifold;

    enum ShapeType {
        ST_SPHERE,
        ST_PLANE,
        ST_BOX,
        ST_SIZE
    };

    struct Sphere {
        kln::line inertia;
        float radius;
        kln::point center;
    };

    struct Box {
        kln::line inertia;
        kln::point verts[8];
        static inline std::pair<int, int> edges[12] =
        {
        {0,1},{0,2},{1,3},{2,3},
        {0,4},{2,6},{1,5},{3,7},
        {4,5},{4,6},{5,7},{6,7}
        };
        // TODO add precalculated lines and constructor
    };

    //TODO unfuck inertia

    struct Plane {
        kln::line inertia;
        kln::plane plane;
    };

    struct Rigidbody {
        kln::motor M;
        kln::line B;

        kln::motor dM;
        kln::line dB;

        kln::point centerOfMass;

        ShapeType shapeType;
        void* shape;
    };

    static int shapeSize[ST_SIZE] = { sizeof(Sphere), sizeof(Plane), sizeof(Box) };
    static char* shapeName[ST_SIZE] = { "sphere","plane","cube" };
} //namespace ps::pp