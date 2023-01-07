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
        Rigidbody* rb;
    } Manifold;

    enum ShapeType {
        ST_SPHERE,
        ST_PLANE,
        ST_BOX,
        ST_SIZE
    };

    enum BodyType {
        BT_STATIC = 0,
        BT_DYNAMIC = 1
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
        {0,1},{0,3},{1,2},{2,3},
        {0,4},{1,5},{2,6},{3,7},
        {4,5},{4,7},{5,6},{6,7}
        };
        static inline std::vector<int> faces[6] =
        {
            {0,1,2,3},          // edges[0] + edges[1] + edges[2] + edges[3] (top)
            {0,1,5,4},          // edges[0] + edges[4] + edges[5] + edges[8] (side)
            {0,3,7,4},          // edges[1] + edges[7] + edges[9] + edges[4] (side)
            {1,2,6,5},          // edges[2] + edges[6] + edges[10] + edges[5] (side)
            {2,3,7,6},          // edges[3] + edges[7] + edges[11] + edges[6] (side)
            {4,5,6,7}           // edges[8] + edges[9] + edges[10] + edges[11] (bottom)
        };
        // TODO add precalculated lines and constructor
    };

    struct Cylinder {
        kln::line inertia;
        kln::line centerLine;
        kln::point caps[2];
        float r;
    };

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

        BodyType bodyType;
        ShapeType shapeType;
        void* shape;
    };

    static int shapeSize[ST_SIZE] = { sizeof(Sphere), sizeof(Plane), sizeof(Box) };
    static char* shapeName[ST_SIZE] = { "sphere","plane","cube" };
} //namespace ps::pp