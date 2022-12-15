#pragma once 
#include <klein/klein.hpp>
#include "nvmath/nvmath.h"
#include "pink_structs.hpp"

#define BM(x) (1<<(x))
namespace ps::pp {

    typedef struct Manifold {
        static const int maxContactPoints= 10;
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
        float radius;
        kln::point center;
    };

    struct Plane {
        kln::plane plane;
    };

    struct Box {
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

    static int shapeSize[ST_SIZE] = {sizeof(Sphere), sizeof(Plane), sizeof(Box)};
    static char* shapeName[ST_SIZE] = { "sphere","plane","cube" };
} //namespace ps::pp