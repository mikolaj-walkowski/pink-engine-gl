#pragma once 
#include <klein/klein.hpp>
#include "nvmath/nvmath.h"
#include "pink_structs.hpp"

namespace ps::pp {

    typedef struct Manifold {
        static const int maxContactPoints= 10;
        int count;
        float penetration;
        kln::line normal;
        kln::point pointsOfContact[maxContactPoints];
    } Manifold;

    enum ShapeType {
        SHAPE_TYPE_SPHERE = 1 << 0,
        SHAPE_TYPE_PLANE = 1 << 1,
    };

    struct Sphere {
        float radius;
        kln::point center;
    };

    struct Plane {
        kln::plane plane;
    };

    struct Rigidbody {
        kln::motor M;
        kln::line B = kln::line();
        
        kln::motor dM;
        kln::line dB;
        
        kln::point centerOfMass;

        ShapeType shapeType;
        void* shape;

    };

} //namespace ps::pp