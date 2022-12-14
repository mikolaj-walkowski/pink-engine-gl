#pragma once 
#include <pga3d.hpp>
#include "nvmath/nvmath.h"
#include "pink_structs.hpp"

namespace ps::pp {

    typedef struct Manifold {
        static const int maxContactPoints= 10;
        int count;
        float penetration;
        PGA3D normal;
        PGA3D pointsOfContact[maxContactPoints];
    } Manifold;

    enum ShapeType {
        SHAPE_TYPE_SPHERE = 1 << 0,
        SHAPE_TYPE_PLANE = 1 << 1,
    };

    struct Sphere {
        float radius;
        PGA3D center;
    };

    struct Plane {
        PGA3D plane;
    };

    struct Rigidbody {
        PGA3D M;
        PGA3D B;
        
        PGA3D  dM;
        PGA3D  dB;
        
        PGA3D  centerOfMass;

        ShapeType shapeType;
        void* shape;
    };

} //namespace ps::pp