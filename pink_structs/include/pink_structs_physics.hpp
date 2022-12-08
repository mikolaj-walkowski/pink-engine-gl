#pragma once 
#include <klein/klein.hpp>
#include "nvmath/nvmath.h"
#include "pink_structs.hpp"

namespace ps::pp {

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
        kln::motor B = kln::motor();
        
        kln::motor dM;
        kln::motor dB;
        
        kln::point centerOfMass;

        ShapeType shapeType;
        void* shape;

    };

} //namespace ps::pp