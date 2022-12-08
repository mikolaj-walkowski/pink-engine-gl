#pragma once 
#include <klein/klein.hpp>
#include "nvmath/nvmath.h"
#include "pink_structs.hpp"

namespace ps::pp {

    enum ShapeType {
        SHAPE_TYPE_SPHERE,
        SHAPE_TYPE_PLAIN,
    };

    struct Sphere {
        float radius;
    };

    struct Plain {
        kln::line normal;
    };

    struct Rigidbody {
        kln::motor M;
        kln::line B = kln::line();
        
        kln::motor dM;
        kln::line dB;
        
        kln::point center;

        ShapeType shapeType;
        void* shape;

    };

} //namespace ps::pp