#pragma once 
#include <klein/klein.hpp>


namespace pp{

    enum ShapeType{
        SHAPE_TYPE_SPHERE,
    };

    struct Sphere{
        int radius;
    };

    struct Rigidbody {
        kln::motor M;
        kln::point center;
        
        ShapeType shapeType;
        void* shape;
    };
} //namespace pp