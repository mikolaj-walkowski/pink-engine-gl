#pragma once 

namespace pp{

    enum ShapeType{
        SHAPE_TYPE_SPHERE,
    };

    struct Sphere{
        int radius;
    };

    struct Rigidbody{
        ShapeType shapeType;
        void* shape;
    };
} //namespace pp