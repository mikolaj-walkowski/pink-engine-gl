#pragma once 
#include <klein/klein.hpp>
#include "nvmath/nvmath.h"
#include "pink_structs.hpp"

#define BM(x) (int)(1<<((int)(x)))

namespace ps::pp {
    class Rigidbody;
    typedef void (*applyImpulse)(Rigidbody* rb, kln::line dir, float scalar);//TODO yuck

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

        Rigidbody* rb1;
        Rigidbody* rb2;

        kln::motor rb1atch;
        kln::motor rb2atch;

        float restingLength;
        float k;
    };

    struct Joint {
        Rigidbody* parent;
        Rigidbody* child;

        kln::point Att[2];
        kln::line constraint;
        
        kln::line parentInertiaMask;
        kln::line childInertiaMask;

        bool valid = true;

        float travel;
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
        nvmath::mat4f size;
        ShapeType type;
        float mass;

        virtual void move(const kln::motor& M, const BaseShape* og) = 0;
    };

    class Rigidbody {
        public:
        kln::motor M;
        kln::line B;

        kln::motor dM;
        kln::line dB;

        kln::line F;

        kln::point centerOfMass;

        BodyType bodyType;

        BaseShape* shape;
        BaseShape* moved;

        applyImpulse apply; // TODO yuck 

        int* joins; //TODO yuck2 
        int joinSize;
    };


    static const char* shapeName[ST_SIZE] = { "plane","sphere","cube_multi","cylinder", "composite" };
} //namespace ps::pp