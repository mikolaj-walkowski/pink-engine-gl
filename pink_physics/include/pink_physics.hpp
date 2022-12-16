#pragma once
#include "pink_physics_colliders.hpp"
#include "pink_structs.hpp"
#include "nvmath/nvmath.h"
#include <stdint.h>
#include <vector>

namespace ps::pp {

    class Engine;

    typedef void (*SimulateFunc)(Rigidbody*);
    typedef void (*ColliderFunc)(Engine*, Rigidbody*);
    typedef void (*ResolverFunc)(Engine*, Rigidbody*);
    typedef void (*InterpolationFunc)(Engine*, Rigidbody*);

    class Engine {
    public:
        ps::WordState* in;
        ps::WordState* out;
        float dT;

        SimulateFunc simulate;
        ColliderFunc collide;
        ResolverFunc resolve;
        InterpolationFunc interpolation;

        static const int maxNumber = 10;
        struct {
            int size;
            Rigidbody* collisions[Engine::maxNumber];
            Manifold collisionData[Engine::maxNumber];
        }collision_props;

        struct {
            float step = 0.4f;
            int iterations = 10;
        } interpolation_props;

#ifndef NDEBUG
        struct {
            bool stop = true;
            std::vector<std::string> collisions;
            std::vector<Manifold> collisionData;
            
        }debug_data;
#endif

        void step(ps::WordState*, ps::WordState*, float);
        void renderUI();
        Engine(SimulateFunc sF, ColliderFunc cF, ResolverFunc rF, InterpolationFunc iF);
    };

    Rigidbody rigidbodyCreate(kln::motor, ShapeType, void*, int);
    void rigidbodyDestroy(Rigidbody*);


    void basicSimulate(Rigidbody*);
    void basicCollider(Engine*, Rigidbody*);
    void basicResolver(Engine*, Rigidbody*);
    void eulerInterpolation(Engine*, Rigidbody*);

} // namespace ps::pp