#pragma once
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

        SimulateFunc simulate;
        ColliderFunc collide;
        ResolverFunc resolve;
        InterpolationFunc interpolation;

        struct {
            static const int maxNumber = 10;
            int size;
            int collisions[Engine::collision_props.maxNumber];
        }collision_props;

        struct {
            float step;
            int iterations;
        } interpolation_props;

        void step(ps::WordState*, ps::WordState*);
        Engine(SimulateFunc, ColliderFunc, ResolverFunc);
    };

    void basicSimulate(Rigidbody*);
    void basicCollider(Engine*, Rigidbody*);
    void basicResolver(Engine*, Rigidbody*);
    void eulerInterpolation(Engine*, Rigidbody*);

} // namespace ps::pp