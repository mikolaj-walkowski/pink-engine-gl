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

        static const int maxNumber = 10;
        struct {
            int size;
            Rigidbody* collisions[Engine::maxNumber];
            Manifold collisionData[Engine::maxNumber];
        }collision_props;

        struct {
            float step;
            int iterations;
        } interpolation_props;

        void step(ps::WordState*, ps::WordState*);
        Engine(SimulateFunc sF, ColliderFunc cF, ResolverFunc rF, InterpolationFunc iF);
    };

    void basicSimulate(Rigidbody*);
    void basicCollider(Engine*, Rigidbody*);
    void basicResolver(Engine*, Rigidbody*);
    void eulerInterpolation(Engine*, Rigidbody*);

} // namespace ps::pp