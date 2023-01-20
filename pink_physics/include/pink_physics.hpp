#pragma once
#include "pink_physics_colliders.hpp"
#include "pink_structs.hpp"
#include "nvmath/nvmath.h"
#include <stdint.h>
#include <vector>

namespace ps::pp {

    class Engine;


    typedef void (*SimulateFunc)(Rigidbody*);
    typedef void (*ColliderFunc)(Engine*, Rigidbody*, std::vector<ps::Object>&);
    typedef void (*ResolverFunc)(Engine*);
    typedef void (*IntegrationFunc)(Engine*, Rigidbody*);

    
    class Engine {
    public:
        ps::WordState* in;
        ps::WordState* out;
        float dT;

        SimulateFunc simulate;
        ColliderFunc collide;
        ResolverFunc resolve;
        IntegrationFunc integrator;

        static const int maxNumber = 10;
        struct {
            int size;
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
            long step = -1;
            bool stepped = false;
            float dT = 1.0f / 5.0f;

        }debug_data;
#endif
        std::vector<Spring> springs;
        std::vector<Join> joins;
        void step(ps::WordState*, ps::WordState*, float);
        void applySprings();
        void enforceJoints();
        void renderUI();
        Engine(SimulateFunc sF, ColliderFunc cF, ResolverFunc rF, IntegrationFunc iF);
    };

    Rigidbody rigidbodyCreate(kln::motor, ShapeType, void*, int);
    void rigidbodyDestroy(Rigidbody*);

    void checkJoin(Engine* e, Join* j);
    void enforceJoint(Engine* e, Join* j);
    
    void basicSimulate(Rigidbody*);
    void carSimulate(Rigidbody*);

    void basicResolver(Engine*);
    void solidResolver(Engine*);
    
    void basicCollider(Engine*, Rigidbody*);
    void vecCollider(Engine* e, Rigidbody* rb, std::vector<ps::Object>& vec);

    void eulerIntegration(Engine*, Rigidbody*);
    void verletIntegration(Engine*, Rigidbody*);
    
    void applyImpulseNormal(Rigidbody* rb, kln::line dir, float a);
    void applyImpulseStatic(Rigidbody* rb, kln::line dir, float a);

    void print(const char*, kln::point);
} // namespace ps::pp