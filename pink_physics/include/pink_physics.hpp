#pragma once
#include "pink_physics_colliders.hpp"
#include "pink_structs.hpp"
#include "nvmath/nvmath.h"
#include <stdint.h>
#include <vector>

namespace ps::pp {

    class Engine;


    typedef void (*SimulateFunc)(Rigidbody*);
    // typedef void (*ColliderFunc)(Engine*, Rigidbody*, std::vector<ps::Object*>&, EngineCollisionStore&);
    typedef void (*ResolverFunc)(Engine*);
    typedef void (*IntegrationFunc)(Engine*, Rigidbody*);


    class Engine: public Module {
    public:
        virtual void registerObject(Object*);
        virtual void deregisterObject(Object*);

        std::vector<Object*> allBodies;
        std::vector<Object*> simulatedRbs;
        std::vector<Object*> constrainingRbs;
        
        std::vector<Spring> springs;
        std::vector<Joint> joins;

        float dT;

        SimulateFunc simulate;
        // ColliderFunc collide;
        ResolverFunc resolve;
        IntegrationFunc integrator;

        static const int maxNumber = 10;
        struct CollisionStore{
            int size;
            Manifold collisionData[Engine::maxNumber];
        }collision_props, constrainingCollisions;

        struct {
            float step = 0.4f;
            int iterations = 4;
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
        

        
        void step(int, float);
        void applySprings();
        void enforceJoints();
        void renderUI();
        void run(int*,float*,bool* );
        void vecCollider(Rigidbody* rb, std::vector<ps::Object*>& vec, CollisionStore& CS);
        
        Engine(SimulateFunc sF, ResolverFunc rF, IntegrationFunc iF);
    };

    Rigidbody rigidbodyCreate(kln::motor, ShapeType, void*, int);
    void rigidbodyDestroy(Rigidbody*);

    void checkJoin(Engine* e, Joint* j);
    void enforceJoint(Engine* e, Joint* j);

    void basicSimulate(Rigidbody*);
    void carSimulate(Rigidbody*);

    void basicResolver(Engine*);
    void solidResolver(Engine*);

    void basicCollider(Engine*, Rigidbody*);

    void eulerIntegration(Engine*, Rigidbody*);
    void verletIntegration(Engine*, Rigidbody*);

    void applyImpulseNormal(Rigidbody* rb, kln::line dir, float a);
    void applyImpulseStatic(Rigidbody* rb, kln::line dir, float a);

    void print(const char*, kln::point);
} // namespace ps::pp