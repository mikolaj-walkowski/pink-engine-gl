#include "pink_physics.hpp"
#include "pink_structs.hpp"
#include "pink_physics_colliders.hpp"

#include "backends/imgui_impl_glfw.h"
#include "imgui.h"


kln::line::line(motor l) {
    p1_ = l.p1_;
    p2_ = l.p2_;
}

void ps::pp::Engine::renderUI() {

}

void ps::pp::eulerInterpolation(ps::pp::Engine* e, ps::pp::Rigidbody* rb) {
    int iterations = e->interpolation_props.iterations;
    float step = (e->dT / ((float)iterations)) * e->interpolation_props.step;
    for (int i = 0; i < iterations; i++)
    {

        e->simulate(rb);
        rb->M = rb->M + (step * rb->dM);
        rb->B = rb->B + (step * rb->dB);

        e->collide(e, rb);
        e->resolve(e, rb);
    }
    rb->M.normalize();
}

void ps::pp::basicSimulate(ps::pp::Rigidbody* rb) {
    kln::line G = (~(rb->M))(kln::ideal_line(0.f, -9.81f, 0.f));
    kln::line Damp = (-0.25f * rb->B);

    kln::line F = kln::line() + G + Damp; // + wszystkie siły
    // TODO inertia 
    rb->dM = -0.5f * (rb->M * ((kln::motor)(rb->B)));
    rb->dB = F - 0.5f * !(kln::line(!rb->B * rb->B - rb->B * !rb->B));
}

void ps::pp::basicCollider(ps::pp::Engine* e, ps::pp::Rigidbody* rb) {
    auto staticObjects = e->out->staticObjects;

    auto collisions = e->collision_props.collisions;
    auto collisionData = e->collision_props.collisionData;
    auto collisionSize = &e->collision_props.size;
    auto collisionMaxSize = Engine::maxNumber;

    *collisionSize = 0;


    for (auto i : staticObjects)
    {
        if (*collisionSize >= collisionMaxSize) break;
        if (rb == &i.rigidbody) continue;

        auto type = BM(rb->shapeType) | BM(i.rigidbody.shapeType);
        switch (type)
        {
        case (BM(ST_SPHERE) | BM(ST_PLANE)):
            sphereToPlane(rb, &i.rigidbody, &collisionData[*collisionSize]);
            break;
        case (BM(ST_BOX) | BM(ST_PLANE)): {
            boxToPlane(rb, &i.rigidbody, &collisionData[*collisionSize]);
            break;
        }
        default:
            collisionData[*collisionSize].count = 0;
            break;
        }

        if (collisionData[*collisionSize].count != 0) {
            collisions[*collisionSize] = &i.rigidbody;
            ++(*collisionSize);
        }
    }
}

//BUG main nest
void ps::pp::basicResolver(ps::pp::Engine* e, ps::pp::Rigidbody* rb) {
    for (int i = 0; i < e->collision_props.size; i++)
    {
        auto rb2 = e->collision_props.collisions[i];
        auto data = e->collision_props.collisionData[i];
        auto normal = (~rb->M)(data.normal);


        float rho = 0.5;

        for (int ii = 0; ii < data.count; ii++)
        {
            auto point = (~rb->M)(data.pointsOfContact[ii]);

            auto np = kln::project(normal, point);

            auto comV2 = kln::point((point & rb->B).p0_);
            //auto com = 0.5f * (point * rb->B - rb->B * point);
            // auto com2 = 0.5f * (point * !np - !np * point);
            auto com2V2 = kln::point((point & !np).p0_);

            //auto Vm = point & com;
            // auto j = -(1 + rho) * ( point &(Vm | np) / ((point & com2) | np));

            auto Vm = point & comV2;

            auto j = -(1 + rho) * ((Vm | np) / ((point & com2V2) | np));
            j /= (float)data.count;

            auto vd = (kln::line)(j * (kln::motor)!np);
            rb->B = rb->B + vd;

        }

    }

}


namespace ps::pp {

    Engine::Engine(SimulateFunc sF, ColliderFunc cF, ResolverFunc rF, InterpolationFunc iF): simulate(sF), collide(cF), resolve(rF), interpolation(iF) {

    }

    void Engine::step(ps::WordState* _in, ps::WordState* _out, float _dT) {
        this->in = _in;
        this->out = _out;
        this->dT = _dT;

        *out = *in;

#ifndef NDEBUG
        if (debug_data.stop) return;

        debug_data.collisions.clear();
        debug_data.collisionData.clear();

        //if(debug_data.oneStep)
#endif

        for (int i = 0; i < in->simulatedObjects.size(); i++) {
            this->interpolation(this, &out->simulatedObjects[i].rigidbody);
            
#ifndef NDEBUG
            debug_data.collisionData.insert(debug_data.collisionData.end(), collision_props.collisionData, collision_props.collisionData + collision_props.size);
            for (int j = 0; j < collision_props.size; j++)
            {
                std::string name = "Id:" + std::to_string(out->simulatedObjects[j].id) + "<" + shapeName[out->simulatedObjects[j].rigidbody.shapeType] + "> and ";//+ shapeName[collision_props.collisions[i]->shapeType];
                debug_data.collisions.push_back(name);
            }            
#endif
        }

        // for (int i = 0; i < in->simulatedObjects.size(); i++) {
        //     this->collide(this, &out->simulatedObjects[i].rigidbody);
        //     this->resolve(this, &out->simulatedObjects[i].rigidbody);
        // }
    }
}