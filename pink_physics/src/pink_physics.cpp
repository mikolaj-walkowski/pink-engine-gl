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

// RK baby lets gooo
void ps::pp::eulerIntegration(ps::pp::Engine* e, ps::pp::Rigidbody* rb) {
    int iterations = e->interpolation_props.iterations;
    float step = (e->dT * e->interpolation_props.step) / (float)iterations;
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

void ps::pp::verletIntegration(ps::pp::Engine* e, ps::pp::Rigidbody* rb) {
    float step = e->dT * e->interpolation_props.step;

    auto old_dM = rb->dM;
    e->simulate(rb);
    rb->M = rb->M + (rb->dM + old_dM) * (step / 2.f);
    rb->B = rb->B + (step * rb->dB);

    e->collide(e, rb);
    e->resolve(e, rb);
    rb->M.normalize();
}

// void ps::pp::RK4(ps::pp::Engine* e, ps::pp::Rigidbody* rb) {
//     e->simulate(rb);
//     auto k1dM = rb->dM;
//     auto k1dB = rb->dB;

    
    
//     e->simulate(rb);
//     auto k1dM = rb->dM;
//     auto k1dB = rb->dB;


    
// }

// TODO Upgrade Box collision
void ps::pp::basicSimulate(ps::pp::Rigidbody* rb) {
    kln::line G = (~(rb->M))(kln::ideal_line(0.f, 9.81f, 0.f));
    kln::line Damp = (-0.35f * rb->B);

    kln::line F = kln::line() + G;// +Damp; // + wszystkie siły


    rb->dM = -0.5f * (rb->M * ((kln::motor)(rb->B)));


    auto I = ((ps::pp::Plane*)rb->shape)->inertia;
    auto I_1 = I;

    auto comBIB = kln::line(rb->B * I.mult(rb->B) - I.mult(rb->B) * rb->B);

    rb->dB = ~I_1.mult(comBIB + F);
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
            
#ifndef NDEBUG
            for (int di = 0; di < collisionData[*collisionSize].count; di++)
            {
                auto p = collisionData[*collisionSize].pointsOfContact[di];
                nvmath::mat4f s = nvmath::scale_mat4(nvmath::vec3f(0.2f, 0.2f, 0.2f));
                nvmath::mat4f t = nvmath::translation_mat4(nvmath::vec3f(p.x(), p.y(), p.z()));
                e->out->points.push_back(t*s);
            }       
#endif
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
            auto Q = (~rb->M)(data.pointsOfContact[ii]);

            auto B_p = rb->B;
            auto B_m = kln::line(0, 0, 0, 0, 0, 0);//TODO 2nd rigidbody

            auto N = kln::project(normal, Q).normalized();

            auto aaa = (((ps::pp::Plane*)rb->shape)->inertia);

            auto I_pN = !(((ps::pp::Plane*)rb->shape)->inertia).mult(N);
            auto I_mN = kln::line(0, 0, 0, 0, 0, 0);//(~(2nd rigidbody->shape)->inertia).mult(N);

            auto QxB = kln::point((Q & (B_p - B_m)).p0_);
            auto QxI = kln::point((Q & (I_pN - I_mN)).p0_);

            auto top = Q & QxB | ~N;
            auto bottom = Q & QxI | ~N;

            auto j = -(1 + rho) * (top / bottom);

            //j /= (float)data.count;

            rb->B = rb->B + j * I_pN;
            //2nd rigidbody->B = 2nd rigidbody->B - j* I_mN
        }
    }

}


namespace ps::pp {

    Engine::Engine(SimulateFunc sF, ColliderFunc cF, ResolverFunc rF, IntegrationFunc iF): simulate(sF), collide(cF), resolve(rF), integrator(iF) {

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

        out->points.clear();
        //if(debug_data.oneStep)
#endif

        for (int i = 0; i < in->simulatedObjects.size(); i++) {
            this->integrator(this, &out->simulatedObjects[i].rigidbody);

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