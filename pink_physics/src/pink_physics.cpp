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

void ps::pp::print(const char* name, kln::point point) {
    std::string n = name + std::string(" : %fe013, %fe021,%fe032,%fe123\n");
    printf(n.c_str(), point.e013(), point.e021(), point.e032(), point.e123());
}

kln::point klnTestCross(kln::point a, kln::line b) {
    kln::point p;
    float out[4] = {
        0.f, // 14? e123
        b.e31() * a.e021() - b.e12() * a.e013() + b.e01() * a.e123(), //13 e032
        -b.e23() * a.e021() + b.e12() * a.e032() + b.e02() * a.e123() ,  //12 e013
        b.e23() * a.e013() - b.e31() * a.e032() + b.e03() * a.e123()  //11 e021
    };
    p.load(out);
    return p;
}


kln::line klnTestCross(kln::line a, kln::line b) {
    //// Direct initialization from components. A more common way of creating a
        /// motor is to take a product between a rotor and a translator.
        /// The arguments coorespond to the multivector
        /// $a + b\mathbf{e}_{23} + c\mathbf{e}_{31} + d\mathbf{e}_{12} +\
        /// e\mathbf{e}_{01} + f\mathbf{e}_{02} + g\mathbf{e}_{03} +\
        /// h\mathbf{e}_{0123}$.
    kln::line out = kln::motor(
        -b.e12() * a.e12() - b.e31() * a.e31() - b.e23() * a.e23(),//scalar
        b.e31() * a.e12() - b.e12() * a.e31(),//e23
        -b.e23() * a.e12() + b.e12() * a.e23(),//e31
        b.e23() * a.e31() - b.e31() * a.e23(),//e12
        -b.e12() * a.e02() + b.e31() * a.e03() + b.e02() * a.e12() - b.e03() * a.e31(),//e01
        b.e12() * a.e01() - b.e23() * a.e03() - b.e01() * a.e12() + b.e03() * a.e23(),//e02
        b.e31() * a.e01() + b.e23() * a.e02() + b.e01() * a.e31() - b.e02() * a.e23(),//e02
        b.e23() * a.e01() + b.e31() * a.e02() + b.e12() * a.e03() + b.e03() * a.e12() + b.e02() * a.e31() + b.e01() * a.e23() // e0123
    );
    return out;
}

void ps::pp::eulerIntegration(ps::pp::Engine* e, ps::pp::Rigidbody* rb) {
    int iterations = e->interpolation_props.iterations;
    float step = (e->dT * e->interpolation_props.step) / (float)iterations;
    for (int i = 0; i < iterations; i++)
    {

        e->simulate(rb);
        rb->M = rb->M + (step * rb->dM);
        rb->B = rb->B + (step * rb->dB);
        rb->M.normalize();
        rb->B.grade2();
        e->collide(e, rb);
        e->resolve(e);
    }
}

void ps::pp::verletIntegration(ps::pp::Engine* e, ps::pp::Rigidbody* rb) {
    float step = e->dT * e->interpolation_props.step;

    auto old_dM = rb->dM;
    e->simulate(rb);
    rb->M = rb->M + (rb->dM + old_dM) * (step / 2.f);
    rb->B = rb->B + (step * rb->dB);
    rb->M.normalize();
    rb->B.grade2();

    e->collide(e, rb);
    e->resolve(e);
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
    kln::line G(0.f, -9.81f, 0.f, 0.f, 0.f, 0.f);
    G = !~((~rb->M)(G));
    kln::line Damp = !~(-0.35f * rb->B);

    kln::line F = G + Damp;

    rb->dM = -0.5f * (rb->M * (kln::motor)(rb->B));

    auto I = *((kln::line*)rb->shape);

    auto I_B = (!rb->B).mult(I);
    auto helper = !~((F - klnTestCross(I_B, rb->B)).div(I));

    rb->dB = (helper);
}


void ps::pp::basicCollider(ps::pp::Engine* e, ps::pp::Rigidbody* rb) {
    auto& staticObjects = e->out->staticObjects;
    auto& dynamicObjects = e->out->simulatedObjects;

    auto& collisionData = e->collision_props.collisionData;
    auto& collisionSize = e->collision_props.size;
    auto& collisionMaxSize = Engine::maxNumber;

    collisionSize = 0;


    for (int n = 0; n < staticObjects.size(); n++)
    {
        auto& i = staticObjects[n];
        if (collisionSize >= collisionMaxSize) break;
        if (collide(rb, &i.rigidbody, &collisionData[collisionSize])) {
#ifndef NDEBUG
            for (int di = 0; di < collisionData[collisionSize].count; di++)
            {
                auto p = collisionData[collisionSize].pointsOfContact[di];
                nvmath::mat4f s = nvmath::scale_mat4(nvmath::vec3f(0.2f, 0.2f, 0.2f));
                nvmath::mat4f t = nvmath::translation_mat4(nvmath::vec3f(p.x(), p.y(), p.z()));
                // auto x = p.x();
                // auto y = p.y();
                // auto z = p.z();
                // printf("x %f, y %f, z %f\n", x, y, z);
                e->out->points.push_back(t * s);
            }
#endif
            ++(collisionSize);
        }
    }

    for (int n = 0; n < dynamicObjects.size(); n++)
    {
        auto& i = dynamicObjects[n];
        if (collisionSize >= collisionMaxSize) break;
        if (rb == &i.rigidbody) continue;

        if (collide(rb, &i.rigidbody, &collisionData[collisionSize])) {
#ifndef NDEBUG
            for (int di = 0; di < collisionData[collisionSize].count; di++)
            {
                auto p = collisionData[collisionSize].pointsOfContact[di];
                nvmath::mat4f s = nvmath::scale_mat4(nvmath::vec3f(0.2f, 0.2f, 0.2f));
                nvmath::mat4f t = nvmath::translation_mat4(nvmath::vec3f(p.x(), p.y(), p.z()));
                e->out->points.push_back(t * s);
            }
#endif
            ++(collisionSize);
        }
    }
}

//BUG main nest
void ps::pp::basicResolver(ps::pp::Engine* e) {
    // Coefficient of restitution
    float rho = 0.5f;


    for (int i = 0; i < e->collision_props.size; i++)
    {
        // Collision data 
        auto& data = e->collision_props.collisionData[i];
        // Colliding body
        auto rb_p = data.rb1;
        auto rb_m = data.rb2;

        // rb_p inertia map
        const auto& I_p = rb_p->shape->inertia;
        // rb_m inertia map
        const auto& I_m = rb_m->shape->inertia;

        // Collision plane normal
        auto normal = (data.normal);

        for (int ii = 0; ii < data.count; ii++)
        {
            // Point of contact
            auto Q = data.pointsOfContact[ii];
            // Normal meet line ?
            auto N = normal | Q;

            // World rb_p rate
            auto B_p = (rb_p->M)(rb_p->B);
            // World rb_m rate
            auto B_m = (rb_m->M)(rb_m->B);

            //IMPULSE
            // Local (at Q) relative Rate
            auto QxB = klnTestCross(Q, B_p - B_m);

            // Local (at Q) relative rate along meet line N ?
            auto num = (Q & QxB) | N;

            // check if bodies are already moving away
            if (num < 0.f) continue;

            // Inertia of rb_p (at Q) along N (now join line ?) 
            auto I_pN = (rb_p->M)(!~(((~rb_p->M)(N)).div(I_p))) * rb_p->bodyType;
            // Inertia of rb_m (at Q) along N (now join line ?) 
            auto I_mN = (rb_m->M)(!~(((~rb_m->M)(N)).div(I_m))) * rb_m->bodyType;
            
            // Local inertia sum
            auto QxI = klnTestCross(Q, I_pN + I_mN);
            auto den = (Q & QxI) | N;

            auto localB = Q & QxB;
        
            auto j = -(1 + rho) * num / den;
            j /= (float)data.count;
            if (isnan(j)) continue;
            auto I_pNb = j * (~rb_p->M)(I_pN);
            auto I_mNb = j * (~rb_m->M)(I_mN);

            rb_p->B += I_pNb;
            rb_m->B -= I_mNb;

            // FRICTION
            auto p = (localB | Q).normalized();
            auto h = p;
            p -= normal;
            p.zeroE0();
            if (eCmp(p.norm(), 0.0f)) continue;
            p.normalize();
            p = kln::project(p, Q);
            auto T = p | Q;

            T.normalize();

            auto num_T = ((Q & QxB) | T);
            if (eCmp(num_T, 0.0f) || isnan(T.norm())) continue;

            auto I_pT = (rb_p->M)(!~(((~rb_p->M)(T)).div(I_p))) * rb_p->bodyType;
            auto I_mT = (rb_m->M)(!~(((~rb_m->M)(T)).div(I_m))) * rb_m->bodyType;

            auto QxIT = klnTestCross(Q, I_pT + I_mT);

            auto den_T = ((Q & QxIT) | T);

            auto jt = num_T / den_T;

            jt /= (float)data.count;

            if (ps::pp::eCmp(jt, 0.0f) || isnan(jt)) continue;

            float f = 0.05f;

            if (jt > j * f) {
                jt = j * f;
            }
            else if (jt < -j * f) {
                jt = -j * f;
            }

            auto I_pTb = jt * (~rb_p->M)(I_pT);
            auto I_mTb = jt * (~rb_m->M)(I_mT);


            //TODO fix for SIMULATED v SIMULATED collisions / or disable for them ()  
            rb_p->B += I_pTb;
            rb_m->B -= I_mTb;
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
        if (debug_data.stepped) {
            this->dT = debug_data.dT;
            debug_data.stop = false;
            if (debug_data.step <= 0) return;
            debug_data.step--;
        }
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
                //std::string name = "Id:" + std::to_string(out->simulatedObjects[j].id) + "<" + shapeName[out->simulatedObjects[j].rigidbody.shapeType] + "> and ";//+ shapeName[collision_props.collisions[i]->shapeType];
                //debug_data.collisions.push_back(name);
            }
#endif
        }

        // for (int i = 0; i < in->simulatedObjects.size(); i++) {
        //     this->collide(this, &out->simulatedObjects[i].rigidbody);
        //     this->resolve(this, &out->simulatedObjects[i].rigidbody);
        // }
    }
}