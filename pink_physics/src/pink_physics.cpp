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
        e->resolve(e, rb);
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
    e->resolve(e, rb);
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

    kln::line F = G;//+Damp;

    rb->dM = -0.5f * (rb->M * (kln::motor)(rb->B));

    auto I = *((kln::line*)rb->shape);

    auto I_B = (!rb->B).mult(I);
    auto helper = !~((F - klnTestCross(I_B, rb->B)).div(I));

    rb->dB = (helper);
}

void ps::pp::basicCollider(ps::pp::Engine* e, ps::pp::Rigidbody* rb) {
    auto& staticObjects = e->out->staticObjects;
    auto& dynamicObjects = e->out->simulatedObjects;

    auto collisions = e->collision_props.collisions;
    auto collisionData = e->collision_props.collisionData;
    auto& collisionSize = e->collision_props.size;
    auto collisionMaxSize = Engine::maxNumber;

    collisionSize = 0;


    for (int n = 0; n < staticObjects.size(); n++)
    {
        auto& i = staticObjects[n];
        if (collisionSize >= collisionMaxSize) break;
        if (rb == &i.rigidbody) continue;

        auto type = BM(rb->shapeType) | BM(i.rigidbody.shapeType);
        switch (type)
        {
        case (BM(ST_SPHERE) | BM(ST_PLANE)): {
            sphereToPlane(rb, &i.rigidbody, &collisionData[collisionSize]);
            break;
        }
        case (BM(ST_BOX) | BM(ST_PLANE)): {
            boxToPlane(rb, &i.rigidbody, &collisionData[collisionSize]);
            break;
        }
        case (BM(ST_SPHERE)): {
            sphereToSphere(rb, &i.rigidbody, &collisionData[collisionSize]);
            break;
        }
        default:
            collisionData[collisionSize].count = 0;
            break;
        }

        if (collisionData[collisionSize].count != 0) {
            collisionData[collisionSize].rb = &(staticObjects[n].rigidbody);

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

    for (int n = 0; n < dynamicObjects.size(); n++)
    {
        auto& i = dynamicObjects[n];
        if (collisionSize >= collisionMaxSize) break;
        if (rb == &i.rigidbody) continue;

        auto type = BM(rb->shapeType) | BM(i.rigidbody.shapeType);
        switch (type)
        {
        case (BM(ST_SPHERE) | BM(ST_PLANE)): {
            sphereToPlane(rb, &i.rigidbody, &collisionData[collisionSize]);
            break;
        }
        case (BM(ST_BOX) | BM(ST_PLANE)): {
            boxToPlane(rb, &i.rigidbody, &collisionData[collisionSize]);
            break;
        }
        case (BM(ST_SPHERE)): {
            sphereToSphere(rb, &i.rigidbody, &collisionData[collisionSize]);
            break;
        }
        default:
            collisionData[collisionSize].count = 0;
            break;
        }

        if (collisionData[collisionSize].count != 0) {
            collisionData[collisionSize].rb = &(dynamicObjects[n].rigidbody);

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
void ps::pp::basicResolver(ps::pp::Engine* e, ps::pp::Rigidbody* rb) {
    float rho = 0.5f;

    auto& I_p = *((kln::line*)rb->shape);
    auto B_p = (rb->M)(rb->B);

    for (int i = 0; i < e->collision_props.size; i++)
    {
        auto& data = e->collision_props.collisionData[i];
        auto rb2 = data.rb;

        auto& I_m = *((kln::line*)rb2->shape);
        auto B_m = (rb2->M)(rb2->B);

        auto normal = (data.normal);

        for (int ii = 0; ii < data.count; ii++)
        {
            auto Q = data.pointsOfContact[ii];

            auto N = normal | Q;

            auto I_pN = (rb->M)(!~(((~rb->M)(N)).div(I_p))) * rb->bodyType;
            auto I_mN = (rb2->M)(!~(((~rb2->M)(N)).div(I_m))) * rb2->bodyType;

            //IMPULSE
            auto QxB = klnTestCross(Q, B_p - B_m);
            auto QxI = klnTestCross(Q, I_pN + I_mN);

            auto localB = Q & QxB;

            auto num = (Q & QxB) | ~N;
            auto den = (Q & QxI) | ~N;

            auto j = -(1 + rho) * num / den;
            j /= (float)data.count;

            auto I_pNb = j * (~rb->M)(I_pN);
            auto I_mNb = j * (~rb2->M)(I_mN);

            rb->B += I_pNb;
            rb2->B -= I_mNb;

            // FRICTION
            auto T = (kln::project(Q & QxB, normal) | Q) | Q;
            if (eCmp(T.norm(), 0.0f)) return;
            
            T.normalize();

            auto num_T = ((Q & QxB) | ~T);
            if (eCmp(num_T, 0.0f)) return;

            auto I_pT = (rb->M)(!~(((~rb->M)(T)).div(I_p))) * rb->bodyType;
            auto I_mT = (rb2->M)(!~(((~rb2->M)(T)).div(I_m))) * rb2->bodyType;

            auto QxIT = klnTestCross(Q, I_pT + I_mT);

            auto den_T = ((Q & QxIT) | ~T);

            auto jt = num_T / den_T;

            jt /= (float)data.count;

            if (ps::pp::eCmp(jt, 0.0f)) return;

            float f = 0.2f;

            if (jt > j * f) {
                jt = j * f;
            }
            else if (jt < -j * f) {
                jt = -j * f;
            }

            auto I_pTb = jt * (~rb->M)(I_pT);
            auto I_mTb = jt * (~rb2->M)(I_mT);


            //TODO fix for SIMULATED v SIMULATED collisions / or disable for them ()  
            rb->B += I_pTb;
            rb2->B -= I_mTb;
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