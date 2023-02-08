#include "pink_physics.hpp"
#include "pink_structs.hpp"
#include "pink_physics_colliders.hpp"
#include "GLFW/glfw3.h"

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
        rb->move();

        e->collide(e, rb, e->simulatedRbs);
        e->resolve(e);

        e->collide(e, rb, e->constrainingRbs);
        ps::pp::solidResolver(e);
    }
    rb->F *= 0;
}

void ps::pp::verletIntegration(ps::pp::Engine* e, ps::pp::Rigidbody* rb) {
    float step = e->dT * e->interpolation_props.step;

    auto old_dM = rb->dM;
    e->simulate(rb);
    rb->F *= 0;
    rb->M = rb->M + (rb->dM + old_dM) * (step / 2.f);
    rb->B = rb->B + (step * rb->dB);
    rb->M.normalize();
    rb->B.grade2();

    rb->move();

    for (int i = 0; i < 4; i++) {
        e->collide(e, rb, e->simulatedRbs);
        e->resolve(e);
        e->collide(e, rb, e->constrainingRbs);
        ps::pp::solidResolver(e);  
    }
    
    // for (int i = 0; i < 1; i++) {
    // }
}

void ps::pp::basicSimulate(ps::pp::Rigidbody* rb) {
    kln::line G(0.f, -9.81f, 0.f, 0.f, 0.f, 0.f);
    G = !~((~rb->M)(G));
    kln::line Damp = !~(-0.35f * rb->B);

    rb->F += G + Damp;

    rb->dM = -0.5f * (rb->M * (kln::motor)(rb->B));

    auto I = rb->shape->inertia;

    auto I_B = (!rb->B).mult(I);

    rb->dB = !~((rb->F - klnTestCross(I_B, rb->B)).div(I));
}

// void ps::pp::carSimulate(Rigidbody* rb) {
//     auto car = (Car*)rb;

//     kln::line G(0.f, -9.81f, 0.f, 0.f, 0.f, 0.f);
//     G = !~((~rb->M)(G));
//     kln::line Damp = !~(-0.35f * rb->B);

//     rb->F += G + Damp;

//     rb->dM = -0.5f * (rb->M * (kln::motor)(rb->B));

//     auto I = rb->shape->inertia;

//     auto I_B = (!rb->B).mult(I);

//     rb->dB = !~((rb->F - klnTestCross(I_B, rb->B)).div(I));
//     rb->F *= 0;
// }

void ps::pp::basicCollider(ps::pp::Engine* e, ps::pp::Rigidbody* rb) {
    auto& staticObjects = e->constrainingRbs;
    auto& dynamicObjects = e->simulatedRbs;

    auto& collisionData = e->collision_props.collisionData;
    auto& collisionSize = e->collision_props.size;
    auto& collisionMaxSize = Engine::maxNumber;

    collisionSize = 0;

    for (int n = 0; n < dynamicObjects.size(); n++)
    {
        auto& i = dynamicObjects[n];
        if (rb == &i->rigidbody) continue;

        if (collide(rb, &i->rigidbody, &collisionData[collisionSize])) {
#ifndef NDEBUG
            for (int di = 0; di < collisionData[collisionSize].count; di++)
            {
                auto p = collisionData[collisionSize].pointsOfContact[di];
                nvmath::mat4f s = nvmath::scale_mat4(nvmath::vec3f(0.2f, 0.2f, 0.2f));
                nvmath::mat4f t = nvmath::translation_mat4(nvmath::vec3f(p.x(), p.y(), p.z()));
                //e->out->points.push_back(t * s);
            }
#endif
            ++(collisionSize);

        }
    }

    for (int n = 0; n < staticObjects.size(); n++)
    {
        auto& i = staticObjects[n];
        if (collisionSize >= collisionMaxSize) break;
        if (collide(rb, &i->rigidbody, &collisionData[collisionSize])) {
#ifndef NDEBUG
            for (int di = 0; di < collisionData[collisionSize].count; di++)
            {
                auto p = collisionData[collisionSize].pointsOfContact[di];
                nvmath::mat4f s = nvmath::scale_mat4(nvmath::vec3f(0.2f, 0.2f, 0.2f));
                nvmath::mat4f t = nvmath::translation_mat4(nvmath::vec3f(p.x(), p.y(), p.z()));
                // e->out->points.push_back(t * s);
            }
#endif
            ++(collisionSize);
        }
    }
}

void ps::pp::vecCollider(ps::pp::Engine* e, ps::pp::Rigidbody* rb, std::vector<ps::Object*>& vec) {
    auto& collisionData = e->collision_props.collisionData;
    auto& collisionSize = e->collision_props.size;
    auto& collisionMaxSize = Engine::maxNumber;

    collisionSize = 0;

    for (int n = 0; n < vec.size(); n++)
    {
        auto i = vec[n];
        if (rb == &i->rigidbody) continue;

        if (collide(rb, &i->rigidbody, &collisionData[collisionSize])) {
#ifndef NDEBUG
            for (int di = 0; di < collisionData[collisionSize].count; di++)
            {
                auto p = collisionData[collisionSize].pointsOfContact[di];
                nvmath::mat4f s = nvmath::scale_mat4(nvmath::vec3f(0.2f, 0.2f, 0.2f));
                nvmath::mat4f t = nvmath::translation_mat4(nvmath::vec3f(p.x(), p.y(), p.z()));
                // e->out->points.push_back(t * s);
            }
#endif
            ++(collisionSize);

        }
    }
}

void ps::pp::solidResolver(ps::pp::Engine* e) {
    float rho = 0.0f;


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

            int size = 0;
            auto QxB = klnTestCross(Q, B_p - B_m);
            auto localB = (Q & QxB);

            auto num = localB | N;

            auto j = -(1 + rho) * num;
            j /= (float)data.count;
            if (isnan(j)) continue;
            if (j > 0.f) continue;

            rb_p->apply(rb_p, !~((~rb_p->M)(N)), j);
            rb_m->apply(rb_m, !~((~rb_m->M)(N)), -j);

            // FRICTION
            auto p = (localB | Q).normalized();
            p -= normal;
            p.zeroE0();
            if (eCmp(p.norm(), 0.0f)) continue;
            p.normalize();
            p = kln::project(p, Q);
            auto T = p | Q;

            T.normalize();

            auto num_T = ((Q & QxB) | T);
            if (eCmp(num_T, 0.0f) || isnan(T.norm())) continue;

            auto jt = num_T;

            jt /= (float)data.count;

            if (ps::pp::eCmp(jt, 0.0f) || isnan(jt)) continue;

            float f = 0.05f;

            if (jt > j * f) {
                jt = j * f;
            }
            else if (jt < -j * f) {
                jt = -j * f;
            }

            rb_p->apply(rb_p, !~((~rb_p->M)(T)), jt);
            rb_m->apply(rb_m, !~((~rb_m->M)(T)), -jt);
        }
    }
}

void ps::pp::basicResolver(ps::pp::Engine* e) {
    // Coefficient of restitution
    float rho = 0.8f;


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

            int size = 0;
            auto QxB = klnTestCross(Q, B_p - B_m);
            auto localB = (Q & QxB);

            // if rb.
            // for (int iii = 0; iii < 4; iii++) {

            //     auto c = constrains[iii];
            //     auto num = localB | c.C;
            //     if (num < 0.f) continue;

            //     auto QxI = klnTestCross(Q, I_pN + I_mN);
            //     auto den = (Q & QxI) | c.C;

            //     if (den == 0.f) continue;


            // }
            //IMPULSE


            // Local (at Q) relative rate along meet line N ?
            auto num = localB | N;

            // check if bodies are already moving away

            // Inertia of rb_p (at Q) along N (now join line ?) 
            auto I_pN = (rb_p->M)(!~(((~rb_p->M)(N)).div(I_p)));
            // Inertia of rb_m (at Q) along N (now join line ?) 
            auto I_mN = (rb_m->M)(!~(((~rb_m->M)(N)).div(I_m)));

            // Local inertia sum
            auto QxI = klnTestCross(Q, I_pN + I_mN);
            auto den = (Q & QxI) | N;


            auto j = -(1 + rho) * num / den;
            j /= (float)data.count;
            if (isnan(j)) continue;
            if (j > 0.f) continue;

            rb_p->apply(rb_p, (~rb_p->M)(I_pN), j);
            rb_m->apply(rb_m, (~rb_m->M)(I_mN), -j);

            // FRICTION
            auto p = (localB | Q).normalized();
            p -= normal;
            p.zeroE0();
            if (eCmp(p.norm(), 0.0f)) continue;
            p.normalize();
            p = kln::project(p, Q);
            auto T = p | Q;

            T.normalize();

            auto num_T = ((Q & QxB) | T);
            if (eCmp(num_T, 0.0f) || isnan(T.norm())) continue;

            auto I_pT = (rb_p->M)(!~(((~rb_p->M)(T)).div(I_p)));
            auto I_mT = (rb_m->M)(!~(((~rb_m->M)(T)).div(I_m)));

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

            rb_p->apply(rb_p, (~rb_p->M)(I_pT), jt);
            rb_m->apply(rb_m, (~rb_m->M)(I_mT), -jt);
        }
    }

}


namespace ps::pp {
    Rigidbody::~Rigidbody() {
        
    }
    Rigidbody::Rigidbody(kln::motor M, kln::line B, BodyType bt, BaseShape* shape): M(M),B(B),bodyType(bt),shape(shape) {
        dM = kln::uMotor();
        dB = kln::line();

        kln::line F = kln::line(0, 0, 0, 0, 0, 0);
        move();
    }

    void Rigidbody::move() {
        shape->move(M);
    }


    void Engine::registerObject(Object* obj) {

        allBodies.insert(std::upper_bound(allBodies.begin(), allBodies.end(), obj, ps::ObjectIDCmp()), obj);
        if (obj->rigidbody.bodyType == BT_DYNAMIC) {
            simulatedRbs.insert(std::upper_bound(simulatedRbs.begin(), simulatedRbs.end(), obj, ps::ObjectIDCmp()), obj);
        }
        else {
            constrainingRbs.insert(std::upper_bound(constrainingRbs.begin(), constrainingRbs.end(), obj, ps::ObjectIDCmp()), obj);
        }
        for (int i = 0; i < 4; i++) {//TODO yuck
            obj->motors[i] = obj->rigidbody.M;
        }
    }

    void Engine::deregisterObject(Object* obj) {
        allBodies.erase(std::upper_bound(allBodies.begin(), allBodies.end(), obj, ps::ObjectIDCmp()));
        if (obj->rigidbody.bodyType == BT_DYNAMIC) {
            simulatedRbs.erase(std::upper_bound(simulatedRbs.begin(), simulatedRbs.end(), obj, ps::ObjectIDCmp()));
        }
        else {
            constrainingRbs.erase(std::upper_bound(constrainingRbs.begin(), constrainingRbs.end(), obj, ps::ObjectIDCmp()));
        }
    }

    void checkJoin(Engine* e, Joint* j) {
        auto& parent = j->parent;
        auto& child = j->child;
    }

    void enforceJoint(Engine* e, Joint* j) {
        auto& parent = *j->parent;
        auto& child = *j->child;

        auto joinToWorld = parent.M;

        j->valid = true;

        auto line = joinToWorld(j->constraint);

        auto center = child.shape->center;
        auto newCenter = kln::project(center, line).normalized();

        auto minAtt = joinToWorld(j->Att[0]);
        auto maxAtt = joinToWorld(j->Att[1]);

        if ((maxAtt | line).e0() < (minAtt | line).e0()) {
            maxAtt = joinToWorld(j->Att[0]);
            minAtt = joinToWorld(j->Att[1]);
        }

        auto vmin = (minAtt | line).e0();
        auto vmax = (maxAtt | line).e0();


        auto curr = (newCenter | line).e0();

        if (curr < vmin) {
            newCenter = minAtt;
            j->valid = false;
        }
        else if (curr > vmax) {
            newCenter = maxAtt;
            j->valid = false;
        }
        kln::motor correction = kln::sqrt(newCenter * center);
        child.M = child.M * correction;
    }

    void applyImpulseNormal(Rigidbody* rb, kln::line dir, float a) {
        rb->B += dir * a;
    }

    void applyImpulseStatic(Rigidbody* rb, kln::line dir, float a) {
    }

    void applyImpulseWheel(Rigidbody* rb, kln::line dir, float a) {
        rb->B += dir * a;
    }

    Engine::Engine(SimulateFunc sF, ColliderFunc cF, ResolverFunc rF, IntegrationFunc iF): simulate(sF), collide(cF), resolve(rF), integrator(iF) {

    }

    void Engine::applySprings() {
        for (int i = 0; i < springs.size(); i++)
        {
            auto& s = springs[i];
            auto& b1 = *s.rb1;
            auto& b2 = *s.rb2;

            auto c1 = s.rb1atch(b1.shape->center);
            auto c2 = s.rb2atch(b2.shape->center);

            auto line1 = c1 & c2;
            // auto line2 = c2 & c1;

            auto x = line1.norm() - s.restingLength;

            line1.normalize();
            // line2.normalize();

            // b1.rigidbody.F -= ((~b1.rigidbody.M)((s.k * -x) * line1));
            // b2.rigidbody.F -= ((~b2.rigidbody.M)((s.k * -x) * line2));


            b1.B -= !~((~b1.M)(((s.k * -x) / b1.shape->mass) * line1));
            b2.B += !~((~b2.M)(((s.k * -x) / b2.shape->mass) * line1));
        }
    }

    void Engine::enforceJoints() {
        for (int i = 0; i < joins.size();i++) {
            enforceJoint(this, &joins[i]);
        }
    }

    void Engine::step(int writePos, float _dT) {
        this->dT = _dT;


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

        // out->points.clear();
        //if(debug_data.oneStep)
#endif
        applySprings();


        for (int i = 0; i < simulatedRbs.size(); i++) {
            this->integrator(this, &simulatedRbs[i]->rigidbody);

#ifndef NDEBUG
            debug_data.collisionData.insert(debug_data.collisionData.end(), collision_props.collisionData, collision_props.collisionData + collision_props.size);
            for (int j = 0; j < collision_props.size; j++)
            {
                //std::string name = "Id:" + std::to_string(out->simulatedObjects[j].id) + "<" + shapeName[out->simulatedObjects[j].rigidbody.shapeType] + "> and ";//+ shapeName[collision_props.collisions[i]->shapeType];
                //debug_data.collisions.push_back(name);
            }
#endif
        }

        enforceJoints();

        for (int i = 0; i < simulatedRbs.size(); i++) {
            simulatedRbs[i]->motors[writePos] = simulatedRbs[i]->rigidbody.M;
        }

    }

    void Engine::run(int* inc,float* lastTime, bool* kill) {
        static float limitFPS = 1.0f / 15.0f;

        static float dT = 1000 * limitFPS;

        float timer = *lastTime;
        float deltaTime = 0, nowTime = 0;
        int next;

        while (!*kill)
        {
            nowTime = (float)glfwGetTime();
            deltaTime += (nowTime - *lastTime) / limitFPS;

            if (deltaTime >= 1.0) {
                next = (*inc + 1) % 4; //TODO
                step(next, (nowTime - *lastTime));
                *lastTime = nowTime;
                *inc = next;
            }
        }
    }
}