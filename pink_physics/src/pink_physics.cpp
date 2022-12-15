#include "pink_physics.hpp"
#include "pink_structs.hpp"
#include "pink_physics_colliders.hpp"

kln::line::line(motor l) {
    p1_ = l.p1_;
    p2_ = l.p2_;
}


void ps::pp::eulerInterpolation(ps::pp::Engine* e, ps::pp::Rigidbody* rb) {
    int iterations = e->interpolation_props.iterations;
    float step = e->interpolation_props.step;
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

    kln::line F = G; // + wszystkie siły

    // BUG  using P-something dual instead of Hodge may backfire
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

        auto type = BM(rb->shapeType) & BM(i.rigidbody.shapeType);
        switch (type)
        {
        case BM(ST_SPHERE) & BM(ST_PLANE):
            sphereToPlane(rb, &i.rigidbody, &collisionData[*collisionSize]);
            break;

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

            auto np = kln::project(normal,point);

            auto comV2 =kln::point((point & rb->B).p0_);
            //auto com = 0.5f * (point * rb->B - rb->B * point);
            // auto com2 = 0.5f * (point * !np - !np * point);
            auto com2V2 = kln::point((point & !np).p0_);

            //auto Vm = point & com;
            // auto j = -(1 + rho) * ( point &(Vm | np) / ((point & com2) | np));

            auto Vm = point & comV2;

            auto j = -(1 + rho) * ((Vm | np) / ((point & com2V2) | np));

            auto vd = (kln::line)(j * (kln::motor)!np);
            rb->B = rb->B + vd;

        }

    }

}

nvmath::mat4f ps::interpolate(ps::Object* a, ps::Object* b, float t) {
    if (a->interpolation_catche.obj != b->id) {
        a->interpolation_catche.log = kln::log(b->rigidbody.M * ~a->rigidbody.M);
        a->interpolation_catche.obj = b->id;
    }

    kln::motor m = kln::exp(a->interpolation_catche.log * t) * a->rigidbody.M;
    return nvmath::mat4f(m.as_mat4x4().data);
}



namespace ps::pp {

    Engine::Engine(SimulateFunc sF, ColliderFunc cF, ResolverFunc rF, InterpolationFunc iF): simulate(sF), collide(cF), resolve(rF), interpolation(iF) {

    }

    //TODO move collide and resolve in between the interpolation steps its dumb but is easy
    void Engine::step(ps::WordState* _in, ps::WordState* _out) {
        this->in = _in;
        this->out = _out;

        *out = *in;

        for (int i = 0; i < in->simulatedObjects.size(); i++) {
            this->interpolation(this, &out->simulatedObjects[i].rigidbody);
        }

        // for (int i = 0; i < in->simulatedObjects.size(); i++) {
        //     this->collide(this, &out->simulatedObjects[i].rigidbody);
        //     this->resolve(this, &out->simulatedObjects[i].rigidbody);
        // }
    }
}