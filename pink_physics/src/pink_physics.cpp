#include "pink_physics.hpp"
#include "pink_structs.hpp"
#include "pink_physics_colliders.hpp"

void ps::pp::eulerInterpolation(ps::pp::Engine* e, ps::pp::Rigidbody* rb) {
    int iterations = e->interpolation_props.iterations;
    float step = e->interpolation_props.step;
    for (int i = 0; i < iterations; i++)
    {
        e->simulate(rb);
        rb->M = rb->M + step * rb->dM;
        rb->B = rb->B + step * rb->dB;
    }

}

void ps::pp::basicSimulate(ps::pp::Rigidbody* rb) {
    kln::line G = (~(rb->M))(kln::ideal_line(0.f, 9.81f, 0.f));

    kln::line F = G; // + wszystkie siły
    
    // rb->dM = -0.5f * (rb->M * rb->B);
    // rb->dB = F -0.5f * !(!rb->B * rb->B - rb->B * !rb->B);
}
void ps::pp::basicCollider(ps::pp::Engine* e, ps::pp::Rigidbody* rb) {

}

void ps::pp::basicResolver(ps::pp::Engine* e, ps::pp::Rigidbody* rb) {

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

    Engine::Engine(SimulateFunc sf, ColliderFunc cf, ResolverFunc rf) : simulate(sf), collide(cf), resolve(rf) {

    }

    void Engine::step(ps::WordState* _in, ps::WordState* _out) {
        this->in = _in;
        this->out = _out;

        *out = *in;

        for (int i = 0; i < in->simulatedObjects.size(); i++) {
            this->simulate(&out->simulatedObjects[i].rigidbody);
        }

        for (int i = 0; i < in->simulatedObjects.size(); i++) {
            this->collide(this, &out->simulatedObjects[i].rigidbody);
            this->resolve(this, &out->simulatedObjects[i].rigidbody);
        }
    }
}