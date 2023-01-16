#include "pink_graphics.hpp"
#include "klein/klein.hpp"

//Lowest effort fix
nvmath::mat4f ps::Object::interpolate(ps::Object* a, float t) {
    if (this->interpolation_catche.obj.approx_eq(rigidbody.M,0.f)) {
        this->interpolation_catche.log = kln::log(a->rigidbody.M  * ~this->rigidbody.M);
        this->interpolation_catche.obj = rigidbody.M;
    }

    kln::motor m = kln::exp(this->interpolation_catche.log * t) * this->rigidbody.M;
    return nvmath::mat4f(m.as_mat4x4().data) * a->rigidbody.shape->size;
}