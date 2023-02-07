#include "pink_graphics.hpp"
#include "klein/klein.hpp"

//Lowest effort fix
nvmath::mat4f ps::Object::interpolate(int prev,int now, float t) {
    if (model.interpolation_catche.obj.approx_eq(rigidbody.M,0.f)) {
        model.interpolation_catche.log = kln::log(motors[prev]  * ~motors[now]);
        model.interpolation_catche.obj = rigidbody.M;
    }

    kln::motor m = kln::exp(model.interpolation_catche.log * t) * motors[now];
    return nvmath::mat4f(m.as_mat4x4().data) * model.scale;
}