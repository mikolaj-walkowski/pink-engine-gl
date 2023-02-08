#pragma once
#include "imgui.h"
#include "pink_structs.hpp"
#include "pink_physics.hpp"
#include "pink_graphics.hpp"
#include "GLFW/glfw3.h"

class UI {
    bool showGraphics = false;
    bool showPhysics = false;

    ps::pp::Engine* pe;
    SolidColor* ge;
    UI();
public:
    static UI& D() {
        static UI instance;
        return instance;
    }

    void init(GLFWwindow*, bool, ps::pp::Engine*, SolidColor*);
    void newFrame();
    void show();

    void PhysicsWindow();
    void GraphicsWindow();
};