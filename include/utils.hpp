#pragma once
#include "imgui/imgui_camera_widget.h"
#include "nvh/cameramanipulator.hpp"
#include "nvh/fileoperations.hpp"
#include "nvpsystem.hpp"

#include "nvvk/commands_vk.hpp"
#include "nvvk/context_vk.hpp"
#include "GLFW/glfw3.h"

#include "pink_structs.hpp"
#include <vector>

namespace utils {
    struct ExtensionList
    {
        uint32_t count;
        const char** names;
    };


    namespace glfw {
        static int const SAMPLE_WIDTH = 1280;
        static int const SAMPLE_HEIGHT = 720;

        static void onErrorCallback(int error, const char* description);
        GLFWwindow* setupGLFWindow();

        utils::ExtensionList getGLFWExtensions();

    }//namespace glfw

    namespace nvidia {
        //void setupSe(std::vector<std::string>*,);
        void setupContext(nvvk::Context*, std::vector<utils::ExtensionList>);
    }//namespace nvidia

    ps::UniqueID newID();

    ps::Object objectCreate(kln::motor m, ps::pp::BodyType, ps::pp::ShapeType type, void* shape);
    ps::pp::Box boxCreate(float x, float y, float z, float mass, kln::motor offset);
    ps::pp::Sphere sphereCreate(float r, float mass, kln::motor offset);
    ps::pp::Plane planeCreate(kln::plane p);

    void objectDestroy(ps::Object* rb);
}//namespace utils