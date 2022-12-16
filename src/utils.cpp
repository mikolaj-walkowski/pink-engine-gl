#include "utils.hpp"

#include "nvh/cameramanipulator.hpp"
#include "nvh/fileoperations.hpp"
#include "nvpsystem.hpp"
#include "nvvk/commands_vk.hpp"
#include "nvvk/context_vk.hpp"


void utils::glfw::onErrorCallback(int error, const char* description)
{
    fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

GLFWwindow* utils::glfw::setupGLFWindow() {
    glfwSetErrorCallback(utils::glfw::onErrorCallback);
    if (!glfwInit())
    {
        return NULL;
    }
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    auto out = glfwCreateWindow(utils::glfw::SAMPLE_WIDTH, utils::glfw::SAMPLE_HEIGHT, PROJECT_NAME, nullptr, nullptr);

    // Setup camera
    CameraManip.setWindowSize(utils::glfw::SAMPLE_WIDTH, utils::glfw::SAMPLE_HEIGHT);
    CameraManip.setLookat(nvmath::vec3f(2.0f, 2.0f, 2.0f), nvmath::vec3f(0, 0, 0), nvmath::vec3f(0, 1, 0));

    // Setup Vulkan
    if (!glfwVulkanSupported())
    {
        printf("GLFW: Vulkan Not Supported\n");
        return NULL;
    }
    return out;

}

utils::ExtensionList utils::glfw::getGLFWExtensions() {
    assert(glfwVulkanSupported() == 1);
    utils::ExtensionList out;
    out.names = glfwGetRequiredInstanceExtensions(&out.count);
    return out;
}

void utils::nvidia::setupContext(nvvk::Context* c, std::vector<utils::ExtensionList> list) {
    nvvk::ContextCreateInfo ci;
    ci.setVersion(1, 2);                       // Using Vulkan 1.2
    for (uint32_t i = 0; i < list.size(); i++)
    {
        for (uint32_t ext_id = 0; ext_id < list[i].count; ext_id++)  // Adding required extensions (surface, win32, linux, ..)
        {
            ci.addInstanceExtension(list[i].names[ext_id]);
        }
    }
    ci.addInstanceLayer("VK_LAYER_LUNARG_monitor", true);              // FPS in titlebar
    ci.addInstanceExtension(VK_EXT_DEBUG_UTILS_EXTENSION_NAME, true);  // Allow debug names
    ci.addDeviceExtension(VK_KHR_SWAPCHAIN_EXTENSION_NAME);

    c->initInstance(ci);

    // Find all compatible devices
    auto compatibleDevices = c->getCompatibleDevices(ci);
    assert(!compatibleDevices.empty());
    // Use a compatible device
    c->initDevice(compatibleDevices[0], ci);


}

ps::Object utils::objectCreate(kln::motor m, ps::pp::ShapeType type, void* shape) {

    void* shape_dynamic = malloc(ps::pp::shapeSize[type]);
    memcpy(shape_dynamic, shape, ps::pp::shapeSize[type]);

    ps::pp::Rigidbody rb = { m, kln::line(), kln::motor(), kln::line(), kln::origin(), type, shape_dynamic };

    ps::Object out = {};

    out.rigidbody = rb;
    out.mesh = ps::pg::ObjLibrary::getObjLibrary().GetMesh(ps::pp::shapeName[type]);

    return out;
}

void utils::objectDestroy(ps::Object* ob) {
    if (ob->rigidbody.shape != NULL) {
        free(ob->rigidbody.shape);
    }
}

ps::pp::Box utils::boxCreate(float x, float y, float z, kln::motor offset) {
    ps::pp::Box out;
    x /= 2.f;
    y /= 2.f;
    z /= 2.f;
    out.verts[0] = offset(kln::point(-x, -y, -z));
    out.verts[1] = offset(kln::point(x, -y, -z));
    out.verts[2] = offset(kln::point(-x, y, -z));
    out.verts[3] = offset(kln::point(x, y, -z));
    out.verts[4] = offset(kln::point(-x, -y, z));
    out.verts[5] = offset(kln::point(x, -y, z));
    out.verts[6] = offset(kln::point(-x, y, z));
    out.verts[7] = offset(kln::point(x, y, z));


    // for (int i = 0; i < 8; i++) {
    //     auto p = out.verts[i];
    //     printf("Point: %f,%f,%f,%f\n", p.e013(), p.e021(), p.e032(), p.e123());
    // }
    return out;
}
