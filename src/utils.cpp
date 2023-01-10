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

ps::UniqueID utils::newID() {
    static ps::UniqueID id = ps::nullID;
    return ++id;
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

ps::Object utils::objectCreate(kln::motor m, ps::pp::BodyType bt_type, std::string meshName, ps::pp::BaseShape* shape) {
    m.normalize();

    ps::pp::Rigidbody rb = { m, ~m(kln::line(0,0,0,0,0,0)), kln::uMotor(), kln::line(), kln::origin(), bt_type, shape };

    ps::Object out = {};

    out.id = newID();
    out.rigidbody = rb;
    auto& names = ps::pg::ObjLibrary::getObjLibrary().m_objectNames;
    auto& name = std::find(names.begin(), names.end(), meshName);
    out.mesh = ps::pg::ObjLibrary::getObjLibrary().GetMesh(name != names.end() ? meshName : ps::pp::shapeName[shape->type]);

    return out;
}

void utils::objectDestroy(ps::Object* ob) {
    if (ob->rigidbody.shape != NULL) {
        delete ob->rigidbody.shape;
    }
}