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

ps::Object* utils::objectCreate(ps::ObjectManager& objM, kln::motor m, ps::pp::BodyType bt_type, std::string meshName, ps::pp::BaseShape* shape, nvmath::mat4f scale) {
    ps::Object out = {};

    m.normalize();

    //Physics
    out.rigidbody = ps::pp::Rigidbody(m, kln::line(0, 0, 0, 0, 0, 0), bt_type, shape);
    out.rigidbody.apply = bt_type == ps::pp::BT_DYNAMIC ? ps::pp::applyImpulseNormal : ps::pp::applyImpulseStatic; //TODO yuck

    //Graphics 
    ps::pg::Model model;
    auto& names = ps::pg::ObjLibrary::getObjLibrary().m_objectNames;
    auto& name = std::find(names.begin(), names.end(), meshName);
    model.mesh = ps::pg::ObjLibrary::getObjLibrary().GetMesh(name != names.end() ? meshName : ps::pp::shapeName[shape->type]);
    model.scale = scale;

    out.model = model;


    return objM.addObject(out);
}

ps::Object* utils::createCube(ps::ObjectManager& objM, kln::motor m, kln::line b, nvmath::vec3f s,float mass) {
    ps::Object out = {};

    m.normalize();

    //Physics
    out.rigidbody = ps::pp::Rigidbody(
        m,
        b,
        ps::pp::BT_DYNAMIC,
        new ps::pp::Box(s[0], s[1], s[2], mass, kln::uMotor()));
    out.rigidbody.apply = ps::pp::applyImpulseNormal; //TODO yuck

    //Graphics 
    ps::pg::Model model;
    model.mesh = ps::pg::ObjLibrary::getObjLibrary().GetMesh(ps::pp::shapeName[ps::pp::ST_BOX]);
    model.scale = nvmath::scale_mat4(s);

    out.model = model;


    return objM.addObject(out);
}

ps::Object* utils::createBall(ps::ObjectManager& objM, kln::motor m, kln::line b, float s, float mass) {
    ps::Object out = {};

    m.normalize();

    //Physics
    out.rigidbody = ps::pp::Rigidbody(
        m,
        b,
        ps::pp::BT_DYNAMIC,
        new ps::pp::Sphere(s, mass, kln::uMotor()));
    out.rigidbody.apply = ps::pp::applyImpulseNormal; //TODO yuck

    //Graphics 
    ps::pg::Model model;
    model.mesh = ps::pg::ObjLibrary::getObjLibrary().GetMesh(ps::pp::shapeName[ps::pp::ST_SPHERE]);
    model.scale = nvmath::scale_mat4(nvmath::vec3f(s,s,s));

    out.model = model;


    return objM.addObject(out);
}

void utils::createCar(ps::ObjectManager& objM, ps::pp::Engine* e, kln::motor m) {
    ps::Object* body = objectCreate(
        objM,
        m,
        ps::pp::BT_DYNAMIC,
        "car_body",
        new ps::pp::Box(2.f, 1.f, 4.f, 5, kln::uMotor()),
        nvmath::scale_mat4(nvmath::vec3f(1.f, 1.f, 1.f))
    );

    ps::Object* wheels[4];
    ps::pp::Joint joins[4];
    ps::pp::Spring springs[4];
    struct {
        float x;
        float y;
        float z;
    } s[4] = { {1, -1,1},{1, -1,-1},{-1, -1,-1},{-1, -1,1} }, pt = { 1.2f, .5f , 1.4f };
    float travel = 0.6f;
    float wheelR = 0.4f;

    for (int i = 0; i < 4; i++) {
        wheels[i] = objectCreate(
            objM,
            m * kln::sqrt(kln::point(pt.x * s[i].x, pt.y * s[i].y, pt.z * s[i].z) * kln::origin()),
            ps::pp::BT_DYNAMIC,
            "wheel",
            new ps::pp::Sphere(wheelR, 0.2f, kln::uMotor()),
            nvmath::scale_mat4(nvmath::vec3f(wheelR, wheelR, wheelR))
        );
        wheels[i]->rigidbody.joins = new int[1];
        wheels[i]->rigidbody.joins[0] = (int)e->joins.size() + i;
        wheels[i]->rigidbody.joinSize = 1;

        kln::point topAttch = kln::point(pt.x * s[i].x, pt.y * s[i].y - (travel / 2.f) * s[i].y, pt.z * s[i].z);
        kln::point botAttch = kln::point(pt.x * s[i].x, pt.y * s[i].y + (travel / 2.f) * s[i].y, pt.z * s[i].z);

        auto line = (topAttch & botAttch).normalized();
        joins[i] = {
            &body->rigidbody,
            &wheels[i]->rigidbody,
            { topAttch, botAttch},
            line
        };

        kln::point springAttch = kln::point(pt.x * s[i].x, pt.y * s[i].y - travel * s[i].y, pt.z * s[i].z);
        springs[i] = {
            &body->rigidbody,
            &wheels[i]->rigidbody,
            kln::sqrt(springAttch * kln::origin()),
            kln::uMotor(),
            travel,
            -0.01f//-10.f -0.3f
        };
    }

    body->rigidbody.joins = new int[4];
    int tmp[] = { (int)e->joins.size(), (int)e->joins.size() + 1,(int)e->joins.size() + 2 ,(int)e->joins.size() + 3 };
    memcpy(body->rigidbody.joins, tmp, sizeof(int) * 4);
    body->rigidbody.joinSize = 4;

    e->joins.insert(e->joins.end(), joins, joins + 4);
    e->springs.insert(e->springs.end(), springs, springs + 4);
}
