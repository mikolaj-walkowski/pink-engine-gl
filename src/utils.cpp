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

ps::Object utils::objectCreate(kln::motor m, ps::pp::BodyType bt_type, std::string meshName, ps::pp::BaseShape* shape, ps::pp::BaseShape* moved) {
    m.normalize();


    ps::pp::Rigidbody rb = { m, ~m(kln::line(0,0,0,0,0,0)), kln::uMotor(), kln::line(),kln::line(0,0,0,0,0,0), kln::origin(), bt_type, shape, moved };
    rb.moved->move(rb.M, rb.shape);
    rb.apply = bt_type == ps::pp::BT_DYNAMIC ? ps::pp::applyImpulseNormal : ps::pp::applyImpulseStatic; //TODO yuck
    ps::Object out = {};

    out.id = newID();
    out.rigidbody = rb;

    auto& names = ps::pg::ObjLibrary::getObjLibrary().m_objectNames;
    auto& name = std::find(names.begin(), names.end(), meshName);
    out.mesh = ps::pg::ObjLibrary::getObjLibrary().GetMesh(name != names.end() ? meshName : ps::pp::shapeName[shape->type]);

    return out;
}

void utils::createCar(ps::WordState* ws, ps::pp::Engine* e, kln::motor m) {
    ps::Object body = objectCreate(
        m,
        ps::pp::BT_DYNAMIC,
        "car_body",
        new ps::pp::Box(2.f, 1.f, 4.f, 5, kln::uMotor()),
        new ps::pp::Box(2.f, 1.f, 4.f, 5, kln::uMotor())
    );
    body.rigidbody.shape->size = nvmath::scale_mat4(nvmath::vec3f(1, 1, 1));
    
    int bodyIndex = (int)ws->simulatedObjects.size();
    ws->simulatedObjects.push_back(body);
    int whIndexStart = (int)ws->simulatedObjects.size();

    ps::Object wheels[4];
    ps::pp::Joint joins[4];
    ps::pp::Spring springs[4];
    struct {
        float x;
        float y;
        float z;
    } s[4] = { {1, -1,1},{1, -1,-1},{-1, -1,-1},{-1, -1,1} } , pt = {1.3f, .5f , 1.4f};
    float travel = 0.5;
    float wheelR = 0.5;
    
    for (int i = 0; i < 4; i++) {
        wheels[i] = objectCreate(
            m * kln::sqrt(kln::point(pt.x * s[i].x, pt.y * s[i].y, pt.z * s[i].z)* kln::origin()),
            ps::pp::BT_DYNAMIC,
            "wheel",
            new ps::pp::Sphere(wheelR, 0.2f, kln::uMotor()),
            new ps::pp::Sphere(wheelR, 0.2f, kln::uMotor())
        );
        wheels[i].rigidbody.joins = new int[1];
        wheels[i].rigidbody.joins[0] = (int)e->joins.size() + i;
        wheels[i].rigidbody.joinSize = 1;
        
        kln::point topAttch = kln::point(pt.x * s[i].x, pt.y * s[i].y - (travel / 2.f) * s[i].y, pt.z * s[i].z);
        kln::point botAttch = kln::point(pt.x * s[i].x, pt.y * s[i].y + (travel / 2.f) * s[i].y, pt.z * s[i].z);

        auto line = ( topAttch & botAttch).normalized();
        joins[i] = {
            bodyIndex,
            whIndexStart + i,
            { topAttch, botAttch},
            line
        };
        
        kln::point springAttch = kln::point(pt.x * s[i].x, pt.y * s[i].y - travel * s[i].y, pt.z * s[i].z);
        springs[i] = {
            bodyIndex,
            whIndexStart + i,
            kln::sqrt( springAttch * kln::origin()),
            kln::uMotor(),
            travel,
            -0.3f//-10.f -0.3f
        };
    }
    
    body.rigidbody.joins = new int[4];
    int tmp[] = { (int)e->joins.size(), (int)e->joins.size() + 1,(int)e->joins.size() + 2 ,(int)e->joins.size() + 3 };
    memcpy(body.rigidbody.joins, tmp, sizeof(int) * 4);
    body.rigidbody.joinSize = 4;
    
    ws->simulatedObjects.insert(ws->simulatedObjects.end(), wheels, wheels + 4);
    e->joins.insert(e->joins.end(), joins, joins + 4);
    e->springs.insert(e->springs.end(), springs, springs + 4);
}

void utils::objectDestroy(ps::Object* ob) {
    if (ob->rigidbody.shape != NULL) {
        delete ob->rigidbody.shape;
        delete ob->rigidbody.moved;
        delete ob->rigidbody.joins;
    }
}