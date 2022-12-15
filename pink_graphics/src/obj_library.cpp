#include "pink_structs.hpp"
#include "nvvk/commands_vk.hpp"
#include "nvvk/buffers_vk.hpp"
#include "nvh/fileoperations.hpp"
#include "host_device.h"
#include "obj_loader.h"

#include <filesystem>

void ps::pg::ObjLibrary::init(nvvk::ResourceAllocatorDma* ap_alloc, VkDevice a_device, 
                uint32_t a_graphicsQueueIndex, nvvk::DebugUtil& ar_debug){

        mp_alloc = ap_alloc;
        m_device = a_device;
        m_graphicsQueueIndex = a_graphicsQueueIndex;
        mp_debug = &ar_debug;
    }

void ps::pg::ObjLibrary::AddMesh(ps::pg::ObjMesh& ar_objMesh, const std::string a_objName){
    ar_objMesh.objIndex = static_cast<uint32_t>(m_meshContainer.size());
    this->m_meshContainer.push_back(ar_objMesh);
    this->m_objectNames.push_back(a_objName);

    return;
}

ps::pg::ObjMesh* ps::pg::ObjLibrary::GetMesh(uint32_t index){
    return (m_meshContainer.size() > index ? &m_meshContainer[index] : nullptr);
}

ps::pg::ObjMesh* ps::pg::ObjLibrary::GetMesh(const std::string& ar_objName){
    uint32_t index = 0;
    for(auto name: this->m_objectNames){
        if(ar_objName == name){
            return &m_meshContainer[index];
        }
        ++index;
    }
    return nullptr;
}

void ps::pg::ObjLibrary::LoadMesh(const std::string& ar_file_path, const std::string& ar_name){
    
    const std::string objName = ar_name.empty()? nvh::getFileName(ar_file_path) : ar_name;

    ObjLoader loader;
    loader.loadModel(ar_file_path);

    // Converting from Srgb to linear
    for (auto& m : loader.m_materials)
    {
        m.ambient = nvmath::pow(m.ambient, 2.2f);
        m.diffuse = nvmath::pow(m.diffuse, 2.2f);
        m.specular = nvmath::pow(m.specular, 2.2f);
    }

    ps::pg::ObjMesh model;
    model.nbIndices = static_cast<uint32_t>(loader.m_indices.size());
    model.nbVertices = static_cast<uint32_t>(loader.m_vertices.size());

    // Create the buffers on Device and copy vertices, indices and materials
    nvvk::CommandPool  cmdBufGet(m_device, m_graphicsQueueIndex);
    VkCommandBuffer    cmdBuf = cmdBufGet.createCommandBuffer();
    VkBufferUsageFlags flag = VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT;
    model.vertexBuffer = mp_alloc->createBuffer(cmdBuf, loader.m_vertices, VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | flag);
    model.indexBuffer = mp_alloc->createBuffer(cmdBuf, loader.m_indices, VK_BUFFER_USAGE_INDEX_BUFFER_BIT | flag);
    model.matColorBuffer = mp_alloc->createBuffer(cmdBuf, loader.m_materials, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | flag);
    model.matIndexBuffer = mp_alloc->createBuffer(cmdBuf, loader.m_matIndx, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | flag);

    // Creates all textures found and find the offset for this model
    // auto txtOffset = static_cast<uint32_t>(m_textures.size());
    // createTextureImages(cmdBuf, loader.m_textures);
    cmdBufGet.submitAndWait(cmdBuf);
    mp_alloc->finalizeAndReleaseStaging();

    std::string objNb = std::to_string(m_meshContainer.size());
    mp_debug->setObjectName(model.vertexBuffer.buffer, (std::string("vertex_" + objNb)));
    mp_debug->setObjectName(model.indexBuffer.buffer, (std::string("index_" + objNb)));
    mp_debug->setObjectName(model.matColorBuffer.buffer, (std::string("mat_" + objNb)));
    mp_debug->setObjectName(model.matIndexBuffer.buffer, (std::string("matIdx_" + objNb)));

    // Keeping transformation matrix of the instance
    // ObjInstance instance;
    // instance.transform = transform;
    // instance.objIndex = static_cast<uint32_t>(m_objModel.size());
    // m_instances.push_back(instance);

    // Creating information for device access
    ObjDesc desc;
    desc.txtOffset = 0;
    // desc.txtOffset = txtOffset;
    desc.vertexAddress = nvvk::getBufferDeviceAddress(m_device, model.vertexBuffer.buffer);
    desc.indexAddress = nvvk::getBufferDeviceAddress(m_device, model.indexBuffer.buffer);
    desc.materialAddress = nvvk::getBufferDeviceAddress(m_device, model.matColorBuffer.buffer);
    desc.materialIndexAddress = nvvk::getBufferDeviceAddress(m_device, model.matIndexBuffer.buffer);

    // Keeping the obj host model and device description
    this->AddMesh(model, objName);
    m_descContainer.emplace_back(desc);
    return;
}

void ps::pg::ObjLibrary::LoadDirectory(const std::string& ar_directory_path, std::vector<std::string>& ar_names){

    uint32_t index = 0;
    std::string objName = "";

    std::filesystem::path input_directory(ar_directory_path);
    for (const auto& dir_entry : std::filesystem::directory_iterator{input_directory}) 
    {
        auto test = dir_entry.path().extension().string();
        if(dir_entry.path().extension().string() != ".obj")
            continue;

        if(ar_names.empty() || ar_names.size() <= index || ar_names[index].empty())
            objName = dir_entry.path().stem().string(); // this almost looks like Java code. Yep, that's why c++ > 11 is a joke.
        else
            objName = ar_names[index];
        
        LoadMesh(dir_entry.path().string(), objName);
    }

    return;
}

void ps::pg::ObjLibrary::CreateObjDescriptionBuffer(){
    nvvk::CommandPool cmdGen(m_device, m_graphicsQueueIndex);

    auto cmdBuf = cmdGen.createCommandBuffer();
    m_bObjDesc = mp_alloc->createBuffer(cmdBuf, m_descContainer, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
    cmdGen.submitAndWait(cmdBuf);
    mp_alloc->finalizeAndReleaseStaging();
    mp_debug->setObjectName(m_bObjDesc.buffer, "ObjDescs");
}