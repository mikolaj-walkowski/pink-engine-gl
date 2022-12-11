#include "obj_library.hpp"
#include "obj_loader.h"
#include "nvvk/commands_vk.hpp"
#include "nvvk/buffers_vk.hpp"
#include "host_device.h"

void ObjLibrary::AddMesh(ps::pg::ObjMesh& a_objMesh, const std::string& a_objName){
    a_objMesh.objIndex = static_cast<uint32_t>(m_meshContainer.size());
    this->m_meshContainer.push_back(a_objMesh);
    this->m_objectNames.push_back(a_objName);

    return;
}

ps::pg::ObjMesh* ObjLibrary::GetMesh(uint32_t index){
    return (m_meshContainer.size() > index ? &m_meshContainer[index] : nullptr);
}

ps::pg::ObjMesh* ObjLibrary::GetMesh(const std::string& ar_objName){
    uint32_t index = 0;
    for(auto name: this->m_objectNames){
        if(ar_objName == name){
            return &m_meshContainer[index];
        }
        ++index;
    }
    return nullptr;
}

void ObjLibrary::LoadMesh(const std::string& filename, const std::string& name, 
                          nvvk::ResourceAllocatorDma& alloc, VkDevice device, 
                          uint32_t graphicsQueueIndex, nvvk::DebugUtil& debug){
    
    ObjLoader loader;
    loader.loadModel(filename);

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
    nvvk::CommandPool  cmdBufGet(device, graphicsQueueIndex);
    VkCommandBuffer    cmdBuf = cmdBufGet.createCommandBuffer();
    VkBufferUsageFlags flag = VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT;
    model.vertexBuffer = alloc.createBuffer(cmdBuf, loader.m_vertices, VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | flag);
    model.indexBuffer = alloc.createBuffer(cmdBuf, loader.m_indices, VK_BUFFER_USAGE_INDEX_BUFFER_BIT | flag);
    model.matColorBuffer = alloc.createBuffer(cmdBuf, loader.m_materials, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | flag);
    model.matIndexBuffer = alloc.createBuffer(cmdBuf, loader.m_matIndx, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | flag);

    // Creates all textures found and find the offset for this model
    // auto txtOffset = static_cast<uint32_t>(m_textures.size());
    // createTextureImages(cmdBuf, loader.m_textures);
    cmdBufGet.submitAndWait(cmdBuf);
    alloc.finalizeAndReleaseStaging();

    std::string objNb = std::to_string(m_meshContainer.size());
    debug.setObjectName(model.vertexBuffer.buffer, (std::string("vertex_" + objNb)));
    debug.setObjectName(model.indexBuffer.buffer, (std::string("index_" + objNb)));
    debug.setObjectName(model.matColorBuffer.buffer, (std::string("mat_" + objNb)));
    debug.setObjectName(model.matIndexBuffer.buffer, (std::string("matIdx_" + objNb)));

    // Keeping transformation matrix of the instance
    // ObjInstance instance;
    // instance.transform = transform;
    // instance.objIndex = static_cast<uint32_t>(m_objModel.size());
    // m_instances.push_back(instance);

    // Creating information for device access
    ObjDesc desc;
    desc.txtOffset = 0;
    // desc.txtOffset = txtOffset;
    desc.vertexAddress = nvvk::getBufferDeviceAddress(device, model.vertexBuffer.buffer);
    desc.indexAddress = nvvk::getBufferDeviceAddress(device, model.indexBuffer.buffer);
    desc.materialAddress = nvvk::getBufferDeviceAddress(device, model.matColorBuffer.buffer);
    desc.materialIndexAddress = nvvk::getBufferDeviceAddress(device, model.matIndexBuffer.buffer);

    // Keeping the obj host model and device description
    this->AddMesh(model, name);
    m_descContainer.emplace_back(desc);
}

void ObjLibrary::CreateObjDescriptionBuffer(nvvk::ResourceAllocatorDma& alloc, VkDevice device, uint32_t graphicsQueueIndex, nvvk::DebugUtil& debug){
    nvvk::CommandPool cmdGen(device, graphicsQueueIndex);

    auto cmdBuf = cmdGen.createCommandBuffer();
    m_bObjDesc = alloc.createBuffer(cmdBuf, m_descContainer, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
    cmdGen.submitAndWait(cmdBuf);
    alloc.finalizeAndReleaseStaging();
    debug.setObjectName(m_bObjDesc.buffer, "ObjDescs");
}