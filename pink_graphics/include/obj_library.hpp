#pragma once
#include "pink_structs.hpp"
#include "nvvk/debug_util_vk.hpp"
#include <iostream>
#include <vector>


class ObjLibrary
{
public:
    struct ObjDesc
    {
        int      txtOffset;             // Texture index offset in the array of textures
        uint64_t vertexAddress;         // Address of the Vertex buffer
        uint64_t indexAddress;          // Address of the index buffer
        uint64_t materialAddress;       // Address of the material buffer
        uint64_t materialIndexAddress;  // Address of the triangle material index buffer
    };
    
    std::vector<ps::pg::ObjMesh> m_meshContainer;    //Robię to jako private, żeby raczej nie dodawać elementów ręcznie, tylko metodą AddMesh()
    std::vector<ObjDesc> m_descContainer;
    std::vector<std::string> m_objectNames;
    nvvk::Buffer m_bObjDesc;

    void AddMesh(ps::pg::ObjMesh& a_objMesh, const std::string& a_objName);
    ps::pg::ObjMesh* GetMesh(uint32_t index);
    ps::pg::ObjMesh* GetMesh(const std::string& ar_objName);

    void LoadMesh(const std::string& filename, const std::string& name, 
                  nvvk::ResourceAllocatorDma& alloc, VkDevice device, 
                  uint32_t graphicsQueueIndex, nvvk::DebugUtil& debug);

    void CreateObjDescriptionBuffer(nvvk::ResourceAllocatorDma& alloc, VkDevice device, uint32_t graphicsQueueIndex, nvvk::DebugUtil& debug);
};