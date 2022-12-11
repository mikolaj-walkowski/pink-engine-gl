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
    nvvk::Buffer m_bObjDesc;


    void AddMesh(ps::pg::ObjMesh& a_objMesh);
    ps::pg::ObjMesh GetMesh(uint32_t index);
    void LoadMesh(const std::string& filename, nvvk::ResourceAllocatorDma& alloc, 
                                    VkDevice device, uint32_t graphicsQueueIndex, nvvk::DebugUtil& debug);

    void CreateObjDescriptionBuffer(nvvk::ResourceAllocatorDma& alloc, VkDevice device, uint32_t graphicsQueueIndex, nvvk::DebugUtil& debug);
};