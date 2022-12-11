#pragma once
#include "pink_structs.hpp"
#include "nvvk/debug_util_vk.hpp"
#include <iostream>
#include <vector>

class ObjLibrary
{
private:
    std::vector<ps::pg::ObjMesh> m_meshContainer;    //Robię to jako private, żeby raczej nie dodawać elementów ręcznie, tylko metodą AddMesh()
    std::vector<ps::pg::ObjMesh> m_descContainer;
    nvvk::Buffer m_bObjDesc;

public:
    void AddMesh(ps::pg::ObjMesh& a_objMesh);
    ps::pg::ObjMesh GetMesh(uint32_t index);
    void LoadMesh(const std::string& filename, nvvk::ResourceAllocatorDma& alloc, 
                                    VkDevice device, uint32_t graphicsQueueIndex, nvvk::DebugUtil& debug);

    void CreateObjDescriptionBuffer(nvvk::ResourceAllocatorDma& alloc, VkDevice device, uint32_t graphicsQueueIndex, nvvk::DebugUtil& debug);
};