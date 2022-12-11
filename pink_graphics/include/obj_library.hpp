#pragma once
#include "pink_structs.hpp"
#include "nvvk/debug_util_vk.hpp"
#include <iostream>
#include <vector>


class ObjLibrary
{
public:
    /**
     * Struct z informacjami do desktryptora
     */
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
    nvvk::ResourceAllocatorDma* mp_alloc; 
    VkDevice m_device; 
    uint32_t m_graphicsQueueIndex; 
    nvvk::DebugUtil* mp_debug;

    void init(nvvk::ResourceAllocatorDma* ap_alloc, VkDevice a_device, uint32_t a_graphicsQueueIndex, nvvk::DebugUtil& ar_debug);

    void AddMesh(ps::pg::ObjMesh& ar_objMesh, const std::string a_objName);
    ps::pg::ObjMesh* GetMesh(uint32_t index);
    ps::pg::ObjMesh* GetMesh(const std::string& ar_objName);

    void LoadMesh(const std::string& ar_file_path, const std::string& ar_name = "");

    void LoadDirectory(const std::string& ar_directory_path, std::vector<std::string>& ar_names = std::vector<std::string>());

    void CreateObjDescriptionBuffer();
};