#pragma once
#include "nvvk/resourceallocator_vk.hpp"
#include "nvvk/debug_util_vk.hpp"
#include <iostream>
#include <vector>
#include "host_device.h"
namespace ps::pg {

    struct ObjMesh
    {
        uint32_t     objIndex;
        uint32_t     nbIndices{ 0 };
        uint32_t     nbVertices{ 0 };
        nvvk::Buffer vertexBuffer;    // Device buffer of all 'Vertex'
        nvvk::Buffer indexBuffer;     // Device buffer of the indices forming triangles
        nvvk::Buffer matColorBuffer;  // Device buffer of array of 'Wavefront material'
        nvvk::Buffer matIndexBuffer;  // Device buffer of array of 'Wavefront material'
    };


    class ObjLibrary
    {
    public:

        static ObjLibrary& getObjLibrary() {
            static ObjLibrary instance;
            return instance;
        }

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


    struct MeshRenderer {
        //TODO MeshRenderer
    };

} //namespace pg