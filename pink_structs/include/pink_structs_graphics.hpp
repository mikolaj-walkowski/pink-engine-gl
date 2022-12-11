#pragma once
#include "nvmath/nvmath.h"

namespace ps::pg{

class ObjMesh
{
    uint32_t     objIndex;
    uint32_t     nbIndices{ 0 };
    uint32_t     nbVertices{ 0 };
    nvvk::Buffer vertexBuffer;    // Device buffer of all 'Vertex'
    nvvk::Buffer indexBuffer;     // Device buffer of the indices forming triangles
    nvvk::Buffer matColorBuffer;  // Device buffer of array of 'Wavefront material'
    nvvk::Buffer matIndexBuffer;  // Device buffer of array of 'Wavefront material'
};

struct MeshRenderer {
    //TODO MeshRenderer
};

} //namespace pg