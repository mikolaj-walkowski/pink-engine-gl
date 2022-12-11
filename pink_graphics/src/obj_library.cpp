#include "obj_library.hpp"

void ObjLibrary::AddMesh(ps::pg::ObjMesh& a_objMesh){
    a_objMesh.objIndex = static_cast<uint32_t>(m_meshContainer.size());
    this->m_meshContainer.push_back(a_objMesh);

    return;
}

ps::pg::ObjMesh ObjLibrary::GetMesh(uint32_t index){
    return m_meshContainer[index];
}