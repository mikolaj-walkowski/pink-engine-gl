#pragma once
#include "pink_structs.hpp"
#include <iostream>
#include <vector>

class ObjLibrary
{
private:
    std::vector<ps::pg::ObjMesh> m_meshContainer;    //Robię to jako private, żeby raczej nie dodawać elementów ręcznie, tylko metodą AddMesh()

public:
    void AddMesh(ps::pg::ObjMesh& a_objMesh);
    ps::pg::ObjMesh GetMesh(uint32_t index);

};