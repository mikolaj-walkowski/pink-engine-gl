#pragma once
#include <vector>
#include <klein/klein.hpp>

#include "pink_structs_physics.hpp"
#include "pink_structs_graphics.hpp"
#include "nvp/nvpsystem.hpp"

namespace ps {

  static std::vector<std::string> defaultSearchPaths = {
    NVPSystem::exePath() + "..",
    NVPSystem::exePath() + "../../",
    std::string(PROJECT_NAME),
  };

  typedef long int UniqueID; // Na razie robię tak bo nie wiem co dokładnie będziemy chcieli mieć jako ID
  const UniqueID nullID = 0;

  class Object {
  public:
    UniqueID id;
    pp::Rigidbody rigidbody;
    pg::Model model;

    nvmath::mat4f interpolate(int ,int ,float);
    kln::motor motors[4];
  };

  struct ObjectIDCmp {
    bool operator()(const ps::Object* left, const ps::Object* right)const
    {
      return (left->id < right->id);
    }
  };

  class Module {
  public:
    virtual void registerObject(Object*) = 0;
    virtual void deregisterObject(Object*) = 0;
  };

  class ObjectManager {
    UniqueID idCounter = nullID;
  public:
    std::vector<Object*> objects;
    std::vector<Module*> modules;

    UniqueID newUniqueId();
    Object* addObject(const Object&);
    void deleteObject(Object*);
    Object* findObject(UniqueID id);
    std::vector<Object*>::iterator ObjectManager::findObjectInVector(UniqueID id);
    ObjectManager(std::vector<Module*>);
  };

}  // namespace ps
