#pragma once
#include <vector>
#include <klein/klein.hpp>
#include "pink_structs_physics.hpp"
#include "pink_structs_graphics.hpp"
#include "nvp/nvpsystem.hpp"

namespace ps {

  // Default search path for shaders
  static std::vector<std::string> defaultSearchPaths = {
        //NVPSystem::exePath() + PROJECT_RELDIRECTORY,
        //NVPSystem::exePath() + PROJECT_RELDIRECTORY "..",
        NVPSystem::exePath() + "..",
        NVPSystem::exePath() + "../../",
        std::string(PROJECT_NAME),
  };

  typedef long int UniqueID; // Na razie robię tak bo nie wiem co dokładnie będziemy chcieli mieć jako ID
  const UniqueID nullID = 0;

  class Object {
  private:
  public:
    UniqueID id;
    std::vector<Object*> children;  // OPT można zmienić na tab albo zrobić własną klasę/struct pod tablice o jednolitym rozmiarze
    pp::Rigidbody rigidbody;  // Mogę dać funkcje getInterpolatedTransform(RigidBody previousState, float blend (0.0 ... 1.0 )) zamiast transform i rotation 
    pg::ObjMesh* mesh;
    pg::MeshRenderer meshRenderer;
    struct Interpolation_catche {
      kln::line log;
      ps::UniqueID obj;
    }interpolation_catche;
    nvmath::mat4f interpolate(ps::Object*, float);
  };


  struct WordState { // Defacto output silnika fizycznego
    std::vector<Object> staticObjects;
    std::vector<Object> simulatedObjects;
  };

}  // namespace ps
