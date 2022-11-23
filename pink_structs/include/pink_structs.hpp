#pragma once
#include <vector>
#include "glm/glm.hpp"
#include "glm/gtx/quaternion.hpp"
#include <klein/klein.hpp>

namespace ps {
#include "pink_structs_graphics.hpp"
#include "pink_structs_physics.hpp"
  class Object {
 private:
 public:
  glm::vec3 transform;
  glm::quat rotation;
  std::vector<Object*> children;  // OPT można zmienić na tab albo zrobić własną klasę/struct pod tablice o jednolitym rozmiarze
  pp::Rigidbody rigidbody;
  pg::MeshRenderer meshRenderer;
};
}  // namespace ps
