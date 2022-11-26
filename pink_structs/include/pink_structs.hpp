#pragma once
#include <vector>
#include <klein/klein.hpp>


namespace ps {
#include "pink_structs_graphics.hpp"
#include "pink_structs_physics.hpp"

  typedef long int UniqueID; // Na razie robię tak bo nie wiem co dokładnie będziemy chcieli mieć jako ID

  class Object {
  private:
  public:
    UniqueID id;
    std::vector<Object*> children;  // OPT można zmienić na tab albo zrobić własną klasę/struct pod tablice o jednolitym rozmiarze
    pp::Rigidbody rigidbody;  // Mogę dać funkcje getInterpolatedTransform(RigidBody previousState, float blend (0.0 ... 1.0 )) zamiast transform i rotation 
    pg::MeshRenderer meshRenderer;
  };

  typedef std::vector<Object> WordState; // Defacto output silnika fizycznego   

  WordState wordChain[4]; // Buffer dla kolejnych stanów 

}  // namespace ps
