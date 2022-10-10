#include "pink_structs.hpp"
#include <stdint.h>
#include <vector>

namespace pp{
    
    class PhysicsEngine
    {
    private:
        uint32_t dT;
        std::vector<ps::Object*> objects;
        std::vector<ps::Object*> constraints;
        std::vector<std::pair<ps::Object*,ps::Object*>> collisions;
        
        void resolveConstraints();
        void applyImpulses();
        void findColliding();
        void updatePositions();
        void correctSinking();
        void step();
       public:
        void start();
        
        PhysicsEngine(/* args */);
        ~PhysicsEngine();
    };
    
}