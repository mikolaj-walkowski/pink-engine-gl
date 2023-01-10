#include "pink_structs.hpp"
#include "klein/klein.hpp"

namespace ps::pp {
    class Sphere: public BaseShape {
    public:
        float radius;
        kln::point center;

        Sphere(float r, float mass, kln::motor offset);

    };

    class Box: public BaseShape {
    public:

        kln::point verts[8];
        static inline std::pair<int, int> edges[12] =
        {
        {0,1},{0,3},{1,2},{2,3},
        {0,4},{1,5},{2,6},{3,7},
        {4,5},{4,7},{5,6},{6,7}
        };
        Box(float x, float y, float z, float mass, kln::motor offset);
    };


    struct Cylinder: public BaseShape {
    public:
        kln::line centerLine;
        kln::point caps[2];
        float r;

        Cylinder(float len, float r, float m, kln::motor offset);
    };

    struct Plane: public BaseShape {
    public:
        kln::plane plane;

        Plane(kln::plane p);
    };

    struct Composite: public BaseShape {
    public:
        std::vector < std::pair<BaseShape*, kln::motor>> children;

        Composite(std::vector < std::pair<BaseShape*, kln::motor>> data);

        ~Composite(); 
    };

}