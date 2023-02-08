#include "pink_structs.hpp"
#include "klein/klein.hpp"

namespace ps::pp {
    class Sphere: public BaseShape {
    public:
        float radius;

        Sphere(float r, float mass, kln::motor offset);
        virtual void move(const kln::motor& M);
    };

    class Box: public BaseShape {
        inline static const kln::point _verts[8] =
        {
            kln::point(-1.f, 1.f, 1.f),
            kln::point(1.f, 1.f, 1.f),
            kln::point(1.f, 1.f, -1.f),
            kln::point(-1.f, 1.f, -1.f),
            kln::point(-1.f, -1.f, 1.f),
            kln::point(1.f, -1.f, 1.f),
            kln::point(1.f, -1.f, -1.f),
            kln::point(-1.f, -1.f, -1.f)
        };
    public:

        kln::point verts[8];
        static inline const std::pair<int, int> edges[12] =
        {
        {0,1},{0,3},{1,2},{2,3},
        {0,4},{1,5},{2,6},{3,7},
        {4,5},{4,7},{5,6},{6,7}
        };

        static inline const int faces[6][4] =
        {
            {0,1,2,3},          // edges[0] + edges[1] + edges[2] + edges[3] (top)                     
            {0,1,5,4},          // edges[0] + edges[4] + edges[5] + edges[8] (side)                  
            {0,3,7,4},          // edges[1] + edges[7] + edges[9] + edges[4] (side)     
            {1,2,6,5},          // edges[2] + edges[6] + edges[10] + edges[5] (side)    
            {2,3,7,6},          // edges[3] + edges[7] + edges[11] + edges[6] (side)    
            {4,5,6,7}           // edges[8] + edges[9] + edges[10] + edges[11] (bottom) 
        };

        Box(float x, float y, float z, float mass, kln::motor offset);
        virtual void move(const kln::motor& M);
        void getBodyVerts(kln::point* v);
    };


    class Cylinder: public BaseShape {
        inline static const kln::point _caps[2] = {
            kln::point(0.f, 1.f, 0.f),
            kln::point(0.f, -1.f, 0.f)
        };
    public:
        kln::point caps[2];
        float r;

        Cylinder(float len, float r, float m, kln::motor offset);
        virtual void move(const kln::motor& M);

    };

    class Plane: public BaseShape {
        kln::plane _plane;
    public:
        kln::plane plane;

        Plane(kln::plane p);
        virtual void move(const kln::motor& M);

    };

    class Composite: public BaseShape {
    public:
        std::vector < std::pair<BaseShape*, kln::motor>> children;

        Composite(std::vector < std::pair<BaseShape*, kln::motor>> data);
        virtual void move(const kln::motor& M);

        ~Composite();
    };

}