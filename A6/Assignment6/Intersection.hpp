#ifndef RAYTRACING_INTERSECTION_H
#define RAYTRACING_INTERSECTION_H
#include "Vector.hpp"
#include "Material.hpp"
class Object;
class Sphere;

struct Intersection
{
    Intersection(){
        happened=false;
        coords=Vector3f();
        normal=Vector3f();
        distance= std::numeric_limits<double>::max();
        obj =nullptr;
        m=nullptr;
    }
    bool happened;
    Vector3f coords;
    Vector3f normal;
    double distance;
    Object* obj;
    Material* m;
};

inline bool closer(Intersection i1, Intersection i2)
{
    if(!i1.happened)
        return false;
    if(!i2.happened)
        return true;
    return i1.distance <= i2.distance;
}

#endif //RAYTRACING_INTERSECTION_H
