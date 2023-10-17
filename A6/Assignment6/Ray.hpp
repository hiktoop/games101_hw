#ifndef RAYTRACING_RAY_H
#define RAYTRACING_RAY_H
#include "Vector.hpp"
struct Ray{
    // Destnation = origin + t * direction
    Vector3f origin;
    Vector3f direction, direction_inv;
    double t; //transportation time
    double t_min, t_max; // why not const?

    Ray(const Vector3f& ori, const Vector3f& dir, const double _t = 0.0): origin(ori), direction(dir),t(_t) {
        direction_inv = Vector3f(1./direction.x, 1./direction.y, 1./direction.z); // what's the meaning?
        t_min = 0.0;
        t_max = std::numeric_limits<double>::max();

    }

    // return the position ray teach at time t
    // but the type is too strange
    Vector3f operator()(double t) const{return origin+direction*t;}

    // this is a common way I should learn
    friend std::ostream &operator<<(std::ostream& os, const Ray& r){
        os<<"[origin:="<<r.origin<<", direction="<<r.direction<<", time="<< r.t<<"]\n";
        return os;
    }
};
#endif //RAYTRACING_RAY_H
