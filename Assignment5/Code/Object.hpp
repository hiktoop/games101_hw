#pragma once

#include "Vector.hpp"
#include "global.hpp"

// virtual class
class Object
{
public:
    Object()
        : materialType(DIFFUSE_AND_GLOSSY)
        , ior(1.3)
        , Kd(0.8)
        , Ks(0.2)
        , diffuseColor(0.2)
        , specularExponent(25)
    {}

    virtual ~Object() = default;

    virtual bool intersect(const Vector3f&, const Vector3f&, float&, uint32_t&, Vector2f&) const = 0;

    // too many arguements
    virtual void getSurfaceProperties(const Vector3f&, const Vector3f&, const uint32_t&,
                                      const Vector2f&, Vector3f&, Vector2f&) const = 0;

    // what's the meanning of the arguement
    virtual Vector3f evalDiffuseColor(const Vector2f&) const
    {
        return diffuseColor;
    }

    // material properties
    MaterialType materialType;
    float ior;
    float Kd, Ks;
    Vector3f diffuseColor;
    float specularExponent;
};
