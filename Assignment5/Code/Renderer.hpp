#pragma once
#include "Scene.hpp"

// hit position's properties
struct hit_payload
{
    float tNear;
    uint32_t index;
    Vector2f uv; // triangle's barycenter proportions
    Object* hit_obj;
};

class Renderer
{
public:
    void Render(const Scene& scene);

private:
};