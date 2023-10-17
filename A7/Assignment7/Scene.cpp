#include "Scene.hpp"
#include "Intersection.hpp"
#include "Material.hpp"
#include "Vector.hpp"
#include "global.hpp"
#include <cstdint>


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

// sample a light and get it's intersection and pdf
void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){ // is a light source
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray) const
{
    Vector3f hitColor;
    Intersection inter_p = intersect(ray);
    hitColor += inter_p.emit;
    if(inter_p.happened) {
        if(inter_p.m->hasEmission()) {
            return inter_p.m->getEmission();
        }

        Vector3f p = inter_p.coords;
        Vector3f wo = ray.direction;
        Vector3f N = inter_p.normal;
        Vector3f L_dir, L_indir;

        Intersection inter_l;
        float pdf_l;
        sampleLight(inter_l, pdf_l);

        Vector3f x = inter_l.coords;
        Vector3f ws = normalize(x - p);
        Vector3f NN = inter_l.normal;
        Vector3f emit = inter_l.emit;
        // Vector3f emit = inter_l.m->getEmission();
        float dis = (x - p).norm();

        Ray r(p, ws);
        Intersection inter_b = intersect(r);
        if(std::abs(inter_b.distance - dis) < 0.01) {
            L_dir += emit * inter_p.m->eval(wo, ws, N) * dotProduct(ws, N) * dotProduct(-ws, NN) / std::pow(dis, 2) / pdf_l;
        }

        if(get_random_float() < RussianRoulette) {
            Vector3f wi = normalize(inter_p.m->sample(wo, N));
            Intersection inter_t = intersect(Ray(p, wi));
            if(inter_t.happened && !inter_t.m->hasEmission()) {
                r = Ray(inter_t.coords, wi);
                L_indir += castRay(r) * inter_p.m->eval(wo, wi, N)
                         * dotProduct(wi, N) / inter_p.m->pdf(wo, wi, N) / RussianRoulette;
            }
        }
        hitColor += L_dir + L_indir;
        // std::cout << hitColor.x << " " << hitColor.y << " " << hitColor.z << std::endl;
    }
    return hitColor;
}

// Implementation of the Whitted-syle light transport algorithm
//
// This function is the function that compute the color at the intersection point
// of a ray defined by a position and a direction. Note that thus function is recursive (it calls itself).
//
// If the material of the intersected object is either reflective or reflective and refractive,
// then we compute the reflection/refracton direction and cast two new rays into the scene
// by calling the castRay() function recursively. When the surface is transparent, we mix
// the reflection and refraction color using the result of the fresnel equations (it computes
// the amount of reflection and refractin depending on the surface normal, incident view direction
// and surface refractive index).
//
// If the surface is duffuse/glossy we use the Phong illumation model to compute the color
// at the intersection point.
//
// \param ray is the ray to casy
// \param depth is the depth of recursion
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    if (depth > this->maxDepth) {
        return Vector3f(0.0,0.0,0.0);
    }
    Intersection intersection = Scene::intersect(ray);
    Material *m = intersection.m;
    Object *hitObject = intersection.obj;
    Vector3f hitColor = this->backgroundColor;
    Vector2f uv;
    uint32_t index = 0;
    if(intersection.happened) {
        Vector3f hitPoint = intersection.coords;
        Vector3f N = intersection.normal; // normal
        Vector2f st; // st coordinates
        hitObject->getSurfaceProperties(hitPoint, ray.direction, index, uv, N, st);
        switch (m->getType()) {
            case REFLECTION_AND_REFRACTION: // reflection and refraction
            {
                Vector3f reflectionDirection = normalize(reflect(ray.direction, N));
                Vector3f refractionDirection = normalize(refract(ray.direction, N, m->ior));
                Vector3f reflectionRayOrig = (dotProduct(reflectionDirection, N) < 0) ?
                                             hitPoint - N * EPSILON : hitPoint + N * EPSILON;
                Vector3f refractionRayOrig = (dotProduct(refractionDirection, N) < 0) ?
                                             hitPoint - N * EPSILON : hitPoint + N * EPSILON;
                Vector3f reflectionColor = castRay(Ray(reflectionRayOrig, reflectionDirection), depth + 1);
                Vector3f refractionColor = castRay(Ray(refractionRayOrig, refractionDirection), depth + 1);
                float kr;
                fresnel(ray.direction, N, m->ior, kr);
                hitColor = reflectionColor * kr + refractionColor * (1 - kr);
                break;
            }
            case REFLECTION: // reflection
            {
                float kr;
                fresnel(ray.direction, N, m->ior, kr);
                Vector3f reflectionDirection = reflect(ray.direction, N);
                Vector3f reflectionRayOrig = (dotProduct(reflectionDirection, N) < 0) ?
                                             hitPoint + N * EPSILON : hitPoint - N * EPSILON;
                hitColor = castRay(Ray(reflectionRayOrig, reflectionDirection),depth + 1) * kr;
                break;
            }
            default:
            {
                // We use the Phong illumation model int the default case. The phong model
                // is composed of a diffuse and a specular reflection component.
                Vector3f lightAmt = 0, specularColor = 0;
                Vector3f shadowPointOrig = (dotProduct(ray.direction, N) < 0) ?
                                           hitPoint + N * EPSILON : hitPoint - N * EPSILON;

                // Loop over all lights in the scene and sum their contribution up
                // We also apply the lambert cosine law
                for (uint32_t i = 0; i < get_lights().size(); ++i)
                {
                    auto area_ptr = dynamic_cast<AreaLight*>(this->get_lights()[i].get());
                    if (area_ptr)
                    {
                        // Do nothing for this assignment
                    }
                    else
                    {
                        Vector3f lightDir = get_lights()[i]->position - hitPoint;
                        // square of the distance between hitPoint and the light
                        float lightDistance2 = dotProduct(lightDir, lightDir);
                        lightDir = normalize(lightDir);
                        float LdotN = std::max(0.f, dotProduct(lightDir, N));
                        Object *shadowHitObject = nullptr;
                        float tNearShadow = kInfinity;
                        // is the point in shadow, and is the nearest occluding object closer to the object than the light itself?
                        bool inShadow = bvh->Intersect(Ray(shadowPointOrig, lightDir)).happened;
                        lightAmt += (1 - inShadow) * get_lights()[i]->intensity * LdotN;
                        Vector3f reflectionDirection = reflect(-lightDir, N);
                        specularColor += powf(std::max(0.f, -dotProduct(reflectionDirection, ray.direction)),
                                              m->specularExponent) * get_lights()[i]->intensity;
                    }
                }
                hitColor = lightAmt * (hitObject->evalDiffuseColor(st) * m->Kd + specularColor * m->Ks);
                break;
            }
        }
    }
    return hitColor;
}