#pragma once
#include "../math/vec.h"
#include "../math/ray.h"
#include <vector>
#include <string>
#include <cstdint>
#include <cmath>
#include <algorithm>

// ----------------------------------------------------------------
// Texture
// ----------------------------------------------------------------
struct Texture {
    std::vector<uint8_t> data;
    int w=0, h=0, ch=0;

    bool valid() const { return !data.empty(); }

    Vec3 sample(Vec2 uv) const {
        float u = uv.x - std::floor(uv.x);
        float v = 1.f - (uv.y - std::floor(uv.y));
        int px = std::clamp(int(u*w), 0, w-1);
        int py = std::clamp(int(v*h), 0, h-1);
        int idx = (py*w + px)*ch;
        float r = std::pow(data[idx]/255.f, 2.2f);
        float g = std::pow(data[std::min(idx+1,(int)data.size()-1)]/255.f, 2.2f);
        float b = std::pow(data[std::min(idx+2,(int)data.size()-1)]/255.f, 2.2f);
        return {r, g, b};
    }

    // For alpha-mask textures: read the first channel only
    float sampleAlpha(Vec2 uv) const {
        float u = uv.x - std::floor(uv.x);
        float v = 1.f - (uv.y - std::floor(uv.y));
        int px = std::clamp(int(u*w), 0, w-1);
        int py = std::clamp(int(v*h), 0, h-1);
        return data[(py*w + px)*ch] / 255.f;
    }
};

// ----------------------------------------------------------------
// Material
// ----------------------------------------------------------------
struct Material {
    std::string name;
    Vec3  Kd{0.5f,0.5f,0.5f}, Ks{0,0,0}, emission{0,0,0};
    float Ns = 10.f, Tr = 0.f, Ni = 1.5f;
    int   texKd = -1, texD = -1;
    bool  twoSidedEmission = true;
    bool  suppressNonSpecularEmissiveHit = false;

    bool isMirror()   const { return Ks.maxComp()>0.99f && Ns>50000.f && Tr<0.01f; }
    bool isGlass()    const { return Tr>0.01f; }
    bool isEmissive() const { return emission.maxComp()>0.f; }
};

// ----------------------------------------------------------------
// Triangle  (path tracer primitive)
// ----------------------------------------------------------------
struct Triangle {
    Vec3 v0,v1,v2;
    Vec3 n0,n1,n2;
    Vec2 uv0,uv1,uv2;
    int  matId = 0;

    float area() const { return (v1-v0).cross(v2-v0).length()*0.5f; }
};

// ----------------------------------------------------------------
// HitRecord
// ----------------------------------------------------------------
struct HitRecord {
    float t      = PT_INF;
    Vec3  pos, normal, geomNormal;
    Vec2  uv;
    int   matId  = -1;
    bool  front  = true;
};

// ----------------------------------------------------------------
// Camera
// ----------------------------------------------------------------
struct Camera {
    Vec3  eye, lookat, up;
    float fovx   = 60.f;   // degrees
    int   width  = 1280;
    int   height = 720;
};

// ----------------------------------------------------------------
// AreaLight  (XML light descriptor)
// ----------------------------------------------------------------
struct AreaLight {
    std::string mtlname;
    Vec3 radiance;
};

// ----------------------------------------------------------------
// Scene
// ----------------------------------------------------------------
struct Scene {
    Camera                cam;
    std::vector<Material> materials;
    std::vector<Texture>  textures;
    std::vector<Triangle> triangles;

    // Light sampling data (built by buildLightData)
    std::vector<int>   lightTriIdx;
    float              totalLightArea = 0.f;
    std::vector<float> lightCDF;

    Vec3 environmentRadiance{0.0f, 0.0f, 0.0f};
    bool hasEnvironment = false;

    // BVH is declared in bvh.h; forward-declared here as a pointer
    // to avoid circular includes. Defined fully in scene.cpp via Scene::bvh.
    struct BVH* bvhPtr = nullptr; // set by loader after build

    void buildLightData();
};
