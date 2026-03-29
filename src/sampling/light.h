#pragma once
#include "../scene/mesh.h"
#include "../scene/bvh.h"
#include <cstdint>

struct Camera;
struct Ray;
Ray genCameraRay(const Camera& cam, int px, int py, float u1, float u2);

// Area-light NEE with MIS (power heuristic vs BRDF pdf).
Vec3 sampleDirectLight(const HitRecord& rec, const Vec3& wo,
                       const Vec3& Kd, const Vec3& Ks, float Ns,
                       const BVH& bvh,
                       const std::vector<Triangle>& tris,
                       const std::vector<Texture>&  texs,
                       const std::vector<Material>& mats,
                       const std::vector<int>& lightTriIdx,
                       float totalLightArea,
                       uint32_t& rng);

// Solid-angle PDF for having sampled a direction that hits lightRec via area sampling.
float lightPdfSolidAngle(const Vec3& shadingPos, const HitRecord& lightRec,
                         float totalLightArea, bool twoSidedLight);

// Constant-environment NEE with MIS (power heuristic vs BRDF pdf).
Vec3 sampleDirectEnvironment(const HitRecord& rec, const Vec3& wo,
                             const Vec3& Kd, const Vec3& Ks, float Ns,
                             const Vec3& environmentRadiance,
                             const BVH& bvh,
                             const std::vector<Triangle>& tris,
                             const std::vector<Texture>& texs,
                             const std::vector<Material>& mats,
                             uint32_t& rng);

// Uniform-sphere environment-light PDF used by sampleDirectEnvironment.
float environmentLightPdf();
