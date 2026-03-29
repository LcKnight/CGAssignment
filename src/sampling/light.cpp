#include "light.h"
#include "brdf.h"
#include "rng.h"
#include "../math/ray.h"
#include <cmath>
#include <algorithm>

static constexpr float PI  = PT_PI;
static constexpr float EPS = PT_EPS;

static Vec3 sampleUniformSphere(float u1, float u2) {
    float z = 1.0f - 2.0f * u1;
    float r = std::sqrt(std::max(0.0f, 1.0f - z * z));
    float phi = 2.0f * PI * u2;
    return {r * std::cos(phi), r * std::sin(phi), z};
}

float environmentLightPdf() {
    return 1.0f / (4.0f * PI);
}

// ----------------------------------------------------------------
// Camera ray
// ----------------------------------------------------------------
Ray genCameraRay(const Camera& cam, int px, int py, float u1, float u2) {
    Vec3  fwd   = (cam.lookat - cam.eye).normalize();
    Vec3  right = fwd.cross(cam.up).normalize();
    Vec3  up    = right.cross(fwd);
    float aspect = (float)cam.width / cam.height;
    float halfW  = std::tan(cam.fovx * PI / 360.f);
    float halfH  = halfW / aspect;
    float sx = (2.f*(px+u1)/cam.width  - 1.f)*halfW;
    float sy = (1.f - 2.f*(py+u2)/cam.height)*halfH;
    Vec3 dir = (fwd + right*sx + up*sy).normalize();
    return {cam.eye, dir};
}

// ----------------------------------------------------------------
// lightPdfSolidAngle
// pdf_area = 1 / totalLightArea
// convert to solid angle: pdf_omega = pdf_area * dist2 / cosL
// ----------------------------------------------------------------
float lightPdfSolidAngle(const Vec3& shadingPos, const HitRecord& lightRec,
                         float totalLightArea, bool twoSidedLight) {
    if(totalLightArea <= 0.f) return 0.f;
    Vec3  toLight = lightRec.pos - shadingPos;
    float dist2   = toLight.dot(toLight);
    if(dist2 < 1e-10f) return 0.f;
    float ndotl = -lightRec.geomNormal.dot(toLight * (1.f / std::sqrt(dist2)));
    float cosL = twoSidedLight ? std::abs(ndotl) : std::max(0.0f, ndotl);
    if(cosL < 1e-6f) return 0.f;
    return dist2 / (cosL * totalLightArea);
}

Vec3 sampleDirectLight(const HitRecord& rec, const Vec3& wo,
                       const Vec3& Kd, const Vec3& Ks, float Ns,
                       const BVH& bvh,
                       const std::vector<Triangle>& tris,
                       const std::vector<Texture>&  texs,
                       const std::vector<Material>& mats,
                       const std::vector<int>& lightTriIdx,
                       float totalLightArea,
                       uint32_t& rng)
{
    if(lightTriIdx.empty() || totalLightArea <= 0.f) return {0,0,0};

    // Skip enclosing-shell lights from NEE — their huge area causes
    // pdf_light = dist²/(cosL×area) to be tiny → firefly spikes.
    // Shell contributes via specularBounce emissive hit path instead.
    float effectiveArea = 0.f;
    for(int li : lightTriIdx)
        if(!mats[tris[li].matId].suppressNonSpecularEmissiveHit)
            effectiveArea += tris[li].area();
    if(effectiveArea <= 0.f) return {0,0,0};

    float ru = randf(rng) * effectiveArea;
    float cum = 0.f;
    int chosenIdx = -1;
    int lastValid = -1;
    for(int li : lightTriIdx) {
        if(mats[tris[li].matId].suppressNonSpecularEmissiveHit) continue;
        lastValid = li;
        cum += tris[li].area();
        if(ru <= cum) { chosenIdx = li; break; }
    }
    if(chosenIdx < 0) chosenIdx = lastValid;  // floating-point fallback
    if(chosenIdx < 0) return {0,0,0};
    const Triangle& lightTri = tris[chosenIdx];
    const Material& lightMat = mats[lightTri.matId];
    // Use effectiveArea for pdf (excludes shell)
    const float sampledLightArea = effectiveArea;

    float r1 = randf(rng), r2 = randf(rng);
    float sq = std::sqrt(r1);
    float bary0 = 1.f - sq;
    float bary1 = sq * (1.f - r2);
    float bary2 = sq * r2;
    Vec3 lightPos = lightTri.v0*bary0 + lightTri.v1*bary1 + lightTri.v2*bary2;
    Vec3 lightN   = (lightTri.v1-lightTri.v0).cross(lightTri.v2-lightTri.v0).normalize();

    Vec3  toLight = lightPos - rec.pos;
    float dist2   = toLight.dot(toLight);
    float dist    = std::sqrt(dist2);
    if(dist < EPS) return {0,0,0};
    Vec3 wi = toLight * (1.f / dist);

    float cosGeoS = wi.dot(rec.geomNormal);
    if(cosGeoS <= 0.f) return {0,0,0};

    float cosS = wi.dot(rec.normal);
    if(cosS <= 0.f) return {0,0,0};

    float ndotl = (-wi).dot(lightN);
    float cosL = lightMat.twoSidedEmission ? std::abs(ndotl) : std::max(0.0f, ndotl);
    if(cosL <= 0.f) return {0,0,0};

    if(bvh.occluded({rec.pos, wi}, tris, texs, mats, EPS, dist - EPS))
        return {0,0,0};

    float pdf_light = dist2 / (cosL * sampledLightArea);
    if(pdf_light < 1e-8f) return {0,0,0};

    float pdf_brdf = pdfBRDF(Kd, Ks, Ns, rec.normal, wo, wi);
    float w_light = pdf_light * pdf_light / (pdf_light*pdf_light + pdf_brdf*pdf_brdf + 1e-10f);

    Vec3 Le = mats[lightTri.matId].emission;
    Vec3 brdf = evalBRDF(Kd, Ks, Ns, rec.normal, wo, wi);
    return Le * brdf * (cosS * w_light / pdf_light);
}

Vec3 sampleDirectEnvironment(const HitRecord& rec, const Vec3& wo,
                             const Vec3& Kd, const Vec3& Ks, float Ns,
                             const Vec3& environmentRadiance,
                             const BVH& bvh,
                             const std::vector<Triangle>& tris,
                             const std::vector<Texture>& texs,
                             const std::vector<Material>& mats,
                             uint32_t& rng)
{
    if(environmentRadiance.maxComp() <= 0.0f) return {0,0,0};

    const float pdf_light = environmentLightPdf();
    Vec3 wi = sampleUniformSphere(randf(rng), randf(rng));

    float cosGeoS = wi.dot(rec.geomNormal);
    if(cosGeoS <= 0.0f) return {0,0,0};

    float cosS = wi.dot(rec.normal);
    if(cosS <= 0.0f) return {0,0,0};

    if(bvh.occluded({rec.pos, wi}, tris, texs, mats, EPS, PT_INF))
        return {0,0,0};

    float pdf_brdf = pdfBRDF(Kd, Ks, Ns, rec.normal, wo, wi);
    float w_light = pdf_light * pdf_light /
                    (pdf_light * pdf_light + pdf_brdf * pdf_brdf + 1e-10f);

    Vec3 brdf = evalBRDF(Kd, Ks, Ns, rec.normal, wo, wi);
    return environmentRadiance * brdf * (cosS * w_light / pdf_light);
}
