#include "pathtracer.h"
#include "../sampling/brdf.h"
#include "../sampling/light.h"
#include "../sampling/rng.h"
#include <cmath>
#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <string>
#include <cstdio>

static constexpr float PI  = PT_PI;
static constexpr float EPS = PT_EPS;
static constexpr float INF = PT_INF;

static bool envEnabled(const char* name) {
    const char* raw = std::getenv(name);
    if(!raw || !*raw) return false;
    return raw[0] != '0';
}

static float envFloat(const char* name, float fallback) {
    const char* raw = std::getenv(name);
    if(!raw || !*raw) return fallback;
    char* end = nullptr;
    float v = std::strtof(raw, &end);
    if(end == raw) return fallback;
    return v;
}

struct DebugSettings {
    bool enabled = false;
    int pixelX = -1;
    int pixelY = -1;
    int maxSamples = 2;
    int maxBounces = 12;
};

static const DebugSettings& getDebugSettings() {
    static DebugSettings s;
    static bool initialized = false;
    if(!initialized){
        initialized = true;
        const char* raw = std::getenv("PT_DEBUG_PIXEL");
        if(raw && *raw){
            int x = -1, y = -1;
            if(std::sscanf(raw, "%d,%d", &x, &y) == 2 || std::sscanf(raw, "%d %d", &x, &y) == 2){
                s.enabled = true;
                s.pixelX = x;
                s.pixelY = y;
            }
        }
        int ms = (int)envFloat("PT_DEBUG_MAX_SAMPLES", 2.0f);
        int mb = (int)envFloat("PT_DEBUG_MAX_BOUNCES", 12.0f);
        s.maxSamples = std::max(1, ms);
        s.maxBounces = std::max(1, mb);
    }
    return s;
}

static std::mutex gDebugTraceMutex;

static void debugTraceLog(bool active, const std::string& line) {
    if(!active) return;
    std::lock_guard<std::mutex> lock(gDebugTraceMutex);
    std::cerr << line << "\n";
}

Vec3 pathTrace(const Ray& ray, const SceneFull& sc, uint32_t& rng,
               int pixelX, int pixelY, int sampleIndex) {
    Vec3  throughput(1,1,1);
    Vec3  color(0,0,0);
    Ray   r = ray;
    bool  specularBounce = true;
    Vec3  prevPos{0,0,0};
    float prevBrdfPdf = 1.f;
    const bool onlyNee = envEnabled("PT_ONLY_NEE");
    const bool onlyEmissiveHit = envEnabled("PT_ONLY_EMISSIVE_HIT");
    const DebugSettings& dbg = getDebugSettings();
    bool debugActive = dbg.enabled
                    && pixelX == dbg.pixelX
                    && pixelY == dbg.pixelY
                    && sampleIndex >= 0
                    && sampleIndex < dbg.maxSamples;

    if(debugActive){
        debugTraceLog(true,
            "[DBG] pixel(" + std::to_string(pixelX) + "," + std::to_string(pixelY) + ") sample=" + std::to_string(sampleIndex)
            + " rayO=(" + std::to_string(r.o.x) + "," + std::to_string(r.o.y) + "," + std::to_string(r.o.z) + ")"
            + " rayD=(" + std::to_string(r.d.x) + "," + std::to_string(r.d.y) + "," + std::to_string(r.d.z) + ")");
    }

    for(int depth=0; depth<32; depth++){
        if(debugActive && depth >= dbg.maxBounces){
            debugTraceLog(true, "[DBG] depth limit reached by PT_DEBUG_MAX_BOUNCES");
            break;
        }

        HitRecord rec;
        if(!sc.bvh.intersect(r, sc.triangles, sc.textures, sc.materials,
                              EPS, INF, rec)){
            if(sc.hasEnvironment){
                Vec3 add;
                if(specularBounce){
                    add = throughput * sc.environmentRadiance;
                } else {
                    float pdf_light = environmentLightPdf();
                    float w_brdf = prevBrdfPdf * prevBrdfPdf /
                                   (prevBrdfPdf * prevBrdfPdf + pdf_light * pdf_light + 1e-10f);
                    add = throughput * sc.environmentRadiance * w_brdf;
                }
                color += add;
                if(debugActive) debugTraceLog(true,
                    "[DBG] depth=" + std::to_string(depth) + " miss -> env add=("
                    + std::to_string(add.x) + ","
                    + std::to_string(add.y) + ","
                    + std::to_string(add.z) + ")");
            }
            break;
        }

        const Material& mat = sc.materials[rec.matId];
        // ==========================================
        // Alpha Test (透明度遮罩裁剪)
        // ==========================================
        if (mat.texD >= 0 && sc.textures[mat.texD].valid()) {  
            Vec3 mask = sc.textures[mat.texD].sample(rec.uv);
            if (mask.x < 0.5f) {                               
                r.o = rec.pos + r.d * (EPS * 2.0f); 
                depth--; 
                continue; 
            }                                                  
        }
        Vec3 wo = -r.d;
        if(debugActive) debugTraceLog(true,
            "[DBG] depth=" + std::to_string(depth) + " hit mat='" + mat.name
            + "' matId=" + std::to_string(rec.matId)
            + " front=" + std::to_string(rec.front ? 1 : 0)
            + " t=" + std::to_string(rec.t)
            + " throughput=(" + std::to_string(throughput.x) + ","
            + std::to_string(throughput.y) + ","
            + std::to_string(throughput.z) + ")");

        // --- Emissive hit ---
        if(mat.isEmissive()){
            if(onlyNee) {
                break;
            }
            if(envEnabled("PT_DISABLE_EMISSIVE_HIT")) {
                break;
            }
            if(!mat.twoSidedEmission && !rec.front){
                break;
            }
            if(mat.suppressNonSpecularEmissiveHit && !specularBounce){
                break;
            }
            if(mat.suppressNonSpecularEmissiveHit && depth == 0){
                break;
            }
            float shellSpecScale = 1.0f;
            if(mat.suppressNonSpecularEmissiveHit && specularBounce){
                shellSpecScale = envFloat("PT_SHELL_SPECULAR_HIT_SCALE", 1.0f);
                if(shellSpecScale < 0.0f) shellSpecScale = 0.0f;
                if(shellSpecScale > 1.0f) shellSpecScale = 1.0f;
            }
            if(specularBounce){
                Vec3 add = throughput * mat.emission * shellSpecScale;
                color += add;
                debugTraceLog(debugActive,
                    "[DBG] depth=" + std::to_string(depth) + " emissive(spec) scale=" + std::to_string(shellSpecScale)
                    + " add=(" + std::to_string(add.x) + ","
                    + std::to_string(add.y) + ","
                    + std::to_string(add.z) + ")");
            } else {
                float pdf_light = lightPdfSolidAngle(prevPos, rec, sc.totalLightArea, mat.twoSidedEmission);
                float w_brdf = prevBrdfPdf * prevBrdfPdf /
                               (prevBrdfPdf*prevBrdfPdf + pdf_light*pdf_light + 1e-10f);
                Vec3 add = throughput * mat.emission * shellSpecScale * w_brdf;
                color += add;
                debugTraceLog(debugActive,
                    "[DBG] depth=" + std::to_string(depth) + " emissive(diffuse-hit) scale=" + std::to_string(shellSpecScale)
                    + " pdf_light=" + std::to_string(pdf_light)
                    + " prevBrdfPdf=" + std::to_string(prevBrdfPdf)
                    + " w_brdf=" + std::to_string(w_brdf)
                    + " add=(" + std::to_string(add.x) + ","
                    + std::to_string(add.y) + ","
                    + std::to_string(add.z) + ")");
            }
            break;
        }

        // --- Texture lookup ---
        Vec3 Kd = mat.Kd;
        if(mat.texKd>=0 && sc.textures[mat.texKd].valid())
            Kd = sc.textures[mat.texKd].sample(rec.uv);
        Vec3 Ks = mat.Ks;

        // Energy conservation
        Kd = {std::clamp(Kd.x,0.f,1.f), std::clamp(Kd.y,0.f,1.f), std::clamp(Kd.z,0.f,1.f)};
        Ks = {std::clamp(Ks.x,0.f,1.f), std::clamp(Ks.y,0.f,1.f), std::clamp(Ks.z,0.f,1.f)};
        Kd = {std::min(Kd.x,1.f-Ks.x), std::min(Kd.y,1.f-Ks.y), std::min(Kd.z,1.f-Ks.z)};

        // Ultra-glossy non-transmissive materials are very high variance at low spp
        // under Phong lobe sampling. Route them through a deterministic specular
        // reflection step to avoid near-black frames/trim at 16 spp.
        if(mat.Tr < 0.01f && mat.Ns > 3000.0f && Ks.maxComp() > 0.1f){
            Vec3 dir = reflect(r.d, rec.normal);
            float side = dir.dot(rec.geomNormal) >= 0.0f ? 1.0f : -1.0f;
            Vec3 origin = rec.pos + rec.geomNormal * (side * EPS * 2.0f);
            r = {origin, dir};
            throughput = throughput * Ks;
            specularBounce = true;
            prevPos = rec.pos;
            continue;
        }

        // --- Mirror ---
        if(mat.isMirror()){
            Vec3 dir = reflect(r.d, rec.normal);
            float side = dir.dot(rec.geomNormal) >= 0.0f ? 1.0f : -1.0f;
            Vec3 origin = rec.pos + rec.geomNormal * (side * EPS * 2.0f);
            r = {origin, dir};
            throughput = throughput * mat.Ks;
            specularBounce = true;
            prevPos = rec.pos;
            continue;
        }

        // --- Glass ---
        if(mat.isGlass()){
            Vec3  refracted;
            Vec3  n    = rec.front ? rec.normal : -rec.normal;
            float cosI = std::abs((-r.d).dot(n));
            float eta  = rec.front ? (1.f / mat.Ni) : mat.Ni;
            bool  tir  = !refract(r.d, n, eta, refracted);
            float fr   = tir ? 1.f : schlick(std::abs(cosI), mat.Ni);
            if(tir || randf(rng)<fr){
                Vec3 dir = reflect(r.d, rec.normal);
                float side = dir.dot(rec.geomNormal) >= 0.0f ? 1.0f : -1.0f;
                Vec3 origin = rec.pos + rec.geomNormal * (side * EPS * 2.0f);
                r = {origin, dir};
            } else {
                float side = refracted.dot(rec.geomNormal) >= 0.0f ? 1.0f : -1.0f;
                Vec3 origin = rec.pos + rec.geomNormal * (side * EPS * 2.0f);
                r = {origin, refracted};
                throughput = throughput * mat.Tr;
            }
            specularBounce = true;
            prevPos = rec.pos;
            continue;
        }

        if(!onlyEmissiveHit && !envEnabled("PT_DISABLE_NEE") && !sc.lightTriIdx.empty()){
            Vec3 direct = throughput * sampleDirectLight(
                rec, wo, Kd, Ks, mat.Ns,
                sc.bvh, sc.triangles, sc.textures, sc.materials,
                sc.lightTriIdx, sc.totalLightArea, rng);
            color += direct;
            if(debugActive && direct.maxComp() > 0.0f){
                debugTraceLog(true,
                    "[DBG] depth=" + std::to_string(depth) + " directNEE add=("
                    + std::to_string(direct.x) + ","
                    + std::to_string(direct.y) + ","
                    + std::to_string(direct.z) + ")");
            }
        }

        if(!onlyEmissiveHit && !envEnabled("PT_DISABLE_NEE") && sc.hasEnvironment){
            Vec3 directEnv = throughput * sampleDirectEnvironment(
                rec, wo, Kd, Ks, mat.Ns,
                sc.environmentRadiance,
                sc.bvh, sc.triangles, sc.textures, sc.materials,
                rng);
            color += directEnv;
            if(debugActive && directEnv.maxComp() > 0.0f){
                debugTraceLog(true,
                    "[DBG] depth=" + std::to_string(depth) + " directENV add=("
                    + std::to_string(directEnv.x) + ","
                    + std::to_string(directEnv.y) + ","
                    + std::to_string(directEnv.z) + ")");
            }
        }

        if(onlyNee){
            break;
        }

        float pCont = 1.f;
        if(depth>5){
            pCont = std::min((Kd+Ks).maxComp(), 0.95f);
            pCont = std::max(pCont, 0.3f);
            if(randf(rng)>pCont) break;
        }

        Vec3  wi; float pdf;
        Vec3  brdf = sampleBRDF(Kd, Ks, mat.Ns, rec.normal, wo, wi, pdf, rng);
        if(pdf < 1e-8f) break;
        if(wi.dot(rec.geomNormal) <= 0.f) break;

        float cosT = std::max(0.0f, wi.dot(rec.normal));
        if(cosT <= 0.f) break;
        debugTraceLog(debugActive,
            "[DBG] depth=" + std::to_string(depth) + " scatter pdf=" + std::to_string(pdf)
            + " cosT=" + std::to_string(cosT)
            + " wi=(" + std::to_string(wi.x) + ","
            + std::to_string(wi.y) + ","
            + std::to_string(wi.z) + ")");
        throughput = throughput * brdf * cosT / (pdf * pCont);
        prevPos     = rec.pos;
        prevBrdfPdf = pdf;
        r = {rec.pos, wi};
        specularBounce = false;
    }
    return color;
}
