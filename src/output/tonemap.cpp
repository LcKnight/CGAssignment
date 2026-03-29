#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include "tonemap.h"
#include <cstdint>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <cstdlib>

static float getEnvFloat(const char* name, float fallback) {
    const char* raw = std::getenv(name);
    if(!raw||!*raw) return fallback;
    char* end=nullptr;
    float v = std::strtof(raw,&end);
    return (end==raw||!std::isfinite(v)) ? fallback : v;
}

static bool envEnabled(const char* name) {
    const char* raw = std::getenv(name);
    if(!raw || !*raw) return false;
    return raw[0] != '0';
}

static float luminance(const Vec3& c) {
    return 0.2126f * c.x + 0.7152f * c.y + 0.0722f * c.z;
}

float computeExposure(const std::vector<Vec3>& fb) {
    if(envEnabled("PT_REFERENCE_MODE")) {
        return getEnvFloat("PT_EXPOSURE", 1.0f);
    }

    float manualExposure = getEnvFloat("PT_EXPOSURE", -1.0f);
    if(manualExposure > 0.0f) return manualExposure;

    if(fb.empty()) return 1.0f;

    double logSum = 0.0;
    int validPixels = 0;
    
    // 智能重点测光 (Spot Metering)
    for(const Vec3& c : fb) {
        float lum = luminance(c);

        if (lum > 1e-3f && lum < 200.0f) { 
            logSum += std::log(lum);
            validPixels++;
        }
    }
    
    float logAvgLum = 1.0f;
    if (validPixels > 0) {
        logAvgLum = std::exp(logSum / static_cast<double>(validPixels));
    } else {

        for(const Vec3& c : fb) {
            logSum += std::log(1e-4f + luminance(c));
        }
        logAvgLum = std::exp(logSum / static_cast<double>(fb.size()));
    }

    float targetGray = getEnvFloat("PT_TARGET_GRAY", 0.18f);
    return targetGray / std::max(logAvgLum, 1e-4f);
}

Vec3 tonemap(const Vec3& c) {

    float a = 2.51f;
    float b = 0.03f;
    float c_val = 2.43f;
    float d = 0.59f;
    float e = 0.14f;
    
    auto aces = [&](float x) {
        return (x * (a * x + b)) / (x * (c_val * x + d) + e);
    };

    return {
        std::clamp(aces(c.x), 0.0f, 1.0f),
        std::clamp(aces(c.y), 0.0f, 1.0f),
        std::clamp(aces(c.z), 0.0f, 1.0f)
    };
}

uint8_t toSRGB(float v) {
    v = std::clamp(v, 0.f, 1.f);
    return (uint8_t)(std::pow(v, 1.f/2.2f)*255.f + 0.5f);
}

void writePPM(const std::string& path, const std::vector<Vec3>& fb, int w, int h) {
    float exposure = computeExposure(fb);
    std::ofstream f(path, std::ios::binary);
    f << "P6\n" << w << " " << h << "\n255\n";
    for(int i=0;i<w*h;i++){
        Vec3 c = tonemap(fb[i] * exposure);
        f.put(toSRGB(c.x)); f.put(toSRGB(c.y)); f.put(toSRGB(c.z));
    }
}

void writePNG(const std::string& path, const std::vector<Vec3>& fb, int w, int h) {
    float exposure = computeExposure(fb);
    std::vector<uint8_t> px(w*h*3);
    for(int i=0;i<w*h;i++){
        Vec3 c = tonemap(fb[i] * exposure);
        px[i*3+0]=toSRGB(c.x);
        px[i*3+1]=toSRGB(c.y);
        px[i*3+2]=toSRGB(c.z);
    }
    stbi_write_png(path.c_str(), w, h, 3, px.data(), w*3);
}

void writePFM(const std::string& path, const std::vector<Vec3>& fb, int w, int h) {
    std::ofstream f(path, std::ios::binary);
    f << "PF\n" << w << " " << h << "\n-1.0\n";
    for(int y=h-1; y>=0; --y){
        for(int x=0; x<w; ++x){
            const Vec3& c = fb[y*w + x];
            f.write(reinterpret_cast<const char*>(&c.x), sizeof(float));
            f.write(reinterpret_cast<const char*>(&c.y), sizeof(float));
            f.write(reinterpret_cast<const char*>(&c.z), sizeof(float));
        }
    }
}