#pragma once
#include "../math/vec.h"
#include <vector>
#include <string>
#include <cstdint>

Vec3 tonemap(const Vec3& c);
float computeExposure(const std::vector<Vec3>& fb);

// Convert linear float [0,1] to sRGB uint8
uint8_t toSRGB(float v);

void writePPM(const std::string& path, const std::vector<Vec3>& fb, int w, int h);
void writePNG(const std::string& path, const std::vector<Vec3>& fb, int w, int h);
void writePFM(const std::string& path, const std::vector<Vec3>& fb, int w, int h);
