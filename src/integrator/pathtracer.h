#pragma once
#include "../scene/loader.h"
#include <cstdint>

// Trace one path from the given ray and return its radiance estimate.
Vec3 pathTrace(const Ray& ray, const SceneFull& sc, uint32_t& rng,
               int pixelX = -1, int pixelY = -1, int sampleIndex = -1);
