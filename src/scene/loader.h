#pragma once
#include "mesh.h"
#include "bvh.h"
#include <string>

// Full scene (owns BVH by value)
struct SceneFull : public Scene {
    BVH bvh;
};

Texture    loadTexture(const std::string& path);
void       loadScene(const std::string& dir, SceneFull& scene);