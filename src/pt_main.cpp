#include "scene/loader.h"
#include "sampling/light.h"
#include "integrator/pathtracer.h"
#include "output/tonemap.h"
#include "sampling/rng.h"

#include <iostream>
#include <vector>
#include <chrono>
#include <algorithm>
#include <cstdlib>
#include <atomic>
#include <string>

static bool isIntegerArg(const char* s) {
    if(!s || !*s) return false;
    if(*s == '-' || *s == '+') ++s;
    if(!*s) return false;
    while(*s){
        if(*s < '0' || *s > '9') return false;
        ++s;
    }
    return true;
}

static bool setProcessEnv(const std::string& key, const std::string& value){
#ifdef _WIN32
    return _putenv_s(key.c_str(), value.c_str()) == 0;
#else
    return setenv(key.c_str(), value.c_str(), 1) == 0;
#endif
}

static bool unsetProcessEnv(const std::string& key){
#ifdef _WIN32
    return _putenv_s(key.c_str(), "") == 0;
#else
    return unsetenv(key.c_str()) == 0;
#endif
}

#ifdef _OPENMP
#include <omp.h>
#endif

int main(int argc, char** argv) {
    if(argc < 2){
        std::cerr << "Usage: PathTracer <scene_dir> [spp] [--set PT_KEY=VALUE] [--unset PT_KEY]\n";
        return 1;
    }
    std::string sceneDir = argv[1];
    int spp = 64;
    int argStart = 2;
    if(argc > 2 && isIntegerArg(argv[2])){
        spp = std::atoi(argv[2]);
        argStart = 3;
    }

    for(int i = argStart; i < argc; ++i){
        std::string arg = argv[i];
        if(arg == "--set"){
            if(i + 1 >= argc){
                std::cerr << "Missing value after --set\n";
                return 1;
            }
            arg = argv[++i];
        } else if(arg.rfind("--set=", 0) == 0){
            arg = arg.substr(6);
        } else if(arg == "--unset"){
            if(i + 1 >= argc){
                std::cerr << "Missing key after --unset\n";
                return 1;
            }
            std::string key = argv[++i];
            if(!unsetProcessEnv(key)){
                std::cerr << "Failed to unset env: " << key << "\n";
                return 1;
            }
            std::cerr << "[cli-env] unset " << key << "\n";
            continue;
        }

        auto eq = arg.find('=');
        if(eq == std::string::npos){
            std::cerr << "Unknown argument: " << arg << "\n";
            return 1;
        }
        std::string key = arg.substr(0, eq);
        std::string value = arg.substr(eq + 1);
        if(key.empty()){
            std::cerr << "Invalid --set key in argument: " << arg << "\n";
            return 1;
        }
        if(!setProcessEnv(key, value)){
            std::cerr << "Failed to set env: " << key << "\n";
            return 1;
        }
        std::cerr << "[cli-env] " << key << "=" << value << "\n";
    }

    SceneFull scene;
    loadScene(sceneDir, scene);

    int W = scene.cam.width, H = scene.cam.height;
    std::vector<Vec3> fb(W*H, {0,0,0});

    std::cerr << "Rendering " << W << "x" << H << " @ " << spp << " spp...\n";
    auto t0 = std::chrono::high_resolution_clock::now();

    const int totalPixels = W * H;
    std::atomic<int> donePixels{0};
    const int progressStep = std::max(1, totalPixels / 20);

    #pragma omp parallel for collapse(2) schedule(guided, 32)
    for(int y=0; y<H; y++){
        for(int x=0; x<W; x++){
            uint32_t rng = (uint32_t)(y*1973 + x*9277 + 6947);
            for(int i=0;i<8;i++) rng ^= rng<<13, rng ^= rng>>17, rng ^= rng<<5;

            Vec3 col(0,0,0);
            for(int s=0; s<spp; s++){
                float u1 = randf(rng);
                float u2 = randf(rng);
                Ray ray = genCameraRay(scene.cam, x, y, u1, u2);
                col += pathTrace(ray, scene, rng, x, y, s);
            }
            fb[y*W+x] = col / (float)spp;

            int finished = donePixels.fetch_add(1, std::memory_order_relaxed) + 1;
            if(finished % progressStep == 0 || finished == totalPixels){
                #pragma omp critical
                {
                    int percent = (100 * finished) / totalPixels;
                    std::cerr << "\rProgress: " << finished << "/" << totalPixels
                              << " (" << percent << "%)" << std::flush;
                }
            }
        }
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    double sec = std::chrono::duration<double>(t1-t0).count();
    std::cerr << "\nDone in " << sec << "s\n";

    // Output filename derived from scene directory name
    std::string base = sceneDir;
    while(!base.empty() && (base.back()=='/'||base.back()=='\\')) base.pop_back();
    auto pos = base.find_last_of("/\\");
    if(pos!=std::string::npos) base = base.substr(pos+1);

    writePPM(base+".ppm", fb, W, H);
    writePNG(base+".png", fb, W, H);
    const char* refMode = std::getenv("PT_REFERENCE_MODE");
    const char* writeLinear = std::getenv("PT_WRITE_LINEAR");
    if((refMode && refMode[0] != '0') || (writeLinear && writeLinear[0] != '0')) {
        writePFM(base+".linear.pfm", fb, W, H);
    }
    std::cerr << "Saved " << base << ".ppm and " << base << ".png\n";
    return 0;
}
