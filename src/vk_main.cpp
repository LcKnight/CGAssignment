#include "vulkan/app.h"
#include <iostream>
#include <string>
#include <algorithm>

int main(int argc, char** argv) {
    HelloVulkanApplication app;
    const char* objFile = nullptr;
    int  mode      = 1;
    int  scenario  = 0;
    bool benchmark = false;
    bool modeSet   = false;

    for(int i=1;i<argc;i++){
        std::string arg=argv[i], lo=arg;
        std::transform(lo.begin(),lo.end(),lo.begin(),::tolower);
        if(lo=="benchmarkmode"||lo=="yes"||lo=="true"||lo=="on"){benchmark=true;continue;}
        if(lo=="no"||lo=="false"||lo=="off"){benchmark=false;continue;}
        if(arg.find(".obj")!=std::string::npos){objFile=argv[i];continue;}
        try{int v=std::stoi(arg);if(!modeSet&&v>=1&&v<=5){mode=v;modeSet=true;}else scenario=v;}catch(...){}
    }
    if(!objFile){
        std::cout<<"[Warning] No .obj file specified, using default.\n";
        std::cout<<"Usage: VulkanApp model.obj [mode] [scenario] [yes/no]\n";
    }
    try{ app.run(objFile, mode, scenario, benchmark); }
    catch(const std::exception& e){ std::cerr<<"FATAL ERROR: "<<e.what()<<"\n"; return 1; }
    return 0;
}
