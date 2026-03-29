#define NOMINMAX
#include "app.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <set>
#include <limits>
#include <algorithm>
#include <cstring>
#include <cmath>
#include <iomanip>
#include <atomic>

#ifdef _WIN32
  #include <windows.h>
#elif defined(__linux__)
  #include <unistd.h>
  #include <limits.h>
#endif
#ifdef _OPENMP
  #include <omp.h>
#endif

// ----------------------------------------------------------------
// Module-level globals (mirroring the originals)
// ----------------------------------------------------------------
static constexpr uint32_t WIDTH  = 1920;
static constexpr uint32_t HEIGHT = 1080;
static constexpr int MAX_FRAMES_IN_FLIGHT = 2;

static bool  keys[1024]    = {false};
static float cameraX = 0.f, cameraY = 0.f, cameraZ = 3.5f;
static float lastX   = WIDTH / 2.f, lastY = HEIGHT / 2.f;
static bool  firstMouse    = true;
static bool  leftMouseDown = false;
static bool  rightMouseDown= false;
static int   currentAxis   = 1;
static Vec3  modelRotation = {180.f, 0.f, 0.f};
static bool  autoRotate    = true;
static int   g_renderMode  = 1;
static std::atomic<uint64_t> g_num_fragments{0};
static std::vector<unsigned char> pixels(WIDTH * HEIGHT * 4);
static std::vector<float>         zBuffer(WIDTH * HEIGHT);
static Mesh g_mesh;

static const std::vector<const char*> validationLayers = {"VK_LAYER_KHRONOS_validation"};
static const std::vector<const char*> deviceExtensions = {VK_KHR_SWAPCHAIN_EXTENSION_NAME};
#ifdef NDEBUG
static constexpr bool enableValidationLayers = false;
#else
static constexpr bool enableValidationLayers = true;
#endif

// ----------------------------------------------------------------
// Simple shading helper
// ----------------------------------------------------------------
static Vec3 simple_shading(const Vec3& normal) {
    Vec3 ambient  = {0.1f, 0.1f, 0.1f};
    Vec3 lightDir1= Vec3{1.f,1.f,1.f}.normalize();
    float diff1   = std::max(normal.dot(lightDir1), 0.f);
    Vec3 lightDir2= Vec3{-1.f,0.5f,-1.f}.normalize();
    float diff2   = std::max(normal.dot(lightDir2), 0.f);
    return {ambient.x + 0.8f*diff1 + 0.3f*diff2,
            ambient.y + 0.8f*diff1 + 0.3f*diff2,
            ambient.z + 0.8f*diff1 + 0.4f*diff2};
}

// ----------------------------------------------------------------
// File helpers
// ----------------------------------------------------------------
static std::string getExecutableDir() {
#ifdef _WIN32
    char buf[MAX_PATH]; GetModuleFileNameA(NULL, buf, MAX_PATH);
    std::string p(buf); size_t pos = p.find_last_of("/\\");
    return (pos==std::string::npos)?"":p.substr(0,pos);
#elif defined(__linux__)
    char buf[PATH_MAX]; ssize_t len=readlink("/proc/self/exe",buf,sizeof(buf)-1);
    if(len!=-1){buf[len]='\0'; std::string p(buf); size_t pos=p.find_last_of("/\\"); return(pos==std::string::npos)?"":p.substr(0,pos);}
    return ".";
#else
    return ".";
#endif
}

std::vector<char> HelloVulkanApplication::readFile(const std::string& filename) {
    std::ifstream file(filename, std::ios::ate|std::ios::binary);
    if(!file.is_open()) throw std::runtime_error("failed to open file: "+filename);
    size_t sz=(size_t)file.tellg(); std::vector<char> buf(sz);
    file.seekg(0); file.read(buf.data(),sz); file.close(); return buf;
}

// ================================================================
// Mesh
// ================================================================
int Mesh::buildBVHRecursive(std::vector<int>& triIndices, int depth) {
    BVHNode node;
    node.minB={1e30f,1e30f,1e30f}; node.maxB={-1e30f,-1e30f,-1e30f};
    for(int ti:triIndices)
        for(int k=0;k<3;k++){
            Vec3 v=vertices[faces[ti*3+k]];
            if(v.x<node.minB.x)node.minB.x=v.x; if(v.x>node.maxB.x)node.maxB.x=v.x;
            if(v.y<node.minB.y)node.minB.y=v.y; if(v.y>node.maxB.y)node.maxB.y=v.y;
            if(v.z<node.minB.z)node.minB.z=v.z; if(v.z>node.maxB.z)node.maxB.z=v.z;
        }
    if(triIndices.size()<=16||depth>20){
        node.triStart=(int)sortedFaces.size()/3; node.triCount=(int)triIndices.size();
        for(int ti:triIndices){sortedFaces.push_back(faces[ti*3]);sortedFaces.push_back(faces[ti*3+1]);sortedFaces.push_back(faces[ti*3+2]);}
        bvhNodes.push_back(node); return (int)bvhNodes.size()-1;
    }
    Vec3 sz={node.maxB.x-node.minB.x, node.maxB.y-node.minB.y, node.maxB.z-node.minB.z};
    int axis=0; if(sz.y>sz.x)axis=1; if(sz.z>sz.y&&sz.z>sz.x)axis=2;
    float mid=((axis==0?node.minB.x:axis==1?node.minB.y:node.minB.z)+(axis==0?node.maxB.x:axis==1?node.maxB.y:node.maxB.z))*0.5f;
    std::vector<int> L,R;
    for(int ti:triIndices){
        float c=0; for(int k=0;k<3;k++) c+=(axis==0?vertices[faces[ti*3+k]].x:axis==1?vertices[faces[ti*3+k]].y:vertices[faces[ti*3+k]].z);
        c/=3.f; (c<mid?L:R).push_back(ti);
    }
    if(L.empty()||R.empty()){
        node.triStart=(int)sortedFaces.size()/3; node.triCount=(int)triIndices.size();
        for(int ti:triIndices){sortedFaces.push_back(faces[ti*3]);sortedFaces.push_back(faces[ti*3+1]);sortedFaces.push_back(faces[ti*3+2]);}
        bvhNodes.push_back(node); return (int)bvhNodes.size()-1;
    }
    int cur=(int)bvhNodes.size(); bvhNodes.push_back(node);
    int lIdx=buildBVHRecursive(L,depth+1), rIdx=buildBVHRecursive(R,depth+1);
    bvhNodes[cur].left=lIdx; bvhNodes[cur].right=rIdx; return cur;
}

void Mesh::buildBVH(){
    if(vertices.empty()) return;
    std::cout<<"Building BVH...\n";
    bvhNodes.clear(); bvhNodes.reserve(faces.size()/2);
    sortedFaces.clear(); sortedFaces.reserve(faces.size());
    std::vector<int> all(faces.size()/3); for(int i=0;i<(int)all.size();i++) all[i]=i;
    buildBVHRecursive(all,0); faces=sortedFaces;
    std::cout<<"BVH Built. Nodes: "<<bvhNodes.size()<<"\n";
}

Mesh Mesh::loadObj(const std::string& filename){
    Mesh mesh; std::ifstream file(filename);
    Vec3 minB={1e30f,1e30f,1e30f}, maxB={-1e30f,-1e30f,-1e30f};
    if(!file.is_open()) throw std::runtime_error("FATAL: Failed to open OBJ: "+filename);
    std::string line;
    while(std::getline(file,line)){
        std::stringstream ss(line); std::string prefix; ss>>prefix;
        if(prefix=="v"){
            Vec3 v; ss>>v.x>>v.y>>v.z; mesh.vertices.push_back(v);
            if(v.x<minB.x)minB.x=v.x; if(v.x>maxB.x)maxB.x=v.x;
            if(v.y<minB.y)minB.y=v.y; if(v.y>maxB.y)maxB.y=v.y;
            if(v.z<minB.z)minB.z=v.z; if(v.z>maxB.z)maxB.z=v.z;
        } else if(prefix=="f"){
            std::string seg; for(int i=0;i<3;i++){ss>>seg; size_t sl=seg.find('/'); mesh.faces.push_back(std::stoi(seg.substr(0,sl))-1);}
        }
    }
    mesh.minY=minB.y; mesh.maxY=maxB.y;
    if(!mesh.vertices.empty()){
        Vec3 ctr={(minB.x+maxB.x)*0.5f,(minB.y+maxB.y)*0.5f,(minB.z+maxB.z)*0.5f};
        mesh.centerOffset={-ctr.x,-ctr.y,-ctr.z};
        float maxDim=std::max({maxB.x-minB.x,maxB.y-minB.y,maxB.z-minB.z});
        if(maxDim>0) mesh.normalizeScale=1.74f/maxDim;
        std::cout<<"Model Bounds: "<<minB.y<<" to "<<maxB.y<<"\n";
    }
    mesh.buildBVH(); return mesh;
}

// ================================================================
// HelloVulkanApplication
// ================================================================
void HelloVulkanApplication::run(const char* objFilename, int mode, int scenario, bool isBenchmark) {
    g_renderMode=mode; benchmarkScenario=scenario; benchmarkMode=isBenchmark;
    std::cout<<"--------------------------------------------------\n";
    std::cout<<"Config: Mode="<<g_renderMode<<" | Scenario="<<benchmarkScenario<<" | Benchmark="<<(benchmarkMode?"ON":"OFF")<<"\n";
    std::cout<<"--------------------------------------------------\n";
    if(objFilename) g_mesh=Mesh::loadObj(objFilename); else g_mesh=Mesh::loadObj("dummy.obj");
    initWindow(); initVulkan();
    frameTimeHistory.reserve(TOTAL_TEST_FRAMES);
    startTime=std::chrono::high_resolution_clock::now();
    mainLoop(); cleanup();
}

// ---- GLFW callbacks ----
void HelloVulkanApplication::framebufferResizeCallback(GLFWwindow* w,int,int){
    auto app=reinterpret_cast<HelloVulkanApplication*>(glfwGetWindowUserPointer(w));
    app->framebufferResized=true;
}
void HelloVulkanApplication::keyCallback(GLFWwindow*,int key,int,int action,int){
    if(key>=0&&key<1024) keys[key]=(action!=GLFW_RELEASE);
    if(key==GLFW_KEY_LEFT_CONTROL&&action==GLFW_PRESS) currentAxis=(currentAxis+1)%3;
    if(key==GLFW_KEY_SPACE&&action==GLFW_PRESS) autoRotate=!autoRotate;
    if(key==GLFW_KEY_1&&action==GLFW_PRESS) g_renderMode=1;
    if(key==GLFW_KEY_2&&action==GLFW_PRESS) g_renderMode=2;
    if(key==GLFW_KEY_3&&action==GLFW_PRESS) g_renderMode=3;
}
void HelloVulkanApplication::scrollCallback(GLFWwindow*,double,double y){
    cameraZ-=(float)y*0.2f; if(cameraZ<0.1f)cameraZ=0.1f; if(cameraZ>20.f)cameraZ=20.f;
}
void HelloVulkanApplication::mouseButtonCallback(GLFWwindow*,int btn,int action,int){
    if(btn==GLFW_MOUSE_BUTTON_LEFT){leftMouseDown=(action==GLFW_PRESS); if(leftMouseDown)autoRotate=false;}
    if(btn==GLFW_MOUSE_BUTTON_RIGHT) rightMouseDown=(action==GLFW_PRESS);
}
void HelloVulkanApplication::cursorPosCallback(GLFWwindow*,double xp,double yp){
    float x=(float)xp, y=(float)yp;
    if(firstMouse){lastX=x;lastY=y;firstMouse=false;}
    float dx=x-lastX; lastX=x; lastY=y;
    if(leftMouseDown){
        float d=dx*0.5f;
        if(currentAxis==0)modelRotation.x+=d;
        else if(currentAxis==1)modelRotation.y+=d;
        else modelRotation.z+=d;
    }
}
VKAPI_ATTR VkBool32 VKAPI_CALL HelloVulkanApplication::debugCallback(
    VkDebugUtilsMessageSeverityFlagBitsEXT,VkDebugUtilsMessageTypeFlagsEXT,
    const VkDebugUtilsMessengerCallbackDataEXT* d,void*){
    std::cerr<<"validation layer: "<<d->pMessage<<"\n"; return VK_FALSE;
}

void HelloVulkanApplication::initWindow(){
    glfwInit(); glfwWindowHint(GLFW_CLIENT_API,GLFW_NO_API); glfwWindowHint(GLFW_RESIZABLE,GLFW_TRUE);
    window=glfwCreateWindow(WIDTH,HEIGHT,"Soft Rasterizer",nullptr,nullptr);
    glfwSetWindowUserPointer(window,this);
    glfwSetFramebufferSizeCallback(window,framebufferResizeCallback);
    glfwSetKeyCallback(window,keyCallback);
    glfwSetCursorPosCallback(window,cursorPosCallback);
    glfwSetMouseButtonCallback(window,mouseButtonCallback);
    glfwSetScrollCallback(window,scrollCallback);
}
void HelloVulkanApplication::processInput(){ if(keys[GLFW_KEY_ESCAPE]) glfwSetWindowShouldClose(window,true); }
void HelloVulkanApplication::mainLoop(){ while(!glfwWindowShouldClose(window)){glfwPollEvents();processInput();drawFrame();} vkDeviceWaitIdle(device); }

// ---- HZB ----
float HelloVulkanApplication::getHzbDepth(int level,int x,int y){
    if(level>=(int)hzb.size()) return 1.f;
    int w=hzbDims[level].first, h=hzbDims[level].second;
    x=std::clamp(x,0,w-1); y=std::clamp(y,0,h-1);
    return hzb[level][y*w+x];
}
void HelloVulkanApplication::buildHZB(){
    int w=WIDTH, h=HEIGHT;
    if(hzb.empty()){hzb.push_back(zBuffer);hzbDims.push_back({w,h});}
    else{hzb[0]=zBuffer;hzbDims[0]={w,h};}
    int level=0;
    while(w>1||h>1){
        w=(w+1)/2; h=(h+1)/2; level++;
        if((int)hzb.size()<=level){hzb.push_back(std::vector<float>(w*h));hzbDims.push_back({w,h});}
        else{if(hzb[level].size()!=(size_t)(w*h))hzb[level].resize(w*h);hzbDims[level]={w,h};}
        for(int y=0;y<h;y++) for(int x=0;x<w;x++){
            float d=std::max({getHzbDepth(level-1,x*2,y*2),getHzbDepth(level-1,x*2+1,y*2),
                              getHzbDepth(level-1,x*2,y*2+1),getHzbDepth(level-1,x*2+1,y*2+1)});
            hzb[level][y*w+x]=d;
        }
    }
}
bool HelloVulkanApplication::queryHZB(int minX,int maxX,int minY,int maxY,float minZ){
    if(hzb.empty()) return false;
    int w=maxX-minX, h=maxY-minY; if(w<=0||h<=0) return true;
    int maxDim=std::max(w,h), level=0;
    while((maxDim>>level)>2&&level<(int)hzb.size()-1) level++;
    for(int y=minY>>level;y<=(maxY>>level);y++)
        for(int x=minX>>level;x<=(maxX>>level);x++)
            if(minZ<=getHzbDepth(level,x,y)) return false;
    return true;
}

// ---- Rasterizer helpers ----
Vec3 HelloVulkanApplication::barycentric(Vec3 p,Vec3 a,Vec3 b,Vec3 c){
    Vec3 v0=b-a,v1=c-a,v2=p-a;
    float d00=v0.dot(v0),d01=v0.dot(v1),d11=v1.dot(v1),d20=v2.dot(v0),d21=v2.dot(v1);
    float denom=d00*d11-d01*d01;
    if(std::abs(denom)<1e-5f) return {-1,-1,-1};
    float v=(d11*d20-d01*d21)/denom, w=(d00*d21-d01*d20)/denom;
    return {1.f-v-w,v,w};
}
double HelloVulkanApplication::get_ms(std::chrono::high_resolution_clock::time_point s,
                                       std::chrono::high_resolution_clock::time_point e){
    return std::chrono::duration<double,std::milli>(e-s).count();
}

uint64_t HelloVulkanApplication::scanlineRasterizeTri(Vec3 v0,Vec3 v1,Vec3 v2,uint8_t r,uint8_t g,uint8_t b){
    if(v0.y>v1.y)std::swap(v0,v1); if(v0.y>v2.y)std::swap(v0,v2); if(v1.y>v2.y)std::swap(v1,v2);
    int yStart=std::max(0,(int)std::ceil(v0.y-0.5f));
    int yEnd  =std::min((int)HEIGHT,(int)std::ceil(v2.y-0.5f));
    if(yStart>=yEnd) return 0;
    float longDy=v2.y-v0.y; if(longDy==0.f) return 0;
    uint64_t frags=0;
    for(int y=yStart;y<yEnd;y++){
        float pY=(float)y+0.5f, tL=(pY-v0.y)/longDy;
        float xL=v0.x+(v2.x-v0.x)*tL, zL=v0.z+(v2.z-v0.z)*tL;
        float xS,zS;
        if(pY<v1.y){float dy=v1.y-v0.y; if(dy==0)continue; float t=(pY-v0.y)/dy; xS=v0.x+(v1.x-v0.x)*t; zS=v0.z+(v1.z-v0.z)*t;}
        else        {float dy=v2.y-v1.y; if(dy==0)continue; float t=(pY-v1.y)/dy; xS=v1.x+(v2.x-v1.x)*t; zS=v1.z+(v2.z-v1.z)*t;}
        if(xL>xS){std::swap(xL,xS);std::swap(zL,zS);}
        int ixS=std::max(0,(int)std::ceil(xL-0.5f)), ixE=std::min((int)WIDTH,(int)std::ceil(xS-0.5f));
        float span=xS-xL; if(span<=0) continue;
        float dz=(zS-zL)/span, cz=zL+(ixS+0.5f-xL)*dz;
        frags+=(uint64_t)(ixE-ixS);
        for(int x=ixS;x<ixE;x++){
            int idx=y*WIDTH+x;
            if(cz<=zBuffer[idx]+1e-4f){zBuffer[idx]=cz; int p=idx*4; pixels[p]=r;pixels[p+1]=g;pixels[p+2]=b;pixels[p+3]=255;}
            cz+=dz;
        }
    }
    return frags;
}

// ---- softRasterize (full implementation) ----
void HelloVulkanApplication::softRasterize(){
    g_num_fragments=0;
    double t_clear=0,t_vertex=0,t_cull=0,t_raster=0,t_prepass=0,t_hzb=0;
    auto t_start=std::chrono::high_resolution_clock::now();

    float curT=(float)glfwGetTime(), dt=curT-lastFrameTime; lastFrameTime=curT;
    if(benchmarkMode&&benchmarkFrameCount>10) frameTimeHistory.push_back(dt*1000.0);

    int numInstances=1; Mat4 matGroupRot=Mat4::identity();
    if(benchmarkScenario==1) numInstances=20;
    else if(benchmarkScenario==2||benchmarkScenario==3) numInstances=5;

    if(benchmarkMode){
        float t=(float)benchmarkFrameCount/TOTAL_TEST_FRAMES;
        modelRotation={180.f,t*720.f,0.f};
        if(benchmarkScenario==0) cameraZ=3.5f+1.5f*std::sin(t*6.28f);
        else if(benchmarkScenario==1) cameraZ=12.f+4.f*std::sin(t*3.14f);
        else if(benchmarkScenario==2){cameraZ=12.f;cameraY=8.f;cameraX=8.f*std::cos(t);modelRotation={180.f,0.f,0.f};}
        else if(benchmarkScenario==3){cameraZ=12.f+3.f*std::cos(t);cameraY=0.f;cameraX=0.f;modelRotation={180.f,0.f,0.f};matGroupRot=Mat4::rotateY(t*1.5f);}
        if(cameraZ<0.5f)cameraZ=0.5f;
    } else {
        if(autoRotate){float spd=50.f*dt;if(currentAxis==0)modelRotation.x+=spd;else if(currentAxis==1)modelRotation.y+=spd;else modelRotation.z+=spd;}
        float ms=15.f*dt; if(keys[GLFW_KEY_LEFT_SHIFT])ms*=4.f;
        if(keys[GLFW_KEY_W])cameraZ-=ms; if(keys[GLFW_KEY_S])cameraZ+=ms;
        if(keys[GLFW_KEY_A])cameraX-=ms; if(keys[GLFW_KEY_D])cameraX+=ms;
        if(keys[GLFW_KEY_E])cameraY+=ms; if(keys[GLFW_KEY_Q])cameraY-=ms;
    }

    auto t0=std::chrono::high_resolution_clock::now();
    std::fill(zBuffer.begin(),zBuffer.end(),1.f);
    std::fill(pixels.begin(),pixels.end(),(unsigned char)30);
    for(size_t i=3;i<pixels.size();i+=4) pixels[i]=255;
    t_clear+=get_ms(t0,std::chrono::high_resolution_clock::now());

    float rz=modelRotation.z*3.14f/180.f, ry=modelRotation.y*3.14f/180.f, rx=modelRotation.x*3.14f/180.f;
    Mat4 matRot=Mat4::rotateZ(rz)*Mat4::rotateY(ry)*Mat4::rotateX(rx);
    Mat4 matScale=Mat4::scale(g_mesh.normalizeScale);
    Mat4 matOffset=Mat4::translate(g_mesh.centerOffset.x,g_mesh.centerOffset.y,g_mesh.centerOffset.z);
    Mat4 view=Mat4::translate(-cameraX,-cameraY,-cameraZ);
    Mat4 proj=Mat4::perspective(45.f*3.14159f/180.f,(float)WIDTH/HEIGHT,0.1f,100.f);
    if(cachedProjectedVerts.size()!=g_mesh.vertices.size()) cachedProjectedVerts.resize(g_mesh.vertices.size());

    auto checkFrustum=[&](const Mat4& mvp,const Mesh::BVHNode& node,float& mnX,float& mxX,float& mnY,float& mxY,float& mnZ)->bool{
        Vec3 corners[8]={{node.minB.x,node.minB.y,node.minB.z},{node.maxB.x,node.minB.y,node.minB.z},
                         {node.minB.x,node.maxB.y,node.minB.z},{node.maxB.x,node.maxB.y,node.minB.z},
                         {node.minB.x,node.minB.y,node.maxB.z},{node.maxB.x,node.minB.y,node.maxB.z},
                         {node.minB.x,node.maxB.y,node.maxB.z},{node.maxB.x,node.maxB.y,node.maxB.z}};
        mnX=mnY=mnZ=1e9f; mxX=mxY=-1e9f; bool any=false;
        for(int k=0;k<8;k++){
            float x=corners[k].x,y=corners[k].y,z=corners[k].z;
            float w=mvp.m[3][0]*x+mvp.m[3][1]*y+mvp.m[3][2]*z+mvp.m[3][3];
            if(w>0.01f){any=true; float iw=1.f/w;
                float px=(mvp.m[0][0]*x+mvp.m[0][1]*y+mvp.m[0][2]*z+mvp.m[0][3])*iw;
                float py=(mvp.m[1][0]*x+mvp.m[1][1]*y+mvp.m[1][2]*z+mvp.m[1][3])*iw;
                float pz=(mvp.m[2][0]*x+mvp.m[2][1]*y+mvp.m[2][2]*z+mvp.m[2][3])*iw;
                float sx=(px+1.f)*.5f*WIDTH, sy=(1.f-py)*.5f*HEIGHT;
                if(sx<mnX)mnX=sx; if(sx>mxX)mxX=sx; if(sy<mnY)mnY=sy; if(sy>mxY)mxY=sy; if(pz<mnZ)mnZ=pz;
            }
        }
        if(!any) return true;
        if(mnX>WIDTH||mxX<0||mnY>HEIGHT||mxY<0||mnZ>1.f) return false;
        return true;
    };
    auto getNodeDepth=[&](const Mat4& mvp,int nid)->float{
        if(nid==-1) return 999999.f;
        const auto& n=g_mesh.bvhNodes[nid];
        Vec3 c={(n.minB.x+n.maxB.x)*.5f,(n.minB.y+n.maxB.y)*.5f,(n.minB.z+n.maxB.z)*.5f};
        return mvp.transformPoint(c).z;
    };
    auto rasterizeDepth=[&](const Vec3& p0,const Vec3& p1,const Vec3& p2){
        if(p0.z<0||p0.z>1||p1.z<0||p1.z>1||p2.z<0||p2.z>1) return;
        Vec3 v0={(p0.x+1.f)*.5f*WIDTH,(1.f-p0.y)*.5f*HEIGHT,p0.z};
        Vec3 v1={(p1.x+1.f)*.5f*WIDTH,(1.f-p1.y)*.5f*HEIGHT,p1.z};
        Vec3 v2={(p2.x+1.f)*.5f*WIDTH,(1.f-p2.y)*.5f*HEIGHT,p2.z};
        if(v0.y>v1.y)std::swap(v0,v1);if(v0.y>v2.y)std::swap(v0,v2);if(v1.y>v2.y)std::swap(v1,v2);
        int yS=std::max(0,(int)std::ceil(v0.y-0.5f)),yE=std::min((int)HEIGHT,(int)std::ceil(v2.y-0.5f));
        if(yS>=yE) return; float ldy=v2.y-v0.y; if(ldy==0.f) return;
        for(int y=yS;y<yE;y++){
            float pY=y+0.5f,tL=(pY-v0.y)/ldy;
            float xL=v0.x+(v2.x-v0.x)*tL,zL=v0.z+(v2.z-v0.z)*tL,xS2,zS2;
            if(pY<v1.y){float d=v1.y-v0.y;if(d==0)continue;float t=(pY-v0.y)/d;xS2=v0.x+(v1.x-v0.x)*t;zS2=v0.z+(v1.z-v0.z)*t;}
            else        {float d=v2.y-v1.y;if(d==0)continue;float t=(pY-v1.y)/d;xS2=v1.x+(v2.x-v1.x)*t;zS2=v1.z+(v2.z-v1.z)*t;}
            if(xL>xS2){std::swap(xL,xS2);std::swap(zL,zS2);}
            int ixS=std::max(0,(int)std::ceil(xL-0.5f)),ixE=std::min((int)WIDTH,(int)std::ceil(xS2-0.5f));
            float dz=(xS2-xL==0)?0:(zS2-zL)/(xS2-xL),cz=zL+(ixS+0.5f-xL)*dz;
            for(int x=ixS;x<ixE;x++){int i2=y*WIDTH+x;if(cz<zBuffer[i2])zBuffer[i2]=cz;cz+=dz;}
        }
    };

    for(int inst=0;inst<numInstances;inst++){
        Mat4 mInst=Mat4::identity();
        if(benchmarkScenario==1){float ix=(inst%5-2.f)*3.f,iz=(inst/5-1.5f)*3.f;mInst=Mat4::translate(ix,0,iz);}
        else if(benchmarkScenario==2) mInst=Mat4::translate((inst-2.f)*3.5f,0,0);
        else if(benchmarkScenario==3) mInst=Mat4::translate(0,0,(inst-2.f)*3.5f);
        Mat4 mvp=proj*view*matGroupRot*mInst*matRot*matScale*matOffset;

        auto tv0=std::chrono::high_resolution_clock::now();
        #pragma omp parallel for
        for(int i=0;i<(int)g_mesh.vertices.size();i++) cachedProjectedVerts[i]=mvp.transformPoint(g_mesh.vertices[i]);
        t_vertex+=get_ms(tv0,std::chrono::high_resolution_clock::now());

        if(g_renderMode==1||g_renderMode==2){
            auto tr0=std::chrono::high_resolution_clock::now();
            for(int i=0;i<(int)g_mesh.faces.size();i+=3){
                Vec3 p0=cachedProjectedVerts[g_mesh.faces[i]],p1=cachedProjectedVerts[g_mesh.faces[i+1]],p2=cachedProjectedVerts[g_mesh.faces[i+2]];
                if(p0.z<0||p0.z>1||p1.z<0||p1.z>1||p2.z<0||p2.z>1) continue;
                Vec3 rn=(g_mesh.vertices[g_mesh.faces[i+1]]-g_mesh.vertices[g_mesh.faces[i]]).cross(g_mesh.vertices[g_mesh.faces[i+2]]-g_mesh.vertices[g_mesh.faces[i]]).normalize();
                Vec3 lit=simple_shading(matRot.transformPoint(rn));
                uint8_t r=(uint8_t)std::clamp(lit.x*255.f,0.f,255.f),g2=(uint8_t)std::clamp(lit.y*255.f,0.f,255.f),b=(uint8_t)std::clamp(lit.z*255.f,0.f,255.f);
                float sx0=(p0.x+1.f)*.5f*WIDTH,sy0=(1.f-p0.y)*.5f*HEIGHT;
                float sx1=(p1.x+1.f)*.5f*WIDTH,sy1=(1.f-p1.y)*.5f*HEIGHT;
                float sx2=(p2.x+1.f)*.5f*WIDTH,sy2=(1.f-p2.y)*.5f*HEIGHT;
                if(g_renderMode==2){
                    g_num_fragments+=scanlineRasterizeTri({sx0,sy0,p0.z},{sx1,sy1,p1.z},{sx2,sy2,p2.z},r,g2,b);
                } else {
                    int mnX=std::max(0,(int)std::min({sx0,sx1,sx2})),mxX=std::min((int)WIDTH-1,(int)std::max({sx0,sx1,sx2}));
                    int mnY=std::max(0,(int)std::min({sy0,sy1,sy2})),mxY=std::min((int)HEIGHT-1,(int)std::max({sy0,sy1,sy2}));
                    for(int y=mnY;y<=mxY;y++) for(int x=mnX;x<=mxX;x++){
                        Vec3 bc=barycentric({(float)x,(float)y,0},{sx0,sy0,0},{sx1,sy1,0},{sx2,sy2,0});
                        if(bc.x>=0&&bc.y>=0&&bc.z>=0){float z=bc.x*p0.z+bc.y*p1.z+bc.z*p2.z; int idx=y*WIDTH+x;
                            if(z<zBuffer[idx]){zBuffer[idx]=z;int p=idx*4;pixels[p]=r;pixels[p+1]=g2;pixels[p+2]=b;pixels[p+3]=255;}}
                    }
                }
            }
            t_raster+=get_ms(tr0,std::chrono::high_resolution_clock::now());
        } else if(g_renderMode==3||g_renderMode==4){
            // Depth pre-pass + HZB culled color pass
            auto tp0=std::chrono::high_resolution_clock::now();
            std::vector<int> preTasks,stack; stack.reserve(64);
            if(!g_mesh.bvhNodes.empty()) stack.push_back(0);
            while(!stack.empty()){
                int nid=stack.back();stack.pop_back();
                const auto& n=g_mesh.bvhNodes[nid];
                float mnX,mxX,mnY,mxY,mnZ;
                if(!checkFrustum(mvp,n,mnX,mxX,mnY,mxY,mnZ)) continue;
                if(n.isLeaf()) preTasks.push_back(nid);
                else{
                    float dL=getNodeDepth(mvp,n.left),dR=getNodeDepth(mvp,n.right);
                    if(dL>dR){if(n.left!=-1)stack.push_back(n.left);if(n.right!=-1)stack.push_back(n.right);}
                    else      {if(n.right!=-1)stack.push_back(n.right);if(n.left!=-1)stack.push_back(n.left);}
                }
            }
            for(int nid:preTasks){const auto& n=g_mesh.bvhNodes[nid];
                for(int i=0;i<n.triCount;i++){int f=( n.triStart+i)*3;
                    rasterizeDepth(cachedProjectedVerts[g_mesh.faces[f]],cachedProjectedVerts[g_mesh.faces[f+1]],cachedProjectedVerts[g_mesh.faces[f+2]]);}
            }
            t_prepass+=get_ms(tp0,std::chrono::high_resolution_clock::now());
            auto th0=std::chrono::high_resolution_clock::now(); buildHZB(); t_hzb+=get_ms(th0,std::chrono::high_resolution_clock::now());
            auto tr0=std::chrono::high_resolution_clock::now();
            std::vector<int> colorTasks; if(!g_mesh.bvhNodes.empty()) stack.push_back(0);
            while(!stack.empty()){
                int nid=stack.back();stack.pop_back(); const auto& n=g_mesh.bvhNodes[nid];
                float mnX,mxX,mnY,mxY,mnZ;
                if(!checkFrustum(mvp,n,mnX,mxX,mnY,mxY,mnZ)) continue;
                if(g_renderMode==4&&queryHZB((int)std::max(0.f,mnX),(int)std::min((float)WIDTH-1,mxX),(int)std::max(0.f,mnY),(int)std::min((float)HEIGHT-1,mxY),mnZ)) continue;
                if(n.isLeaf()) colorTasks.push_back(nid);
                else{float dL=getNodeDepth(mvp,n.left),dR=getNodeDepth(mvp,n.right);
                    if(dL>dR){if(n.left!=-1)stack.push_back(n.left);if(n.right!=-1)stack.push_back(n.right);}
                    else      {if(n.right!=-1)stack.push_back(n.right);if(n.left!=-1)stack.push_back(n.left);}}
            }
            for(int nid:colorTasks){const auto& n=g_mesh.bvhNodes[nid];
                for(int i=0;i<n.triCount;i++){
                    int f=(n.triStart+i)*3,i0=g_mesh.faces[f],i1=g_mesh.faces[f+1],i2=g_mesh.faces[f+2];
                    Vec3 p0=cachedProjectedVerts[i0],p1=cachedProjectedVerts[i1],p2=cachedProjectedVerts[i2];
                    if(p0.z<0||p0.z>1||p1.z<0||p1.z>1||p2.z<0||p2.z>1) continue;
                    float sx0=(p0.x+1.f)*.5f*WIDTH,sy0=(1.f-p0.y)*.5f*HEIGHT;
                    float sx1=(p1.x+1.f)*.5f*WIDTH,sy1=(1.f-p1.y)*.5f*HEIGHT;
                    float sx2=(p2.x+1.f)*.5f*WIDTH,sy2=(1.f-p2.y)*.5f*HEIGHT;
                    int mnX2=std::max(0,(int)std::min({sx0,sx1,sx2})),mxX2=std::min((int)WIDTH-1,(int)std::max({sx0,sx1,sx2}));
                    int mnY2=std::max(0,(int)std::min({sy0,sy1,sy2})),mxY2=std::min((int)HEIGHT-1,(int)std::max({sy0,sy1,sy2}));
                    float mnZ2=std::min({p0.z,p1.z,p2.z});
                    if(g_renderMode==4&&queryHZB(mnX2,mxX2,mnY2,mxY2,mnZ2)) continue;
                    Vec3 rn=(g_mesh.vertices[i1]-g_mesh.vertices[i0]).cross(g_mesh.vertices[i2]-g_mesh.vertices[i0]).normalize();
                    Vec3 lit=simple_shading(matRot.transformPoint(rn));
                    uint8_t r=(uint8_t)std::clamp(lit.x*255.f,0.f,255.f),g2=(uint8_t)std::clamp(lit.y*255.f,0.f,255.f),b=(uint8_t)std::clamp(lit.z*255.f,0.f,255.f);
                    g_num_fragments+=scanlineRasterizeTri({sx0,sy0,p0.z},{sx1,sy1,p1.z},{sx2,sy2,p2.z},r,g2,b);
                }
            }
            t_raster+=get_ms(tr0,std::chrono::high_resolution_clock::now());
        }
    }

    auto t_end=std::chrono::high_resolution_clock::now();
    double t_total=get_ms(t_start,t_end);
    static int pc=0; if(++pc>=60){pc=0;
        std::cout<<std::fixed<<std::setprecision(2)<<"[Mode "<<g_renderMode<<"] Clear:"<<t_clear<<"ms Vtx:"<<t_vertex<<"ms Pre:"<<t_prepass<<"ms HZB:"<<t_hzb<<"ms Cull:"<<t_cull<<"ms Raster:"<<t_raster<<"ms Total:"<<t_total<<"ms\n";}
    if(benchmarkMode){
        std::cout<<"BENCH_DATA,"<<benchmarkFrameCount<<","<<(dt*1000.f)<<","<<g_num_fragments.load()<<"\n";
        benchmarkFrameCount++;
        if(benchmarkFrameCount>=TOTAL_TEST_FRAMES){
            std::sort(frameTimeHistory.begin(),frameTimeHistory.end());
            double q1=frameTimeHistory.empty()?0.0:frameTimeHistory[frameTimeHistory.size()/4];
            std::cout<<"BENCHMARK_RESULT_Q1:"<<q1<<"\n";
            glfwSetWindowShouldClose(window,true);
        }
    }
    // FPS overlay
    float fps=dt>0?1.f/dt:0;int fi=(int)fps;
    static const uint8_t digs[10][15]={{1,1,1,1,0,1,1,0,1,1,0,1,1,1,1},{0,1,0,0,1,0,0,1,0,0,1,0,0,1,0},{1,1,1,0,0,1,1,1,1,1,0,0,1,1,1},{1,1,1,0,0,1,1,1,1,0,0,1,1,1,1},{1,0,1,1,0,1,1,1,1,0,0,1,0,0,1},{1,1,1,1,0,0,1,1,1,0,0,1,1,1,1},{1,1,1,1,0,0,1,1,1,1,0,1,1,1,1},{1,1,1,0,0,1,0,0,1,0,0,1,0,0,1},{1,1,1,1,0,1,1,1,1,1,0,1,1,1,1},{1,1,1,1,0,1,1,1,1,0,0,1,1,1,1}};
    auto dp=[&](int x,int y,uint8_t r,uint8_t g,uint8_t b){if(x<0||x>=(int)WIDTH||y<0||y>=(int)HEIGHT)return;int i=(y*WIDTH+x)*4;pixels[i]=r;pixels[i+1]=g;pixels[i+2]=b;pixels[i+3]=255;};
    auto dd=[&](int n,int sx,int sy){if(n<0||n>9)return;for(int y=0;y<5;y++)for(int x=0;x<3;x++)if(digs[n][y*3+x])for(int sy2=0;sy2<2;sy2++)for(int sx2=0;sx2<2;sx2++)dp(sx+x*2+sx2,sy+y*2+sy2,0,255,0);};
    int cx=10; if((fi/100)%10>0){dd((fi/100)%10,cx,10);cx+=16;} if((fi/100)%10>0||(fi/10)%10>0){dd((fi/10)%10,cx,10);cx+=16;} dd(fi%10,cx,10);
}

void HelloVulkanApplication::updateTexture(){softRasterize();void*d;vkMapMemory(device,stagingBufferMemory,0,WIDTH*HEIGHT*4,0,&d);memcpy(d,pixels.data(),pixels.size());vkUnmapMemory(device,stagingBufferMemory);}

// ---- Vulkan plumbing (boilerplate from original) ----
void HelloVulkanApplication::initVulkan(){
    createInstance();setupDebugMessenger();createSurface();pickPhysicalDevice();createLogicalDevice();
    createSwapChain();createImageViews();createRenderPass();createDescriptorSetLayout();createGraphicsPipeline();
    createCommandPool();
    // staging + texture
    {VkDeviceSize sz=WIDTH*HEIGHT*4;VkBufferCreateInfo bi{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};bi.size=sz;bi.usage=VK_BUFFER_USAGE_TRANSFER_SRC_BIT;vkCreateBuffer(device,&bi,nullptr,&stagingBuffer);VkMemoryRequirements mr;vkGetBufferMemoryRequirements(device,stagingBuffer,&mr);VkMemoryAllocateInfo ai{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};ai.allocationSize=mr.size;ai.memoryTypeIndex=findMemoryType(mr.memoryTypeBits,VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT|VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);vkAllocateMemory(device,&ai,nullptr,&stagingBufferMemory);vkBindBufferMemory(device,stagingBuffer,stagingBufferMemory,0);}
    {VkImageCreateInfo ii{VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO};ii.imageType=VK_IMAGE_TYPE_2D;ii.extent={WIDTH,HEIGHT,1};ii.mipLevels=1;ii.arrayLayers=1;ii.format=VK_FORMAT_R8G8B8A8_SRGB;ii.tiling=VK_IMAGE_TILING_OPTIMAL;ii.initialLayout=VK_IMAGE_LAYOUT_UNDEFINED;ii.usage=VK_IMAGE_USAGE_TRANSFER_DST_BIT|VK_IMAGE_USAGE_SAMPLED_BIT;ii.samples=VK_SAMPLE_COUNT_1_BIT;vkCreateImage(device,&ii,nullptr,&textureImage);VkMemoryRequirements mr;vkGetImageMemoryRequirements(device,textureImage,&mr);VkMemoryAllocateInfo ai{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};ai.allocationSize=mr.size;ai.memoryTypeIndex=findMemoryType(mr.memoryTypeBits,VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);vkAllocateMemory(device,&ai,nullptr,&textureImageMemory);vkBindImageMemory(device,textureImage,textureImageMemory,0);}
    {VkImageViewCreateInfo vi{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};vi.image=textureImage;vi.viewType=VK_IMAGE_VIEW_TYPE_2D;vi.format=VK_FORMAT_R8G8B8A8_SRGB;vi.subresourceRange.aspectMask=VK_IMAGE_ASPECT_COLOR_BIT;vi.subresourceRange.levelCount=1;vi.subresourceRange.layerCount=1;vkCreateImageView(device,&vi,nullptr,&textureImageView);}
    {VkSamplerCreateInfo si{VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO};si.magFilter=VK_FILTER_NEAREST;si.minFilter=VK_FILTER_NEAREST;si.addressModeU=si.addressModeV=si.addressModeW=VK_SAMPLER_ADDRESS_MODE_REPEAT;si.anisotropyEnable=VK_FALSE;si.maxAnisotropy=1.f;si.borderColor=VK_BORDER_COLOR_INT_OPAQUE_BLACK;si.mipmapMode=VK_SAMPLER_MIPMAP_MODE_LINEAR;vkCreateSampler(device,&si,nullptr,&textureSampler);}
    createFramebuffers();createDescriptorPool();createDescriptorSets();createCommandBuffers();createSyncObjects();
}
void HelloVulkanApplication::createInstance(){
    VkApplicationInfo ai{VK_STRUCTURE_TYPE_APPLICATION_INFO};ai.pApplicationName="Soft Rasterizer";ai.apiVersion=VK_API_VERSION_1_0;
    VkInstanceCreateInfo ci{VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO};ci.pApplicationInfo=&ai;
    uint32_t gc=0;const char**ge=glfwGetRequiredInstanceExtensions(&gc);
    std::vector<const char*> exts(ge,ge+gc);if(enableValidationLayers)exts.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
    ci.enabledExtensionCount=(uint32_t)exts.size();ci.ppEnabledExtensionNames=exts.data();
    if(enableValidationLayers){ci.enabledLayerCount=(uint32_t)validationLayers.size();ci.ppEnabledLayerNames=validationLayers.data();}else ci.enabledLayerCount=0;
    if(vkCreateInstance(&ci,nullptr,&instance)!=VK_SUCCESS) throw std::runtime_error("failed to create instance");
}
void HelloVulkanApplication::setupDebugMessenger(){
    if(!enableValidationLayers) return;
    VkDebugUtilsMessengerCreateInfoEXT ci{VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT};
    ci.messageSeverity=VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT|VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
    ci.messageType=VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT|VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
    ci.pfnUserCallback=debugCallback;
    auto func=(PFN_vkCreateDebugUtilsMessengerEXT)vkGetInstanceProcAddr(instance,"vkCreateDebugUtilsMessengerEXT");
    if(func) func(instance,&ci,nullptr,&debugMessenger);
}
void HelloVulkanApplication::createSurface(){ if(glfwCreateWindowSurface(instance,window,nullptr,&surface)!=VK_SUCCESS) throw std::runtime_error("failed to create surface"); }
HelloVulkanApplication::QueueFamilyIndices HelloVulkanApplication::findQueueFamilies(VkPhysicalDevice dev){
    QueueFamilyIndices idx; uint32_t count=0; vkGetPhysicalDeviceQueueFamilyProperties(dev,&count,nullptr);
    std::vector<VkQueueFamilyProperties> fams(count); vkGetPhysicalDeviceQueueFamilyProperties(dev,&count,fams.data());
    for(uint32_t i=0;i<count;i++){if(fams[i].queueFlags&VK_QUEUE_GRAPHICS_BIT)idx.graphicsFamily=i;VkBool32 ps=false;vkGetPhysicalDeviceSurfaceSupportKHR(dev,i,surface,&ps);if(ps)idx.presentFamily=i;if(idx.isComplete())break;}
    return idx;
}
bool HelloVulkanApplication::isDeviceSuitable(VkPhysicalDevice dev){auto q=findQueueFamilies(dev);return q.isComplete()&&checkDeviceExtensionSupport(dev);}
bool HelloVulkanApplication::checkDeviceExtensionSupport(VkPhysicalDevice dev){
    uint32_t c=0;vkEnumerateDeviceExtensionProperties(dev,nullptr,&c,nullptr);std::vector<VkExtensionProperties>av(c);vkEnumerateDeviceExtensionProperties(dev,nullptr,&c,av.data());
    std::set<std::string>req(deviceExtensions.begin(),deviceExtensions.end());for(auto&e:av)req.erase(e.extensionName);return req.empty();
}
void HelloVulkanApplication::pickPhysicalDevice(){
    uint32_t c=0;vkEnumeratePhysicalDevices(instance,&c,nullptr);if(!c)throw std::runtime_error("no GPU");
    std::vector<VkPhysicalDevice>devs(c);vkEnumeratePhysicalDevices(instance,&c,devs.data());
    for(auto&d:devs){if(isDeviceSuitable(d)){physicalDevice=d;return;}}throw std::runtime_error("no suitable GPU");
}
void HelloVulkanApplication::createLogicalDevice(){
    auto idx=findQueueFamilies(physicalDevice);
    std::set<uint32_t>uq={idx.graphicsFamily.value(),idx.presentFamily.value()};
    std::vector<VkDeviceQueueCreateInfo>qcis;float p=1.f;
    for(auto f:uq){VkDeviceQueueCreateInfo qi{VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO};qi.queueFamilyIndex=f;qi.queueCount=1;qi.pQueuePriorities=&p;qcis.push_back(qi);}
    VkPhysicalDeviceFeatures df{};
    VkDeviceCreateInfo ci{VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO};ci.queueCreateInfoCount=(uint32_t)qcis.size();ci.pQueueCreateInfos=qcis.data();ci.pEnabledFeatures=&df;ci.enabledExtensionCount=(uint32_t)deviceExtensions.size();ci.ppEnabledExtensionNames=deviceExtensions.data();
    if(enableValidationLayers){ci.enabledLayerCount=(uint32_t)validationLayers.size();ci.ppEnabledLayerNames=validationLayers.data();}
    if(vkCreateDevice(physicalDevice,&ci,nullptr,&device)!=VK_SUCCESS)throw std::runtime_error("failed to create device");
    vkGetDeviceQueue(device,idx.graphicsFamily.value(),0,&graphicsQueue);vkGetDeviceQueue(device,idx.presentFamily.value(),0,&presentQueue);
}
HelloVulkanApplication::SwapChainSupportDetails HelloVulkanApplication::querySwapChainSupport(VkPhysicalDevice dev){
    SwapChainSupportDetails d;vkGetPhysicalDeviceSurfaceCapabilitiesKHR(dev,surface,&d.capabilities);
    uint32_t fc=0;vkGetPhysicalDeviceSurfaceFormatsKHR(dev,surface,&fc,nullptr);d.formats.resize(fc);vkGetPhysicalDeviceSurfaceFormatsKHR(dev,surface,&fc,d.formats.data());
    uint32_t pc=0;vkGetPhysicalDeviceSurfacePresentModesKHR(dev,surface,&pc,nullptr);d.presentModes.resize(pc);vkGetPhysicalDeviceSurfacePresentModesKHR(dev,surface,&pc,d.presentModes.data());
    return d;
}
VkSurfaceFormatKHR HelloVulkanApplication::chooseSwapSurfaceFormat(const std::vector<VkSurfaceFormatKHR>&fmts){for(auto&f:fmts)if(f.format==VK_FORMAT_B8G8R8A8_SRGB&&f.colorSpace==VK_COLOR_SPACE_SRGB_NONLINEAR_KHR)return f;return fmts[0];}
VkPresentModeKHR HelloVulkanApplication::chooseSwapPresentMode(const std::vector<VkPresentModeKHR>&modes){for(auto m:modes)if(m==VK_PRESENT_MODE_MAILBOX_KHR)return m;return VK_PRESENT_MODE_FIFO_KHR;}
VkExtent2D HelloVulkanApplication::chooseSwapExtent(const VkSurfaceCapabilitiesKHR&cap){if(cap.currentExtent.width!=std::numeric_limits<uint32_t>::max())return cap.currentExtent;int w,h;glfwGetFramebufferSize(window,&w,&h);VkExtent2D e={(uint32_t)w,(uint32_t)h};e.width=std::clamp(e.width,cap.minImageExtent.width,cap.maxImageExtent.width);e.height=std::clamp(e.height,cap.minImageExtent.height,cap.maxImageExtent.height);return e;}
void HelloVulkanApplication::createSwapChain(){
    auto sc=querySwapChainSupport(physicalDevice);auto fmt=chooseSwapSurfaceFormat(sc.formats);auto pm=chooseSwapPresentMode(sc.presentModes);auto ext=chooseSwapExtent(sc.capabilities);
    uint32_t ic=sc.capabilities.minImageCount+1;if(sc.capabilities.maxImageCount>0&&ic>sc.capabilities.maxImageCount)ic=sc.capabilities.maxImageCount;
    VkSwapchainCreateInfoKHR ci{VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR};ci.surface=surface;ci.minImageCount=ic;ci.imageFormat=fmt.format;ci.imageColorSpace=fmt.colorSpace;ci.imageExtent=ext;ci.imageArrayLayers=1;ci.imageUsage=VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
    auto qi=findQueueFamilies(physicalDevice);uint32_t qis[]={qi.graphicsFamily.value(),qi.presentFamily.value()};
    if(qi.graphicsFamily!=qi.presentFamily){ci.imageSharingMode=VK_SHARING_MODE_CONCURRENT;ci.queueFamilyIndexCount=2;ci.pQueueFamilyIndices=qis;}else ci.imageSharingMode=VK_SHARING_MODE_EXCLUSIVE;
    ci.preTransform=sc.capabilities.currentTransform;ci.compositeAlpha=VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;ci.presentMode=pm;ci.clipped=VK_TRUE;
    if(vkCreateSwapchainKHR(device,&ci,nullptr,&swapChain)!=VK_SUCCESS)throw std::runtime_error("failed to create swap chain");
    vkGetSwapchainImagesKHR(device,swapChain,&ic,nullptr);swapChainImages.resize(ic);vkGetSwapchainImagesKHR(device,swapChain,&ic,swapChainImages.data());
    swapChainImageFormat=fmt.format;swapChainExtent=ext;
}
void HelloVulkanApplication::createImageViews(){
    swapChainImageViews.resize(swapChainImages.size());
    for(size_t i=0;i<swapChainImages.size();i++){VkImageViewCreateInfo ci{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};ci.image=swapChainImages[i];ci.viewType=VK_IMAGE_VIEW_TYPE_2D;ci.format=swapChainImageFormat;ci.subresourceRange.aspectMask=VK_IMAGE_ASPECT_COLOR_BIT;ci.subresourceRange.levelCount=1;ci.subresourceRange.layerCount=1;vkCreateImageView(device,&ci,nullptr,&swapChainImageViews[i]);}
}
void HelloVulkanApplication::createRenderPass(){
    VkAttachmentDescription ca{};ca.format=swapChainImageFormat;ca.samples=VK_SAMPLE_COUNT_1_BIT;ca.loadOp=VK_ATTACHMENT_LOAD_OP_CLEAR;ca.storeOp=VK_ATTACHMENT_STORE_OP_STORE;ca.finalLayout=VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
    VkAttachmentReference cr{};cr.attachment=0;cr.layout=VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    VkSubpassDescription sp{};sp.pipelineBindPoint=VK_PIPELINE_BIND_POINT_GRAPHICS;sp.colorAttachmentCount=1;sp.pColorAttachments=&cr;
    VkSubpassDependency dep{};dep.srcSubpass=VK_SUBPASS_EXTERNAL;dep.dstSubpass=0;dep.srcStageMask=VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;dep.dstStageMask=VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;dep.dstAccessMask=VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    VkRenderPassCreateInfo ri{VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO};ri.attachmentCount=1;ri.pAttachments=&ca;ri.subpassCount=1;ri.pSubpasses=&sp;ri.dependencyCount=1;ri.pDependencies=&dep;
    if(vkCreateRenderPass(device,&ri,nullptr,&renderPass)!=VK_SUCCESS)throw std::runtime_error("failed to create render pass");
}
void HelloVulkanApplication::createDescriptorSetLayout(){
    VkDescriptorSetLayoutBinding b{};b.binding=0;b.descriptorType=VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;b.descriptorCount=1;b.stageFlags=VK_SHADER_STAGE_FRAGMENT_BIT;
    VkDescriptorSetLayoutCreateInfo li{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};li.bindingCount=1;li.pBindings=&b;
    if(vkCreateDescriptorSetLayout(device,&li,nullptr,&descriptorSetLayout)!=VK_SUCCESS)throw std::runtime_error("failed to create descriptor set layout");
}
VkShaderModule HelloVulkanApplication::createShaderModule(const std::vector<char>&code){
    VkShaderModuleCreateInfo ci{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};ci.codeSize=code.size();ci.pCode=reinterpret_cast<const uint32_t*>(code.data());
    VkShaderModule sm;if(vkCreateShaderModule(device,&ci,nullptr,&sm)!=VK_SUCCESS)throw std::runtime_error("failed to create shader module");return sm;
}
void HelloVulkanApplication::createGraphicsPipeline(){
    std::string dir=getExecutableDir();
    auto vc=readFile(dir+"/vert.spv"),fc=readFile(dir+"/frag.spv");
    auto vm=createShaderModule(vc),fm=createShaderModule(fc);
    VkPipelineShaderStageCreateInfo ss[2]{};
    ss[0].sType=VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;ss[0].stage=VK_SHADER_STAGE_VERTEX_BIT;ss[0].module=vm;ss[0].pName="main";
    ss[1].sType=VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;ss[1].stage=VK_SHADER_STAGE_FRAGMENT_BIT;ss[1].module=fm;ss[1].pName="main";
    VkPipelineVertexInputStateCreateInfo vi{VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO};
    VkPipelineInputAssemblyStateCreateInfo ia{VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO};ia.topology=VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
    VkPipelineViewportStateCreateInfo vs{VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO};vs.viewportCount=1;vs.scissorCount=1;
    VkPipelineRasterizationStateCreateInfo rs{VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO};rs.polygonMode=VK_POLYGON_MODE_FILL;rs.cullMode=VK_CULL_MODE_NONE;rs.lineWidth=1.f;
    VkPipelineMultisampleStateCreateInfo ms{VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO};ms.rasterizationSamples=VK_SAMPLE_COUNT_1_BIT;
    VkPipelineColorBlendAttachmentState cba{};cba.colorWriteMask=VK_COLOR_COMPONENT_R_BIT|VK_COLOR_COMPONENT_G_BIT|VK_COLOR_COMPONENT_B_BIT|VK_COLOR_COMPONENT_A_BIT;
    VkPipelineColorBlendStateCreateInfo cb{VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO};cb.attachmentCount=1;cb.pAttachments=&cba;
    VkDynamicState ds[]={VK_DYNAMIC_STATE_VIEWPORT,VK_DYNAMIC_STATE_SCISSOR};VkPipelineDynamicStateCreateInfo dy{VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO};dy.dynamicStateCount=2;dy.pDynamicStates=ds;
    VkPipelineLayoutCreateInfo pl{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};pl.setLayoutCount=1;pl.pSetLayouts=&descriptorSetLayout;vkCreatePipelineLayout(device,&pl,nullptr,&pipelineLayout);
    VkGraphicsPipelineCreateInfo pi{VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO};pi.stageCount=2;pi.pStages=ss;pi.pVertexInputState=&vi;pi.pInputAssemblyState=&ia;pi.pViewportState=&vs;pi.pRasterizationState=&rs;pi.pMultisampleState=&ms;pi.pColorBlendState=&cb;pi.pDynamicState=&dy;pi.layout=pipelineLayout;pi.renderPass=renderPass;
    if(vkCreateGraphicsPipelines(device,VK_NULL_HANDLE,1,&pi,nullptr,&graphicsPipeline)!=VK_SUCCESS)throw std::runtime_error("failed to create graphics pipeline");
    vkDestroyShaderModule(device,fm,nullptr);vkDestroyShaderModule(device,vm,nullptr);
}
void HelloVulkanApplication::createFramebuffers(){
    swapChainFramebuffers.resize(swapChainImageViews.size());
    for(size_t i=0;i<swapChainImageViews.size();i++){VkImageView att[]={swapChainImageViews[i]};VkFramebufferCreateInfo fi{VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO};fi.renderPass=renderPass;fi.attachmentCount=1;fi.pAttachments=att;fi.width=swapChainExtent.width;fi.height=swapChainExtent.height;fi.layers=1;vkCreateFramebuffer(device,&fi,nullptr,&swapChainFramebuffers[i]);}
}
void HelloVulkanApplication::createCommandPool(){VkCommandPoolCreateInfo pi{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};pi.queueFamilyIndex=findQueueFamilies(physicalDevice).graphicsFamily.value();pi.flags=VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;vkCreateCommandPool(device,&pi,nullptr,&commandPool);}
uint32_t HelloVulkanApplication::findMemoryType(uint32_t filter,VkMemoryPropertyFlags props){VkPhysicalDeviceMemoryProperties mp;vkGetPhysicalDeviceMemoryProperties(physicalDevice,&mp);for(uint32_t i=0;i<mp.memoryTypeCount;i++)if((filter&(1<<i))&&(mp.memoryTypes[i].propertyFlags&props)==props)return i;throw std::runtime_error("no suitable memory");}
void HelloVulkanApplication::createDescriptorPool(){VkDescriptorPoolSize ps{};ps.type=VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;ps.descriptorCount=(uint32_t)MAX_FRAMES_IN_FLIGHT;VkDescriptorPoolCreateInfo pi{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};pi.poolSizeCount=1;pi.pPoolSizes=&ps;pi.maxSets=(uint32_t)MAX_FRAMES_IN_FLIGHT;vkCreateDescriptorPool(device,&pi,nullptr,&descriptorPool);}
void HelloVulkanApplication::createDescriptorSets(){
    std::vector<VkDescriptorSetLayout>layouts(MAX_FRAMES_IN_FLIGHT,descriptorSetLayout);VkDescriptorSetAllocateInfo ai{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};ai.descriptorPool=descriptorPool;ai.descriptorSetCount=(uint32_t)MAX_FRAMES_IN_FLIGHT;ai.pSetLayouts=layouts.data();descriptorSets.resize(MAX_FRAMES_IN_FLIGHT);vkAllocateDescriptorSets(device,&ai,descriptorSets.data());
    for(size_t i=0;i<MAX_FRAMES_IN_FLIGHT;i++){VkDescriptorImageInfo ii{};ii.imageLayout=VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;ii.imageView=textureImageView;ii.sampler=textureSampler;VkWriteDescriptorSet w{VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};w.dstSet=descriptorSets[i];w.dstBinding=0;w.descriptorType=VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;w.descriptorCount=1;w.pImageInfo=&ii;vkUpdateDescriptorSets(device,1,&w,0,nullptr);}
}
void HelloVulkanApplication::createCommandBuffers(){commandBuffers.resize(MAX_FRAMES_IN_FLIGHT);VkCommandBufferAllocateInfo ai{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};ai.commandPool=commandPool;ai.level=VK_COMMAND_BUFFER_LEVEL_PRIMARY;ai.commandBufferCount=(uint32_t)commandBuffers.size();vkAllocateCommandBuffers(device,&ai,commandBuffers.data());}
void HelloVulkanApplication::createSyncObjects(){
    imageAvailableSemaphores.resize(MAX_FRAMES_IN_FLIGHT);renderFinishedSemaphores.resize(MAX_FRAMES_IN_FLIGHT);inFlightFences.resize(MAX_FRAMES_IN_FLIGHT);
    VkSemaphoreCreateInfo si{VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO};VkFenceCreateInfo fi{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};fi.flags=VK_FENCE_CREATE_SIGNALED_BIT;
    for(size_t i=0;i<MAX_FRAMES_IN_FLIGHT;i++){vkCreateSemaphore(device,&si,nullptr,&imageAvailableSemaphores[i]);vkCreateSemaphore(device,&si,nullptr,&renderFinishedSemaphores[i]);vkCreateFence(device,&fi,nullptr,&inFlightFences[i]);}
}
void HelloVulkanApplication::drawFrame(){
    vkWaitForFences(device,1,&inFlightFences[currentFrame],VK_TRUE,UINT64_MAX);
    uint32_t img;VkResult r=vkAcquireNextImageKHR(device,swapChain,UINT64_MAX,imageAvailableSemaphores[currentFrame],VK_NULL_HANDLE,&img);
    if(r==VK_ERROR_OUT_OF_DATE_KHR){recreateSwapChain();return;}
    vkResetFences(device,1,&inFlightFences[currentFrame]);
    updateTexture();
    vkResetCommandBuffer(commandBuffers[currentFrame],0);
    VkCommandBuffer cb=commandBuffers[currentFrame];VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};vkBeginCommandBuffer(cb,&bi);
    VkImageMemoryBarrier b1{VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER};b1.oldLayout=VK_IMAGE_LAYOUT_UNDEFINED;b1.newLayout=VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;b1.image=textureImage;b1.subresourceRange={VK_IMAGE_ASPECT_COLOR_BIT,0,1,0,1};b1.dstAccessMask=VK_ACCESS_TRANSFER_WRITE_BIT;vkCmdPipelineBarrier(cb,VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,VK_PIPELINE_STAGE_TRANSFER_BIT,0,0,nullptr,0,nullptr,1,&b1);
    VkBufferImageCopy reg{};reg.imageSubresource={VK_IMAGE_ASPECT_COLOR_BIT,0,0,1};reg.imageExtent={WIDTH,HEIGHT,1};vkCmdCopyBufferToImage(cb,stagingBuffer,textureImage,VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,1,&reg);
    VkImageMemoryBarrier b2=b1;b2.oldLayout=VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;b2.newLayout=VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;b2.srcAccessMask=VK_ACCESS_TRANSFER_WRITE_BIT;b2.dstAccessMask=VK_ACCESS_SHADER_READ_BIT;vkCmdPipelineBarrier(cb,VK_PIPELINE_STAGE_TRANSFER_BIT,VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,0,0,nullptr,0,nullptr,1,&b2);
    VkRenderPassBeginInfo rpi{VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO};rpi.renderPass=renderPass;rpi.framebuffer=swapChainFramebuffers[img];rpi.renderArea.extent=swapChainExtent;VkClearValue cv={{{0,0,0,1}}};rpi.clearValueCount=1;rpi.pClearValues=&cv;
    vkCmdBeginRenderPass(cb,&rpi,VK_SUBPASS_CONTENTS_INLINE);vkCmdBindPipeline(cb,VK_PIPELINE_BIND_POINT_GRAPHICS,graphicsPipeline);
    VkViewport vp{};vp.width=(float)swapChainExtent.width;vp.height=(float)swapChainExtent.height;vp.maxDepth=1.f;vkCmdSetViewport(cb,0,1,&vp);VkRect2D sc{};sc.extent=swapChainExtent;vkCmdSetScissor(cb,0,1,&sc);
    vkCmdBindDescriptorSets(cb,VK_PIPELINE_BIND_POINT_GRAPHICS,pipelineLayout,0,1,&descriptorSets[currentFrame],0,nullptr);vkCmdDraw(cb,3,1,0,0);vkCmdEndRenderPass(cb);vkEndCommandBuffer(cb);
    VkSubmitInfo subi{VK_STRUCTURE_TYPE_SUBMIT_INFO};VkSemaphore ws[]={imageAvailableSemaphores[currentFrame]};VkPipelineStageFlags wst[]={VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT};subi.waitSemaphoreCount=1;subi.pWaitSemaphores=ws;subi.pWaitDstStageMask=wst;subi.commandBufferCount=1;subi.pCommandBuffers=&cb;VkSemaphore ss2[]={renderFinishedSemaphores[currentFrame]};subi.signalSemaphoreCount=1;subi.pSignalSemaphores=ss2;vkQueueSubmit(graphicsQueue,1,&subi,inFlightFences[currentFrame]);
    VkPresentInfoKHR pi{VK_STRUCTURE_TYPE_PRESENT_INFO_KHR};pi.waitSemaphoreCount=1;pi.pWaitSemaphores=ss2;VkSwapchainKHR scs[]={swapChain};pi.swapchainCount=1;pi.pSwapchains=scs;pi.pImageIndices=&img;r=vkQueuePresentKHR(presentQueue,&pi);
    if(r==VK_ERROR_OUT_OF_DATE_KHR||r==VK_SUBOPTIMAL_KHR||framebufferResized){framebufferResized=false;recreateSwapChain();}
    currentFrame=(currentFrame+1)%MAX_FRAMES_IN_FLIGHT;vkQueueWaitIdle(presentQueue);
}
void HelloVulkanApplication::cleanupSwapChain(){for(auto f:swapChainFramebuffers)vkDestroyFramebuffer(device,f,nullptr);for(auto v:swapChainImageViews)vkDestroyImageView(device,v,nullptr);vkDestroySwapchainKHR(device,swapChain,nullptr);}
void HelloVulkanApplication::recreateSwapChain(){int w=0,h=0;glfwGetFramebufferSize(window,&w,&h);while(!w||!h){glfwGetFramebufferSize(window,&w,&h);glfwWaitEvents();}vkDeviceWaitIdle(device);cleanupSwapChain();createSwapChain();createImageViews();createFramebuffers();}
void HelloVulkanApplication::cleanup(){
    cleanupSwapChain();vkDestroySampler(device,textureSampler,nullptr);vkDestroyImageView(device,textureImageView,nullptr);vkDestroyImage(device,textureImage,nullptr);vkFreeMemory(device,textureImageMemory,nullptr);vkDestroyBuffer(device,stagingBuffer,nullptr);vkFreeMemory(device,stagingBufferMemory,nullptr);vkDestroyDescriptorPool(device,descriptorPool,nullptr);vkDestroyDescriptorSetLayout(device,descriptorSetLayout,nullptr);vkDestroyPipeline(device,graphicsPipeline,nullptr);vkDestroyPipelineLayout(device,pipelineLayout,nullptr);vkDestroyRenderPass(device,renderPass,nullptr);
    for(size_t i=0;i<MAX_FRAMES_IN_FLIGHT;i++){vkDestroySemaphore(device,renderFinishedSemaphores[i],nullptr);vkDestroySemaphore(device,imageAvailableSemaphores[i],nullptr);vkDestroyFence(device,inFlightFences[i],nullptr);}
    vkDestroyCommandPool(device,commandPool,nullptr);vkDestroyDevice(device,nullptr);
    if(enableValidationLayers&&debugMessenger!=VK_NULL_HANDLE){auto f=(PFN_vkDestroyDebugUtilsMessengerEXT)vkGetInstanceProcAddr(instance,"vkDestroyDebugUtilsMessengerEXT");if(f)f(instance,debugMessenger,nullptr);}
    vkDestroySurfaceKHR(instance,surface,nullptr);vkDestroyInstance(instance,nullptr);glfwDestroyWindow(window);glfwTerminate();
}