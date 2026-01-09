#define NOMINMAX
#include <vulkan/vulkan.h>
#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

#include <chrono>
#include <iostream>
#include <stdexcept>
#include <vector>
#include <cstring>
#include <cstdlib>
#include <optional>
#include <set>
#include <limits>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <cmath>
#include <array>
#include <atomic> // [新增] 用于线程安全的片元计数
#include <string>
#include <algorithm>

#ifdef _WIN32
    #include <windows.h>
#elif defined(__linux__)
    #include <unistd.h>
    #include <limits.h>
#endif

// OpenMP
#ifdef _OPENMP
#include <omp.h>
#endif

// -----------------------------------------------------------
// 3D 数学库
// -----------------------------------------------------------
struct Vec3
{
    float x, y, z;
    Vec3 operator+(const Vec3 &v) const { return {x + v.x, y + v.y, z + v.z}; }
    Vec3 operator-(const Vec3 &v) const { return {x - v.x, y - v.y, z - v.z}; }
    Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
    float dot(const Vec3 &v) const { return x * v.x + y * v.y + z * v.z; }
    Vec3 cross(const Vec3 &v) const { return {y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x}; }
    Vec3 normalize() const
    {
        float len = std::sqrt(x * x + y * y + z * z);
        return (len > 0) ? (*this) * (1.0f / len) : Vec3{0, 0, 0};
    }
};

struct Mat4
{
    float m[4][4];
    static Mat4 identity()
    {
        Mat4 res = {0};
        for (int i = 0; i < 4; i++)
            res.m[i][i] = 1.0f;
        return res;
    }
    static Mat4 perspective(float fov, float aspect, float znear, float zfar)
    {
        Mat4 res = {0};
        float tanHalfFov = std::tan(fov / 2.0f);
        res.m[0][0] = 1.0f / (aspect * tanHalfFov);
        res.m[1][1] = -1.0f / tanHalfFov; 
        res.m[2][2] = -zfar / (zfar - znear);
        res.m[2][3] = -(zfar * znear) / (zfar - znear);
        res.m[3][2] = -1.0f;
        return res;
    }
    static Mat4 scale(float s)
    {
        Mat4 res = identity();
        res.m[0][0] = s; res.m[1][1] = s; res.m[2][2] = s;
        return res;
    }
    static Mat4 translate(float x, float y, float z)
    {
        Mat4 res = identity();
        res.m[0][3] = x; res.m[1][3] = y; res.m[2][3] = z;
        return res;
    }
    static Mat4 rotateX(float angle)
    {
        Mat4 res = identity();
        float c = std::cos(angle), s = std::sin(angle);
        res.m[1][1] = c; res.m[1][2] = -s;
        res.m[2][1] = s; res.m[2][2] = c;
        return res;
    }
    static Mat4 rotateY(float angle)
    {
        Mat4 res = identity();
        float c = std::cos(angle), s = std::sin(angle);
        res.m[0][0] = c; res.m[0][2] = s;
        res.m[2][0] = -s; res.m[2][2] = c;
        return res;
    }
    static Mat4 rotateZ(float angle)
    {
        Mat4 res = identity();
        float c = std::cos(angle), s = std::sin(angle);
        res.m[0][0] = c; res.m[0][1] = -s;
        res.m[1][0] = s; res.m[1][1] = c;
        return res;
    }

    Vec3 transformPoint(const Vec3 &v) const
    {
        float x = m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z + m[0][3];
        float y = m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z + m[1][3];
        float z = m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z + m[2][3];
        float w = m[3][0] * v.x + m[3][1] * v.y + m[3][2] * v.z + m[3][3];
        if (w != 0.0f) { x /= w; y /= w; z /= w; }
        return {x, y, z};
    }
    Mat4 operator*(const Mat4 &r) const
    {
        Mat4 res = {0};
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                for (int k = 0; k < 4; k++)
                    res.m[i][j] += m[i][k] * r.m[k][j];
        return res;
    }
};

Vec3 simple_shading(const Vec3& normal) {
    Vec3 ambient = {0.1f, 0.1f, 0.1f};
    Vec3 lightPos1 = {1.0f, 1.0f, 1.0f}; 
    Vec3 lightDir1 = lightPos1.normalize();
    float diff1 = std::max(normal.dot(lightDir1), 0.0f);
    Vec3 diffuse1 = {0.8f * diff1, 0.8f * diff1, 0.8f * diff1}; 
    Vec3 lightDir2 = Vec3{-1.0f, 0.5f, -1.0f}.normalize();
    float diff2 = std::max(normal.dot(lightDir2), 0.0f);
    Vec3 diffuse2 = {0.3f * diff2, 0.3f * diff2, 0.4f * diff2}; 
    return ambient + diffuse1 + diffuse2;
}

struct Mesh
{
    std::vector<Vec3> vertices;
    std::vector<int> faces;
    Vec3 centerOffset = {0, 0, 0};
    float normalizeScale = 1.0f;
    float minY = std::numeric_limits<float>::max();
    float maxY = std::numeric_limits<float>::lowest();

    struct BVHNode {
        Vec3 minB, maxB; 
        int left = -1;
        int right = -1;
        int triStart = -1;
        int triCount = 0;
        bool isLeaf() const { return left == -1 && right == -1; }
    };

    std::vector<BVHNode> bvhNodes;
    std::vector<int> sortedFaces;

    int buildBVHRecursive(std::vector<int>& triIndices, int depth) {
        BVHNode node;
        node.minB = {std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
        node.maxB = {std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest()};

        for (int ti : triIndices) {
            for (int k = 0; k < 3; k++) {
                Vec3 v = vertices[faces[ti * 3 + k]];
                if (v.x < node.minB.x) node.minB.x = v.x; if (v.x > node.maxB.x) node.maxB.x = v.x;
                if (v.y < node.minB.y) node.minB.y = v.y; if (v.y > node.maxB.y) node.maxB.y = v.y;
                if (v.z < node.minB.z) node.minB.z = v.z; if (v.z > node.maxB.z) node.maxB.z = v.z;
            }
        }

        if (triIndices.size() <= 16 || depth > 20) {
            node.triStart = (int)sortedFaces.size() / 3;
            node.triCount = (int)triIndices.size();
            for (int ti : triIndices) {
                sortedFaces.push_back(faces[ti * 3 + 0]);
                sortedFaces.push_back(faces[ti * 3 + 1]);
                sortedFaces.push_back(faces[ti * 3 + 2]);
            }
            bvhNodes.push_back(node);
            return (int)bvhNodes.size() - 1;
        }

        Vec3 size = node.maxB - node.minB;
        int axis = 0;
        if (size.y > size.x) axis = 1;
        if (size.z > size.y && size.z > size.x) axis = 2;
        float mid = (axis == 0 ? (node.minB.x + node.maxB.x) : (axis == 1 ? (node.minB.y + node.maxB.y) : (node.minB.z + node.maxB.z))) * 0.5f;

        std::vector<int> leftTris, rightTris;
        for (int ti : triIndices) {
            float center = 0;
            for(int k=0; k<3; k++) {
                Vec3 v = vertices[faces[ti * 3 + k]];
                center += (axis == 0 ? v.x : (axis == 1 ? v.y : v.z));
            }
            center /= 3.0f;
            if (center < mid) leftTris.push_back(ti);
            else rightTris.push_back(ti);
        }

        if (leftTris.empty() || rightTris.empty()) {
            node.triStart = (int)sortedFaces.size() / 3;
            node.triCount = (int)triIndices.size();
            for (int ti : triIndices) {
                sortedFaces.push_back(faces[ti * 3 + 0]);
                sortedFaces.push_back(faces[ti * 3 + 1]);
                sortedFaces.push_back(faces[ti * 3 + 2]);
            }
            bvhNodes.push_back(node);
            return (int)bvhNodes.size() - 1;
        }

        int currIndex = (int)bvhNodes.size();
        bvhNodes.push_back(node);
        int lIdx = buildBVHRecursive(leftTris, depth + 1);
        int rIdx = buildBVHRecursive(rightTris, depth + 1);
        bvhNodes[currIndex].left = lIdx;
        bvhNodes[currIndex].right = rIdx;
        return currIndex;
    }

    void buildBVH() {
        if (vertices.empty()) return;
        std::cout << "Building BVH..." << std::endl;
        bvhNodes.clear(); bvhNodes.reserve(faces.size() / 2);
        sortedFaces.clear(); sortedFaces.reserve(faces.size());
        std::vector<int> allTriIndices(faces.size() / 3);
        for(int i=0; i<allTriIndices.size(); i++) allTriIndices[i] = i;
        buildBVHRecursive(allTriIndices, 0);
        faces = sortedFaces; 
        std::cout << "BVH Built. Nodes: " << bvhNodes.size() << std::endl;
    }

    static Mesh loadObj(const std::string &filename)
    {
        Mesh mesh;
        std::ifstream file(filename);
        Vec3 minB = {std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
        Vec3 maxB = {std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest()};

        if (!file.is_open()) {
            throw std::runtime_error("FATAL: Failed to open OBJ file: " + filename);
        }

        std::string line;
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string prefix; ss >> prefix;
            if (prefix == "v") {
                Vec3 v; ss >> v.x >> v.y >> v.z;
                mesh.vertices.push_back(v);
                if (v.x < minB.x) minB.x = v.x; if (v.x > maxB.x) maxB.x = v.x;
                if (v.y < minB.y) minB.y = v.y; if (v.y > maxB.y) maxB.y = v.y;
                if (v.z < minB.z) minB.z = v.z; if (v.z > maxB.z) maxB.z = v.z;
            } else if (prefix == "f") {
                std::string segment;
                for (int i = 0; i < 3; i++) {
                    ss >> segment;
                    size_t slashPos = segment.find('/');
                    int idx = std::stoi(segment.substr(0, slashPos));
                    mesh.faces.push_back(idx - 1);
                }
            }
        }
        mesh.minY = minB.y; mesh.maxY = maxB.y;
        if (!mesh.vertices.empty()) {
            Vec3 center = {(minB.x + maxB.x) / 2.0f, (minB.y + maxB.y) / 2.0f, (minB.z + maxB.z) / 2.0f};
            mesh.centerOffset = {-center.x, -center.y, -center.z};
            float sizeX = maxB.x - minB.x; float sizeY = maxB.y - minB.y; float sizeZ = maxB.z - minB.z;
            float maxDim = std::max({sizeX, sizeY, sizeZ});
            if (maxDim > 0) mesh.normalizeScale = 1.74f / maxDim;
            std::cout << "Model Bounds: " << minB.y << " to " << maxB.y << std::endl;
        }
        mesh.buildBVH();
        return mesh;
    }
};

const uint32_t WIDTH = 1920;
const uint32_t HEIGHT = 1080;
const int MAX_FRAMES_IN_FLIGHT = 2;

bool keys[1024] = {false};
float cameraX = 0.0f, cameraY = 0.0f, cameraZ = 3.5f;
float lastX = WIDTH / 2.0f, lastY = HEIGHT / 2.0f;
bool firstMouse = true;
bool leftMouseDown = false;
bool rightMouseDown = false;

int currentAxis = 1;                     
Vec3 modelRotation = {180.0f, 0.0f, 0.0f};
bool autoRotate = true;                  
int g_renderMode = 1; 

// [新增] 线程安全的片元计数器
std::atomic<uint64_t> g_num_fragments{0};

std::vector<unsigned char> pixels(WIDTH * HEIGHT * 4);
std::vector<float> zBuffer(WIDTH *HEIGHT);
Mesh g_mesh;

const std::vector<const char *> validationLayers = {"VK_LAYER_KHRONOS_validation"};
const std::vector<const char *> deviceExtensions = {VK_KHR_SWAPCHAIN_EXTENSION_NAME};
#ifdef NDEBUG
const bool enableValidationLayers = false;
#else
const bool enableValidationLayers = true;
#endif

static std::string getExecutableDir() {
#ifdef _WIN32
    char buffer[MAX_PATH];
    GetModuleFileNameA(NULL, buffer, MAX_PATH);
    std::string fullPath(buffer);
#elif defined(__linux__)
    char buffer[PATH_MAX];
    ssize_t len = readlink("/proc/self/exe", buffer, sizeof(buffer) - 1);
    if (len != -1) {
        buffer[len] = '\0';
        std::string fullPath(buffer);
    } else {
        return ".";
    }
#else
    return ".";
#endif

    // 统一处理：去掉文件名，只留目录
    std::string path(buffer);
    size_t pos = path.find_last_of("/\\");
    return (std::string::npos == pos) ? "" : path.substr(0, pos);
}

static std::vector<char> readFile(const std::string &filename) {
    std::ifstream file(filename, std::ios::ate | std::ios::binary);
    if (!file.is_open()) throw std::runtime_error("failed to open file: " + filename);
    size_t fileSize = (size_t)file.tellg();
    std::vector<char> buffer(fileSize);
    file.seekg(0); file.read(buffer.data(), fileSize); file.close();
    return buffer;
}

class HelloVulkanApplication
{
public:
    void run(const char *objFilename, int mode, int scenario, bool isBenchmark) {
        g_renderMode = mode;
        benchmarkScenario = scenario;
        benchmarkMode = isBenchmark; 

        std::cout << "--------------------------------------------------" << std::endl;
        std::cout << "Config: Mode=" << g_renderMode 
                  << " | Scenario=" << benchmarkScenario 
                  << " | Benchmark=" << (benchmarkMode ? "ON" : "OFF") << std::endl;
        std::cout << "--------------------------------------------------" << std::endl;

        if (objFilename) g_mesh = Mesh::loadObj(objFilename);
        else g_mesh = Mesh::loadObj("dummy.obj");

        initWindow();
        initVulkan();

        frameTimeHistory.reserve(TOTAL_TEST_FRAMES);
        startTime = std::chrono::high_resolution_clock::now();
        mainLoop();
        cleanup();
    }

private:
    GLFWwindow *window;
    VkInstance instance;
    VkDebugUtilsMessengerEXT debugMessenger = VK_NULL_HANDLE;
    VkSurfaceKHR surface;
    VkPhysicalDevice physicalDevice = VK_NULL_HANDLE;
    VkDevice device;
    VkQueue graphicsQueue;
    VkQueue presentQueue;
    VkSwapchainKHR swapChain;
    std::vector<VkImage> swapChainImages;
    VkFormat swapChainImageFormat;
    VkExtent2D swapChainExtent;
    std::vector<VkImageView> swapChainImageViews;
    std::vector<VkFramebuffer> swapChainFramebuffers;
    VkRenderPass renderPass;
    VkPipelineLayout pipelineLayout;
    VkPipeline graphicsPipeline;
    VkDescriptorPool descriptorPool;
    VkDescriptorSetLayout descriptorSetLayout;
    std::vector<VkDescriptorSet> descriptorSets;
    VkCommandPool commandPool;
    std::vector<VkCommandBuffer> commandBuffers;
    VkBuffer stagingBuffer;
    VkDeviceMemory stagingBufferMemory;
    VkImage textureImage;
    VkDeviceMemory textureImageMemory;
    VkImageView textureImageView;
    VkSampler textureSampler;
    std::vector<VkSemaphore> imageAvailableSemaphores;
    std::vector<VkSemaphore> renderFinishedSemaphores;
    std::vector<VkFence> inFlightFences;
    uint32_t currentFrame = 0;
    bool framebufferResized = false;
    float lastFrameTime = 0.0f;

    bool benchmarkMode = false;         
    int benchmarkScenario = 0;   
    int benchmarkFrameCount = 0;       
    const int TOTAL_TEST_FRAMES = 1200; 
    std::chrono::high_resolution_clock::time_point startTime;
    std::vector<double> frameTimeHistory; 

    std::vector<std::vector<float>> hzb;
    std::vector<std::pair<int, int>> hzbDims;
    std::vector<Vec3> cachedProjectedVerts;

    float getHzbDepth(int level, int x, int y) {
        if (level >= hzb.size()) return 1.0f;
        int w = hzbDims[level].first;
        int h = hzbDims[level].second;
        x = std::clamp(x, 0, w - 1);
        y = std::clamp(y, 0, h - 1);
        return hzb[level][y * w + x];
    }

    static void framebufferResizeCallback(GLFWwindow *window, int width, int height) {
        auto app = reinterpret_cast<HelloVulkanApplication *>(glfwGetWindowUserPointer(window));
        app->framebufferResized = true;
    }

    static void keyCallback(GLFWwindow *window, int key, int scancode, int action, int mods) {
        if (key >= 0 && key < 1024) keys[key] = (action != GLFW_RELEASE);
        if (key == GLFW_KEY_LEFT_CONTROL && action == GLFW_PRESS) currentAxis = (currentAxis + 1) % 3;
        if (key == GLFW_KEY_SPACE && action == GLFW_PRESS) autoRotate = !autoRotate;
        if (key == GLFW_KEY_1 && action == GLFW_PRESS) g_renderMode = 1;
        if (key == GLFW_KEY_2 && action == GLFW_PRESS) g_renderMode = 2;
        if (key == GLFW_KEY_3 && action == GLFW_PRESS) g_renderMode = 3;
    }

    static void scrollCallback(GLFWwindow *window, double xoffset, double yoffset) {
        float zoomSpeed = 0.2f;
        cameraZ -= (float)yoffset * zoomSpeed;
        if (cameraZ < 0.1f) cameraZ = 0.1f;
        if (cameraZ > 20.0f) cameraZ = 20.0f;
    }

    static void mouseButtonCallback(GLFWwindow *window, int button, int action, int mods) {
        if (button == GLFW_MOUSE_BUTTON_LEFT) {
            leftMouseDown = (action == GLFW_PRESS);
            if (leftMouseDown) autoRotate = false;
        }
        if (button == GLFW_MOUSE_BUTTON_RIGHT) rightMouseDown = (action == GLFW_PRESS);
    }

    static void cursorPosCallback(GLFWwindow *window, double xposIn, double yposIn) {
        float xpos = static_cast<float>(xposIn);
        float ypos = static_cast<float>(yposIn);
        if (firstMouse) { lastX = xpos; lastY = ypos; firstMouse = false; }
        float xoffset = xpos - lastX;
        float yoffset = lastY - ypos;
        lastX = xpos; lastY = ypos;
        if (leftMouseDown) {
            float sensitivity = 0.5f;
            float delta = xoffset * sensitivity;
            if (currentAxis == 0) modelRotation.x += delta;
            else if (currentAxis == 1) modelRotation.y += delta;
            else if (currentAxis == 2) modelRotation.z += delta;
        }
    }

    static VKAPI_ATTR VkBool32 VKAPI_CALL debugCallback(VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity, VkDebugUtilsMessageTypeFlagsEXT messageType, const VkDebugUtilsMessengerCallbackDataEXT *pCallbackData, void *pUserData) {
        std::cerr << "validation layer: " << pCallbackData->pMessage << std::endl;
        return VK_FALSE;
    }

    void initWindow() {
        glfwInit();
        glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
        glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);
        window = glfwCreateWindow(WIDTH, HEIGHT, "Soft Rasterizer", nullptr, nullptr);
        glfwSetWindowUserPointer(window, this);
        glfwSetFramebufferSizeCallback(window, framebufferResizeCallback);
        glfwSetKeyCallback(window, keyCallback);
        glfwSetCursorPosCallback(window, cursorPosCallback);
        glfwSetMouseButtonCallback(window, mouseButtonCallback);
        glfwSetScrollCallback(window, scrollCallback); 
    }

    void processInput() { if (keys[GLFW_KEY_ESCAPE]) glfwSetWindowShouldClose(window, true); }

    Vec3 barycentric(Vec3 p, Vec3 a, Vec3 b, Vec3 c) {
        Vec3 v0 = b - a, v1 = c - a, v2 = p - a;
        float d00 = v0.dot(v0); float d01 = v0.dot(v1); float d11 = v1.dot(v1);
        float d20 = v2.dot(v0); float d21 = v2.dot(v1);
        float denom = d00 * d11 - d01 * d01;
        if (std::abs(denom) < 1e-5) return {-1, -1, -1};
        float v = (d11 * d20 - d01 * d21) / denom;
        float w = (d00 * d21 - d01 * d20) / denom;
        float u = 1.0f - v - w;
        return {u, v, w};
    }

    void buildHZB() {
        int w = WIDTH; int h = HEIGHT;
        if (hzb.empty()) { hzb.push_back(zBuffer); hzbDims.push_back({w, h}); } 
        else { hzb[0] = zBuffer; hzbDims[0] = {w, h}; }
        int level = 0;
        while (w > 1 || h > 1) {
            w = (w + 1) / 2; h = (h + 1) / 2;
            level++;
            if (hzb.size() <= level) { hzb.push_back(std::vector<float>(w * h)); hzbDims.push_back({w, h}); } 
            else { if (hzb[level].size() != w * h) hzb[level].resize(w * h); hzbDims[level] = {w, h}; }
            for (int y = 0; y < h; y++) {
                for (int x = 0; x < w; x++) {
                    float d00 = getHzbDepth(level - 1, x * 2, y * 2);
                    float d01 = getHzbDepth(level - 1, x * 2 + 1, y * 2);
                    float d10 = getHzbDepth(level - 1, x * 2, y * 2 + 1);
                    float d11 = getHzbDepth(level - 1, x * 2 + 1, y * 2 + 1);
                    hzb[level][y * w + x] = std::max({d00, d01, d10, d11});
                }
            }
        }
    }

    bool queryHZB(int minX, int maxX, int minY, int maxY, float minZ) {
        if (hzb.empty()) return false;
        int w = maxX - minX; int h = maxY - minY;
        if (w <= 0 || h <= 0) return true;
        int maxDim = std::max(w, h);
        int level = 0;
        while ((maxDim >> level) > 2 && level < hzb.size() - 1) level++;
        int lMinX = minX >> level; int lMinY = minY >> level;
        int lMaxX = maxX >> level; int lMaxY = maxY >> level;
        for (int y = lMinY; y <= lMaxY; y++) {
            for (int x = lMinX; x <= lMaxX; x++) {
                if (minZ <= getHzbDepth(level, x, y)) return false;
            }
        }
        return true;
    }

    // [修复版] 更加稳定的扫描线光栅化
    uint64_t scanlineRasterizeTri(Vec3 v0, Vec3 v1, Vec3 v2, uint8_t r, uint8_t g, uint8_t b) {
        if (v0.y > v1.y) std::swap(v0, v1);
        if (v0.y > v2.y) std::swap(v0, v2);
        if (v1.y > v2.y) std::swap(v1, v2);

        int yStart = (int)std::ceil(v0.y - 0.5f);
        int yEnd   = (int)std::ceil(v2.y - 0.5f);
        yStart = std::max(0, yStart);
        yEnd   = std::min((int)HEIGHT, yEnd);
        if (yStart >= yEnd) return 0;

        float longDy = v2.y - v0.y;
        if (longDy == 0.0f) return 0;

        // [统计] 累加片元数 (线程安全)
        // 粗略估计：计算三角形面积覆盖的像素数，或者在循环中精确累加
        // 为了精确起见，我们在循环中累加扫描线宽度
        uint64_t triFragments = 0;

        for (int y = yStart; y < yEnd; y++) {
            float pixelY = (float)y + 0.5f;
            float tLong = (pixelY - v0.y) / longDy;
            float xLong = v0.x + (v2.x - v0.x) * tLong;
            float zLong = v0.z + (v2.z - v0.z) * tLong;

            float xShort, zShort;
            if (pixelY < v1.y) {
                float dy1 = v1.y - v0.y;
                if (dy1 == 0.0f) continue;
                float t1 = (pixelY - v0.y) / dy1;
                xShort = v0.x + (v1.x - v0.x) * t1;
                zShort = v0.z + (v1.z - v0.z) * t1;
            } else {
                float dy2 = v2.y - v1.y;
                if (dy2 == 0.0f) continue;
                float t2 = (pixelY - v1.y) / dy2;
                xShort = v1.x + (v2.x - v1.x) * t2;
                zShort = v1.z + (v2.z - v1.z) * t2;
            }

            float xStart = xLong; float xEnd = xShort;
            float zStart = zLong; float zEnd = zShort;
            if (xStart > xEnd) { std::swap(xStart, xEnd); std::swap(zStart, zEnd); }

            int ixStart = (int)std::ceil(xStart - 0.5f);
            int ixEnd   = (int)std::ceil(xEnd - 0.5f);
            ixStart = std::max(0, ixStart);
            ixEnd   = std::min((int)WIDTH, ixEnd);

            float spanWidth = xEnd - xStart;
            if (spanWidth <= 0.0f) continue;

            float dzPerPixel = (zEnd - zStart) / spanWidth;
            float xPreStep = (float)ixStart + 0.5f - xStart;
            float currentZ = zStart + xPreStep * dzPerPixel;

            // [统计] 累加本行片元
            triFragments += (uint64_t)(ixEnd - ixStart);

            for (int x = ixStart; x < ixEnd; x++) {
                int idx = y * WIDTH + x;
                if (currentZ < zBuffer[idx]) {
                    zBuffer[idx] = currentZ;
                    int pIdx = idx * 4;
                    pixels[pIdx + 0] = r; pixels[pIdx + 1] = g; pixels[pIdx + 2] = b; pixels[pIdx + 3] = 255;
                }
                currentZ += dzPerPixel;
            }
        }
        return triFragments;
    }

    std::vector<uint8_t> vertexProcessed; 

    void softRasterize()
    {
        // [新增] 重置片元计数器
        g_num_fragments = 0;

        float currentFrameTime = (float)glfwGetTime();
        float deltaTime = currentFrameTime - lastFrameTime;
        lastFrameTime = currentFrameTime;

        if (benchmarkMode && benchmarkFrameCount > 10) { 
            frameTimeHistory.push_back(deltaTime * 1000.0);
        }
        int numInstances = 1;
        Mat4 matGroupRot = Mat4::identity();

        if (benchmarkScenario == 1) numInstances = 20; 
        else if (benchmarkScenario == 2 || benchmarkScenario == 3) numInstances = 5; 
        else numInstances = 1; 

        if (benchmarkMode) {
            float t = (float)benchmarkFrameCount / TOTAL_TEST_FRAMES;    
            modelRotation = {180.0f, t * 360.0f * 2.0f, 0.0f}; 
            if (benchmarkScenario == 0) {
                cameraZ = 3.5f + 1.5f * std::sin(t * 3.14159f * 2.0f);
            } 
            else if (benchmarkScenario == 1) {
                cameraZ = 12.0f + 4.0f * std::sin(t * 3.14159f); 
            }
            else if (benchmarkScenario == 2) {
                cameraZ = 12.0f; cameraY = 8.0f; cameraX = 8.0f * std::cos(t);
                modelRotation = {180.0f, 0.0f, 0.0f}; 
            }
            else if (benchmarkScenario == 3) {
                cameraZ = 12.0f + 3.0f * std::cos(t); cameraY = 0.0f; cameraX = 0.0f;
                modelRotation = {180.0f, 0.0f, 0.0f};
                matGroupRot = Mat4::rotateY(t * 1.5f); 
            }
            if (cameraZ < 0.5f) cameraZ = 0.5f;
        } else {
            if (autoRotate) {
                float speed = 50.0f * deltaTime;
                if (currentAxis == 0) modelRotation.x += speed;
                else if (currentAxis == 1) modelRotation.y += speed;
                else if (currentAxis == 2) modelRotation.z += speed;
            }
            float moveSpeed = 15.0f * deltaTime; 
            if (keys[GLFW_KEY_LEFT_SHIFT]) moveSpeed *= 4.0f;
            if (keys[GLFW_KEY_W]) cameraZ -= moveSpeed;
            if (keys[GLFW_KEY_S]) cameraZ += moveSpeed;
            if (keys[GLFW_KEY_A]) cameraX -= moveSpeed;
            if (keys[GLFW_KEY_D]) cameraX += moveSpeed;
            if (keys[GLFW_KEY_E]) cameraY += moveSpeed; 
            if (keys[GLFW_KEY_Q]) cameraY -= moveSpeed;
        }

        std::fill(pixels.begin(), pixels.end(), 30);
        std::fill(zBuffer.begin(), zBuffer.end(), 1.0f);

        Mat4 matRot = Mat4::rotateZ(modelRotation.z * 3.14f/180.0f) * Mat4::rotateY(modelRotation.y * 3.14f/180.0f) * Mat4::rotateX(modelRotation.x * 3.14f/180.0f);
        Mat4 matScale = Mat4::scale(g_mesh.normalizeScale);
        Mat4 matOffset = Mat4::translate(g_mesh.centerOffset.x, g_mesh.centerOffset.y, g_mesh.centerOffset.z);
        Mat4 view = Mat4::translate(-cameraX, -cameraY, -cameraZ);
        Mat4 proj = Mat4::perspective(45.0f * 3.14159f / 180.0f, (float)WIDTH / HEIGHT, 0.1f, 100.0f);

        if (cachedProjectedVerts.size() != g_mesh.vertices.size()) {
            cachedProjectedVerts.resize(g_mesh.vertices.size());
            vertexProcessed.resize(g_mesh.vertices.size());
        }

        auto transformVertex = [&](Mat4& currMvp, int vIdx) {
            if (!vertexProcessed[vIdx]) {
                cachedProjectedVerts[vIdx] = currMvp.transformPoint(g_mesh.vertices[vIdx]);
                vertexProcessed[vIdx] = 1;
            }
        };

        if (g_renderMode == 1 || g_renderMode == 2) 
        {
            for (int inst = 0; inst < numInstances; inst++) {
                Mat4 matWorldInst = Mat4::identity();
                if (benchmarkScenario == 1) { float x = (inst % 5 - 2.0f)*3.0f; float z = (inst / 5 - 1.5f)*3.0f; matWorldInst = Mat4::translate(x, 0, z); }
                else if (benchmarkScenario == 2) { matWorldInst = Mat4::translate((inst - 2.0f)*3.5f, 0, 0); }
                else if (benchmarkScenario == 3) { matWorldInst = Mat4::translate(0, 0, (inst - 2.0f)*3.5f); }

                Mat4 mvp = proj * view * matGroupRot * matWorldInst * matRot * matScale * matOffset;

                #pragma omp parallel for
                for (int i = 0; i < (int)g_mesh.vertices.size(); i++) {
                    cachedProjectedVerts[i] = mvp.transformPoint(g_mesh.vertices[i]);
                }

                if (g_renderMode == 1) { // Z-Buffer
                    uint64_t batchFragments = 0;
                    //增加 reduction(+:batchFragments)
                    #pragma omp parallel for reduction(+:batchFragments)
                    for (int i = 0; i < (int)g_mesh.faces.size(); i += 3) {
                        Vec3 p0 = cachedProjectedVerts[g_mesh.faces[i]];
                        Vec3 p1 = cachedProjectedVerts[g_mesh.faces[i+1]];
                        Vec3 p2 = cachedProjectedVerts[g_mesh.faces[i+2]];
                        if (p0.z < 0 || p0.z > 1 || p1.z < 0 || p1.z > 1 || p2.z < 0 || p2.z > 1) continue;
                        
                        float sx0 = (p0.x + 1.0f) * 0.5f * WIDTH;  float sy0 = (1.0f - p0.y) * 0.5f * HEIGHT;
                        float sx1 = (p1.x + 1.0f) * 0.5f * WIDTH;  float sy1 = (1.0f - p1.y) * 0.5f * HEIGHT;
                        float sx2 = (p2.x + 1.0f) * 0.5f * WIDTH;  float sy2 = (1.0f - p2.y) * 0.5f * HEIGHT;
                        int minX = std::max(0, (int)std::min({sx0, sx1, sx2})); int maxX = std::min((int)WIDTH - 1, (int)std::max({sx0, sx1, sx2}));
                        int minY = std::max(0, (int)std::min({sy0, sy1, sy2})); int maxY = std::min((int)HEIGHT - 1, (int)std::max({sy0, sy1, sy2}));
                        
                        Vec3 v0 = g_mesh.vertices[g_mesh.faces[i]]; Vec3 v1 = g_mesh.vertices[g_mesh.faces[i+1]]; Vec3 v2 = g_mesh.vertices[g_mesh.faces[i+2]];
                        Vec3 rawNormal = (v1 - v0).cross(v2 - v0).normalize();
                        Vec3 rotNormal = matRot.transformPoint(rawNormal); 
                        Vec3 litColor = simple_shading(rotNormal);
                        uint8_t r = (uint8_t)std::clamp(litColor.x * 255.0f, 0.0f, 255.0f);
                        uint8_t g = (uint8_t)std::clamp(litColor.y * 255.0f, 0.0f, 255.0f);
                        uint8_t b = (uint8_t)std::clamp(litColor.z * 255.0f, 0.0f, 255.0f);

                        // [统计]
                        uint64_t localFragCount = 0;

                        for (int y = minY; y <= maxY; y++) {
                            for (int x = minX; x <= maxX; x++) {
                                Vec3 P = {(float)x, (float)y, 0};
                                Vec3 bc = barycentric(P, {sx0, sy0, 0}, {sx1, sy1, 0}, {sx2, sy2, 0});
                                if (bc.x >= 0 && bc.y >= 0 && bc.z >= 0) {
                                    // [统计] 增加片元数
                                    localFragCount++;

                                    float z = bc.x * p0.z + bc.y * p1.z + bc.z * p2.z;
                                    int idx = y * WIDTH + x;
                                    if (z < zBuffer[idx]) {
                                        zBuffer[idx] = z;
                                        int pIdx = idx * 4;
                                        pixels[pIdx + 0] = r; pixels[pIdx + 1] = g; pixels[pIdx + 2] = b; pixels[pIdx + 3] = 255;
                                    }
                                }
                            }
                        }
                        // [新增] 累加到局部变量 (这是纯寄存器操作，极快)
                        batchFragments += localFragCount;
                    }
                    // 循环结束后，由主线程一次性更新全局变量
                    g_num_fragments += batchFragments;
                } 
                else if (g_renderMode == 2) { // Scanline
                    uint64_t batchFragments = 0;
                    #pragma omp parallel for reduction(+:batchFragments)
                    for (int i = 0; i < g_mesh.faces.size(); i += 3) {
                        Vec3 p0 = cachedProjectedVerts[g_mesh.faces[i]];
                        Vec3 p1 = cachedProjectedVerts[g_mesh.faces[i+1]];
                        Vec3 p2 = cachedProjectedVerts[g_mesh.faces[i+2]];
                        if (p0.z < 0 || p0.z > 1 || p1.z < 0 || p1.z > 1 || p2.z < 0 || p2.z > 1) continue;
                        Vec3 v0_scr, v1_scr, v2_scr;
                        v0_scr.x = (p0.x + 1.0f) * 0.5f * WIDTH;  v0_scr.y = (1.0f - p0.y) * 0.5f * HEIGHT; v0_scr.z = p0.z;
                        v1_scr.x = (p1.x + 1.0f) * 0.5f * WIDTH;  v1_scr.y = (1.0f - p1.y) * 0.5f * HEIGHT; v1_scr.z = p1.z;
                        v2_scr.x = (p2.x + 1.0f) * 0.5f * WIDTH;  v2_scr.y = (1.0f - p2.y) * 0.5f * HEIGHT; v2_scr.z = p2.z;
                        
                        Vec3 v0 = g_mesh.vertices[g_mesh.faces[i]]; Vec3 v1 = g_mesh.vertices[g_mesh.faces[i+1]]; Vec3 v2 = g_mesh.vertices[g_mesh.faces[i+2]];
                        Vec3 rawNormal = (v1 - v0).cross(v2 - v0).normalize(); Vec3 rotNormal = matRot.transformPoint(rawNormal); 

                        Vec3 litColor = simple_shading(rotNormal);
                        uint8_t r = (uint8_t)std::clamp(litColor.x * 255.0f, 0.0f, 255.0f);
                        uint8_t g = (uint8_t)std::clamp(litColor.y * 255.0f, 0.0f, 255.0f);
                        uint8_t b = (uint8_t)std::clamp(litColor.z * 255.0f, 0.0f, 255.0f);

                        batchFragments += scanlineRasterizeTri(v0_scr, v1_scr, v2_scr, r, g, b);
                    }
                    g_num_fragments += batchFragments;
                }
            }
        }
        else if (g_renderMode == 3 || g_renderMode == 4)
        {
            for (int inst = 0; inst < numInstances; inst++) {
                Mat4 matWorldInst = Mat4::identity();
                if (benchmarkScenario == 1) { float x = (inst % 5 - 2.0f)*3.0f; float z = (inst / 5 - 1.5f)*3.0f; matWorldInst = Mat4::translate(x, 0, z); }
                else if (benchmarkScenario == 2) { matWorldInst = Mat4::translate((inst - 2.0f)*3.5f, 0, 0); }
                else if (benchmarkScenario == 3) { matWorldInst = Mat4::translate(0, 0, (inst - 2.0f)*3.5f); }
                Mat4 currMvp = proj * view * matGroupRot * matWorldInst * matRot * matScale * matOffset;
                
                std::memset(vertexProcessed.data(), 0, vertexProcessed.size());

                auto getNodeDepth = [&](int nodeId) {
                    if (nodeId == -1) return 999999.0f;
                    const auto& n = g_mesh.bvhNodes[nodeId];
                    Vec3 center = (n.minB + n.maxB) * 0.5f;
                    Vec3 p = currMvp.transformPoint(center); return p.z;
                };

                auto checkFrustum = [&](const Mesh::BVHNode& node, float& minX, float& maxX, float& minY, float& maxY, float& minZ) -> bool {
                    Vec3 corners[8] = {
                        {node.minB.x, node.minB.y, node.minB.z}, {node.maxB.x, node.minB.y, node.minB.z},
                        {node.minB.x, node.maxB.y, node.minB.z}, {node.maxB.x, node.maxB.y, node.minB.z},
                        {node.minB.x, node.minB.y, node.maxB.z}, {node.maxB.x, node.minB.y, node.maxB.z},
                        {node.minB.x, node.maxB.y, node.maxB.z}, {node.maxB.x, node.maxB.y, node.maxB.z}
                    };
                    minX = std::numeric_limits<float>::max(); maxX = std::numeric_limits<float>::lowest();
                    minY = std::numeric_limits<float>::max(); maxY = std::numeric_limits<float>::lowest();
                    minZ = std::numeric_limits<float>::max();
                    bool visible = false;
                    for(int k=0; k<8; k++) {
                        float x = corners[k].x, y = corners[k].y, z = corners[k].z;
                        float w = currMvp.m[3][0]*x + currMvp.m[3][1]*y + currMvp.m[3][2]*z + currMvp.m[3][3];
                        if (w > 0.01f) { 
                            float invW = 1.0f / w;
                            float px = (currMvp.m[0][0]*x + currMvp.m[0][1]*y + currMvp.m[0][2]*z + currMvp.m[0][3]) * invW;
                            float py = (currMvp.m[1][0]*x + currMvp.m[1][1]*y + currMvp.m[1][2]*z + currMvp.m[1][3]) * invW;
                            float pz = (currMvp.m[2][0]*x + currMvp.m[2][1]*y + currMvp.m[2][2]*z + currMvp.m[2][3]) * invW;
                            float sx = (px + 1.0f) * 0.5f * WIDTH; float sy = (1.0f - py) * 0.5f * HEIGHT;
                            if(sx < minX) minX = sx; if(sx > maxX) maxX = sx;
                            if(sy < minY) minY = sy; if(sy > maxY) maxY = sy;
                            if(pz < minZ) minZ = pz;
                            visible = true;
                        }
                    }
                    if (!visible) return false;
                    if (minX > WIDTH || maxX < 0 || minY > HEIGHT || maxY < 0 || minZ > 1.0f) return false;
                    return true;
                };

                std::vector<int> prePassTasks; prePassTasks.reserve(2048);
                {
                    std::vector<int> nodeStack; nodeStack.reserve(64);
                    if (!g_mesh.bvhNodes.empty()) nodeStack.push_back(0);
                    while(!nodeStack.empty()) {
                        int nodeId = nodeStack.back(); nodeStack.pop_back();
                        const auto& node = g_mesh.bvhNodes[nodeId];
                        float minX, maxX, minY, maxY, minZ;
                        if (!checkFrustum(node, minX, maxX, minY, maxY, minZ)) continue;
                        
                        if (node.isLeaf()) { 
                            prePassTasks.push_back(nodeId); 
                        } else { 
                            float dLeft = getNodeDepth(node.left); float dRight = getNodeDepth(node.right);
                            if (dLeft > dRight) {
                                if(node.left != -1) nodeStack.push_back(node.left);
                                if(node.right != -1) nodeStack.push_back(node.right);
                            } else {
                                if(node.right != -1) nodeStack.push_back(node.right);
                                if(node.left != -1) nodeStack.push_back(node.left);
                            }
                        }
                    }
                }
                
                #pragma omp parallel for schedule(dynamic)
                for(int k=0; k < (int)prePassTasks.size(); ++k) {
                    const auto& node = g_mesh.bvhNodes[prePassTasks[k]];
                    for (int i = 0; i < node.triCount; i++) {
                        int faceIdx = (node.triStart + i) * 3;
                        int i0 = g_mesh.faces[faceIdx]; int i1 = g_mesh.faces[faceIdx+1]; int i2 = g_mesh.faces[faceIdx+2];
                        transformVertex(currMvp, i0); transformVertex(currMvp, i1); transformVertex(currMvp, i2);
                        Vec3 p0 = cachedProjectedVerts[i0]; Vec3 p1 = cachedProjectedVerts[i1]; Vec3 p2 = cachedProjectedVerts[i2];
                        if (p0.z < 0 || p0.z > 1 || p1.z < 0 || p1.z > 1 || p2.z < 0 || p2.z > 1) continue;
                        float sx0 = (p0.x + 1.0f) * 0.5f * WIDTH;  float sy0 = (1.0f - p0.y) * 0.5f * HEIGHT;
                        float sx1 = (p1.x + 1.0f) * 0.5f * WIDTH;  float sy1 = (1.0f - p1.y) * 0.5f * HEIGHT;
                        float sx2 = (p2.x + 1.0f) * 0.5f * WIDTH;  float sy2 = (1.0f - p2.y) * 0.5f * HEIGHT;
                        int minX_t = std::max(0, (int)std::min({sx0, sx1, sx2})); int maxX_t = std::min((int)WIDTH - 1, (int)std::max({sx0, sx1, sx2}));
                        int minY_t = std::max(0, (int)std::min({sy0, sy1, sy2})); int maxY_t = std::min((int)HEIGHT - 1, (int)std::max({sy0, sy1, sy2}));
                        for (int y = minY_t; y <= maxY_t; y++) {
                            for (int x = minX_t; x <= maxX_t; x++) {
                                Vec3 P = {(float)x, (float)y, 0}; Vec3 bc = barycentric(P, {sx0, sy0, 0}, {sx1, sy1, 0}, {sx2, sy2, 0});
                                if (bc.x >= 0 && bc.y >= 0 && bc.z >= 0) {
                                    float z = bc.x * p0.z + bc.y * p1.z + bc.z * p2.z; int idx = y * WIDTH + x;
                                    if (z < zBuffer[idx]) zBuffer[idx] = z;
                                }
                            }
                        }
                    }
                }
            } 

            if (g_renderMode == 4) {
                buildHZB();
            }

            for (int inst = 0; inst < numInstances; inst++) {
                uint64_t batchFragments = 0; // [新增]
                Mat4 matWorldInst = Mat4::identity();
                if (benchmarkScenario == 1) { float x = (inst % 5 - 2.0f)*3.0f; float z = (inst / 5 - 1.5f)*3.0f; matWorldInst = Mat4::translate(x, 0, z); }
                else if (benchmarkScenario == 2) { matWorldInst = Mat4::translate((inst - 2.0f)*3.5f, 0, 0); }
                else if (benchmarkScenario == 3) { matWorldInst = Mat4::translate(0, 0, (inst - 2.0f)*3.5f); }
                Mat4 currMvp = proj * view * matGroupRot * matWorldInst * matRot * matScale * matOffset;
                
                std::memset(vertexProcessed.data(), 0, vertexProcessed.size());

                auto getNodeDepth = [&](int nodeId) {
                    if (nodeId == -1) return 999999.0f;
                    const auto& n = g_mesh.bvhNodes[nodeId];
                    Vec3 center = (n.minB + n.maxB) * 0.5f;
                    Vec3 p = currMvp.transformPoint(center); return p.z;
                };

                auto checkFrustum = [&](const Mesh::BVHNode& node, float& minX, float& maxX, float& minY, float& maxY, float& minZ) -> bool {
                    Vec3 corners[8] = {
                        {node.minB.x, node.minB.y, node.minB.z}, {node.maxB.x, node.minB.y, node.minB.z},
                        {node.minB.x, node.maxB.y, node.minB.z}, {node.maxB.x, node.maxB.y, node.minB.z},
                        {node.minB.x, node.minB.y, node.maxB.z}, {node.maxB.x, node.minB.y, node.maxB.z},
                        {node.minB.x, node.maxB.y, node.maxB.z}, {node.maxB.x, node.maxB.y, node.maxB.z}
                    };
                    minX = std::numeric_limits<float>::max(); maxX = std::numeric_limits<float>::lowest();
                    minY = std::numeric_limits<float>::max(); maxY = std::numeric_limits<float>::lowest();
                    minZ = std::numeric_limits<float>::max();
                    bool visible = false;
                    for(int k=0; k<8; k++) {
                        float x = corners[k].x, y = corners[k].y, z = corners[k].z;
                        float w = currMvp.m[3][0]*x + currMvp.m[3][1]*y + currMvp.m[3][2]*z + currMvp.m[3][3];
                        if (w > 0.01f) { 
                            float invW = 1.0f / w;
                            float px = (currMvp.m[0][0]*x + currMvp.m[0][1]*y + currMvp.m[0][2]*z + currMvp.m[0][3]) * invW;
                            float py = (currMvp.m[1][0]*x + currMvp.m[1][1]*y + currMvp.m[1][2]*z + currMvp.m[1][3]) * invW;
                            float pz = (currMvp.m[2][0]*x + currMvp.m[2][1]*y + currMvp.m[2][2]*z + currMvp.m[2][3]) * invW;
                            float sx = (px + 1.0f) * 0.5f * WIDTH; float sy = (1.0f - py) * 0.5f * HEIGHT;
                            if(sx < minX) minX = sx; if(sx > maxX) maxX = sx;
                            if(sy < minY) minY = sy; if(sy > maxY) maxY = sy;
                            if(pz < minZ) minZ = pz;
                            visible = true;
                        }
                    }
                    if (!visible) return false;
                    if (minX > WIDTH || maxX < 0 || minY > HEIGHT || maxY < 0 || minZ > 1.0f) return false;
                    return true;
                };

                std::vector<int> colorPassTasks; colorPassTasks.reserve(2048); 
                {
                    std::vector<int> nodeStack; nodeStack.reserve(64);
                    if (!g_mesh.bvhNodes.empty()) nodeStack.push_back(0);
                    while(!nodeStack.empty()) {
                        int nodeId = nodeStack.back(); nodeStack.pop_back();
                        const auto& node = g_mesh.bvhNodes[nodeId];
                        float minX, maxX, minY, maxY, minZ;
                        if (!checkFrustum(node, minX, maxX, minY, maxY, minZ)) continue;
                        
                        if (g_renderMode == 4) {
                            if (queryHZB((int)std::max(0.0f, minX), (int)std::min((float)WIDTH-1, maxX), 
                                         (int)std::max(0.0f, minY), (int)std::min((float)HEIGHT-1, maxY), minZ)) {
                                continue; 
                            }
                        }

                        if (node.isLeaf()) { 
                            colorPassTasks.push_back(nodeId); 
                        }
                        else { 
                            float dLeft = getNodeDepth(node.left); float dRight = getNodeDepth(node.right);
                            if (dLeft > dRight) {
                                if(node.left != -1) nodeStack.push_back(node.left);
                                if(node.right != -1) nodeStack.push_back(node.right);
                            } else {
                                if(node.right != -1) nodeStack.push_back(node.right);
                                if(node.left != -1) nodeStack.push_back(node.left);
                            }
                        }
                    }
                }

                #pragma omp parallel for schedule(dynamic) reduction(+:batchFragments)
                for(int k=0; k < (int)colorPassTasks.size(); ++k) {
                    const auto& node = g_mesh.bvhNodes[colorPassTasks[k]];
                    for (int i = 0; i < node.triCount; i++) {
                        int faceIdx = (node.triStart + i) * 3;
                        int i0 = g_mesh.faces[faceIdx]; int i1 = g_mesh.faces[faceIdx+1]; int i2 = g_mesh.faces[faceIdx+2];
                        transformVertex(currMvp, i0); transformVertex(currMvp, i1); transformVertex(currMvp, i2);
                        Vec3 p0 = cachedProjectedVerts[i0]; Vec3 p1 = cachedProjectedVerts[i1]; Vec3 p2 = cachedProjectedVerts[i2];
                        if (p0.z < 0 || p0.z > 1 || p1.z < 0 || p1.z > 1 || p2.z < 0 || p2.z > 1) continue;
                        float sx0 = (p0.x + 1.0f) * 0.5f * WIDTH;  float sy0 = (1.0f - p0.y) * 0.5f * HEIGHT;
                        float sx1 = (p1.x + 1.0f) * 0.5f * WIDTH;  float sy1 = (1.0f - p1.y) * 0.5f * HEIGHT;
                        float sx2 = (p2.x + 1.0f) * 0.5f * WIDTH;  float sy2 = (1.0f - p2.y) * 0.5f * HEIGHT;
                        int minX_t = std::max(0, (int)std::min({sx0, sx1, sx2})); int maxX_t = std::min((int)WIDTH - 1, (int)std::max({sx0, sx1, sx2}));
                        int minY_t = std::max(0, (int)std::min({sy0, sy1, sy2})); int maxY_t = std::min((int)HEIGHT - 1, (int)std::max({sy0, sy1, sy2}));
                        
                        if (g_renderMode == 4) {
                            float triMinZ = std::min({p0.z, p1.z, p2.z});
                            if (minX_t <= maxX_t && minY_t <= maxY_t) {
                                if (queryHZB(minX_t, maxX_t, minY_t, maxY_t, triMinZ)) continue;
                            }
                        }

                        Vec3 v0 = g_mesh.vertices[i0]; Vec3 v1 = g_mesh.vertices[i1]; Vec3 v2 = g_mesh.vertices[i2];
                        Vec3 rawNormal = (v1 - v0).cross(v2 - v0).normalize(); Vec3 rotNormal = matRot.transformPoint(rawNormal); 
                        Vec3 litColor = simple_shading(rotNormal);
                        uint8_t r = (uint8_t)std::clamp(litColor.x * 255.0f, 0.0f, 255.0f);
                        uint8_t g = (uint8_t)std::clamp(litColor.y * 255.0f, 0.0f, 255.0f);
                        uint8_t b = (uint8_t)std::clamp(litColor.z * 255.0f, 0.0f, 255.0f);

                        // [统计]
                        uint64_t localFragCount = 0;

                        for (int y = minY_t; y <= maxY_t; y++) {
                            for (int x = minX_t; x <= maxX_t; x++) {
                                Vec3 P = {(float)x, (float)y, 0}; Vec3 bc = barycentric(P, {sx0, sy0, 0}, {sx1, sy1, 0}, {sx2, sy2, 0});
                                if (bc.x >= 0 && bc.y >= 0 && bc.z >= 0) {
                                    // [统计]
                                    localFragCount++;

                                    float z = bc.x * p0.z + bc.y * p1.z + bc.z * p2.z; int idx = y * WIDTH + x;
                                    if (z <= zBuffer[idx] + 0.00002f) { 
                                        int pIdx = idx * 4; pixels[pIdx + 0] = r; pixels[pIdx + 1] = g; pixels[pIdx + 2] = b; pixels[pIdx + 3] = 255; 
                                    }
                                }
                            }
                        }
                        batchFragments += localFragCount;
                    }
                }
                g_num_fragments += batchFragments;
            } 
        }

        if (benchmarkMode) {
            // [修改] 输出片元计数
            float frameMs = deltaTime * 1000.0f;
            std::cout << "BENCH_DATA," << benchmarkFrameCount << "," << frameMs << "," << g_num_fragments.load() << std::endl;
            
            benchmarkFrameCount++;
            if (benchmarkFrameCount >= TOTAL_TEST_FRAMES) {
                std::sort(frameTimeHistory.begin(), frameTimeHistory.end());
                double q1 = 0.0;
                if (!frameTimeHistory.empty()) {
                    size_t q1Index = frameTimeHistory.size() / 4;
                    q1 = frameTimeHistory[q1Index];
                }
                std::cout << "BENCHMARK_RESULT_Q1:" << q1 << std::endl;
                glfwSetWindowShouldClose(window, true);
            }
        }

        // --- UI ---
        float currentFPS = 0.0f;
        if (deltaTime > 0.0f) currentFPS = 1.0f / deltaTime;
        int fpsInt = (int)currentFPS;
        static const uint8_t digits[10][15] = {
            {1,1,1, 1,0,1, 1,0,1, 1,0,1, 1,1,1}, {0,1,0, 0,1,0, 0,1,0, 0,1,0, 0,1,0},
            {1,1,1, 0,0,1, 1,1,1, 1,0,0, 1,1,1}, {1,1,1, 0,0,1, 1,1,1, 0,0,1, 1,1,1},
            {1,0,1, 1,0,1, 1,1,1, 0,0,1, 0,0,1}, {1,1,1, 1,0,0, 1,1,1, 0,0,1, 1,1,1},
            {1,1,1, 1,0,0, 1,1,1, 1,0,1, 1,1,1}, {1,1,1, 0,0,1, 0,0,1, 0,0,1, 0,0,1},
            {1,1,1, 1,0,1, 1,1,1, 1,0,1, 1,1,1}, {1,1,1, 1,0,1, 1,1,1, 0,0,1, 1,1,1}
        };
        auto drawPixel = [&](int x, int y, uint8_t r, uint8_t g, uint8_t b) {
            if(x<0||x>=WIDTH||y<0||y>=HEIGHT) return;
            int idx = (y * WIDTH + x) * 4;
            pixels[idx+0]=r; pixels[idx+1]=g; pixels[idx+2]=b; pixels[idx+3]=255;
        };
        auto drawDigit = [&](int num, int startX, int startY) {
            if (num < 0 || num > 9) return;
            int scale = 2; 
            for (int y = 0; y < 5; y++) {
                for (int x = 0; x < 3; x++) {
                    if (digits[num][y * 3 + x] == 1) {
                        for (int sy = 0; sy < scale; sy++) {
                            for (int sx = 0; sx < scale; sx++) {
                                drawPixel(startX + x * scale + sx, startY + y * scale + sy, 0, 255, 0);
                            }
                        }
                    }
                }
            }
        };
        int d1 = (fpsInt / 100) % 10; int d2 = (fpsInt / 10) % 10; int d3 = fpsInt % 10;
        int cursorX = 10; int cursorY = 10; int spacing = 16; 
        if (d1 > 0) { drawDigit(d1, cursorX, cursorY); cursorX += spacing; }
        if (d1 > 0 || d2 > 0) { drawDigit(d2, cursorX, cursorY); cursorX += spacing; }
        drawDigit(d3, cursorX, cursorY);
        int modeY = cursorY + 15;
        for(int m=0; m < g_renderMode; m++) {
             for(int py=0; py<4; py++) for(int px=0; px<4; px++)
                drawPixel(10 + m*10 + px, modeY + py, 255, 50, 50);
        }
    }
    // ... (Vulkan plumbing code remains the same)
    void updateTexture() {
        softRasterize();
        void *data;
        vkMapMemory(device, stagingBufferMemory, 0, WIDTH * HEIGHT * 4, 0, &data);
        memcpy(data, pixels.data(), pixels.size());
        vkUnmapMemory(device, stagingBufferMemory);
    }
    void initVulkan() {
        createInstance(); setupDebugMessenger(); createSurface(); pickPhysicalDevice(); createLogicalDevice();
        createSwapChain(); createImageViews(); createRenderPass(); createDescriptorSetLayout(); createGraphicsPipeline();
        createCommandPool(); createStagingBuffer(); createTextureImage(); createTextureImageView(); createTextureSampler();
        createFramebuffers(); createDescriptorPool(); createDescriptorSets(); createCommandBuffers(); createSyncObjects();
    }
    // ... [Copy the rest of the vulkan setup functions from the original file] ...
    // Since the user provided the file content, I assume the rest is standard boilerplate. 
    // I will include the critical parts to make it compile.
    void createInstance() {
        VkApplicationInfo appInfo{VK_STRUCTURE_TYPE_APPLICATION_INFO};
        appInfo.pApplicationName = "Soft Rasterizer"; appInfo.apiVersion = VK_API_VERSION_1_0;
        VkInstanceCreateInfo createInfo{VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO}; createInfo.pApplicationInfo = &appInfo;
        uint32_t glfwExtensionCount = 0; const char **glfwExtensions = glfwGetRequiredInstanceExtensions(&glfwExtensionCount);
        std::vector<const char *> extensions(glfwExtensions, glfwExtensions + glfwExtensionCount);
        if (enableValidationLayers) extensions.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
        createInfo.enabledExtensionCount = static_cast<uint32_t>(extensions.size()); createInfo.ppEnabledExtensionNames = extensions.data();
        if (enableValidationLayers) { createInfo.enabledLayerCount = static_cast<uint32_t>(validationLayers.size()); createInfo.ppEnabledLayerNames = validationLayers.data(); } else createInfo.enabledLayerCount = 0;
        if (vkCreateInstance(&createInfo, nullptr, &instance) != VK_SUCCESS) throw std::runtime_error("failed to create instance!");
    }
    void setupDebugMessenger() {
        if (!enableValidationLayers) return;
        VkDebugUtilsMessengerCreateInfoEXT createInfo{VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT};
        createInfo.messageSeverity = VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
        createInfo.messageType = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
        createInfo.pfnUserCallback = debugCallback;
        auto func = (PFN_vkCreateDebugUtilsMessengerEXT)vkGetInstanceProcAddr(instance, "vkCreateDebugUtilsMessengerEXT");
        if (func != nullptr) func(instance, &createInfo, nullptr, &debugMessenger);
    }
    void createSurface() { if (glfwCreateWindowSurface(instance, window, nullptr, &surface) != VK_SUCCESS) throw std::runtime_error("failed to create window surface!"); }
    void pickPhysicalDevice() {
        uint32_t deviceCount = 0; vkEnumeratePhysicalDevices(instance, &deviceCount, nullptr);
        if (deviceCount == 0) throw std::runtime_error("failed to find GPUs with Vulkan support!");
        std::vector<VkPhysicalDevice> devices(deviceCount); vkEnumeratePhysicalDevices(instance, &deviceCount, devices.data());
        physicalDevice = devices[0];
    }
    void createLogicalDevice() {
        float queuePriority = 1.0f; VkDeviceQueueCreateInfo queueCreateInfo{VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO};
        queueCreateInfo.queueFamilyIndex = 0; queueCreateInfo.queueCount = 1; queueCreateInfo.pQueuePriorities = &queuePriority;
        VkDeviceCreateInfo createInfo{VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO}; createInfo.queueCreateInfoCount = 1; createInfo.pQueueCreateInfos = &queueCreateInfo;
        createInfo.enabledExtensionCount = static_cast<uint32_t>(deviceExtensions.size()); createInfo.ppEnabledExtensionNames = deviceExtensions.data();
        if (enableValidationLayers) { createInfo.enabledLayerCount = static_cast<uint32_t>(validationLayers.size()); createInfo.ppEnabledLayerNames = validationLayers.data(); }
        if (vkCreateDevice(physicalDevice, &createInfo, nullptr, &device) != VK_SUCCESS) throw std::runtime_error("failed to create logical device!");
        vkGetDeviceQueue(device, 0, 0, &graphicsQueue); vkGetDeviceQueue(device, 0, 0, &presentQueue);
    }
    void createSwapChain() {
        VkSurfaceCapabilitiesKHR capabilities; vkGetPhysicalDeviceSurfaceCapabilitiesKHR(physicalDevice, surface, &capabilities);
        VkSurfaceFormatKHR surfaceFormat = {VK_FORMAT_B8G8R8A8_SRGB, VK_COLOR_SPACE_SRGB_NONLINEAR_KHR};
        VkPresentModeKHR presentMode = VK_PRESENT_MODE_IMMEDIATE_KHR; VkExtent2D extent = {WIDTH, HEIGHT};
        uint32_t imageCount = capabilities.minImageCount + 1; if (capabilities.maxImageCount > 0 && imageCount > capabilities.maxImageCount) imageCount = capabilities.maxImageCount;
        VkSwapchainCreateInfoKHR createInfo{VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR};
        createInfo.surface = surface; createInfo.minImageCount = imageCount; createInfo.imageFormat = surfaceFormat.format;
        createInfo.imageColorSpace = surfaceFormat.colorSpace; createInfo.imageExtent = extent; createInfo.imageArrayLayers = 1;
        createInfo.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT; createInfo.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
        createInfo.preTransform = capabilities.currentTransform; createInfo.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
        createInfo.presentMode = presentMode; createInfo.clipped = VK_TRUE; createInfo.oldSwapchain = VK_NULL_HANDLE;
        if (vkCreateSwapchainKHR(device, &createInfo, nullptr, &swapChain) != VK_SUCCESS) throw std::runtime_error("failed to create swap chain!");
        vkGetSwapchainImagesKHR(device, swapChain, &imageCount, nullptr); swapChainImages.resize(imageCount);
        vkGetSwapchainImagesKHR(device, swapChain, &imageCount, swapChainImages.data());
        swapChainImageFormat = surfaceFormat.format; swapChainExtent = extent;
    }
    void createImageViews() {
        swapChainImageViews.resize(swapChainImages.size());
        for (size_t i = 0; i < swapChainImages.size(); i++) {
            VkImageViewCreateInfo createInfo{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
            createInfo.image = swapChainImages[i]; createInfo.viewType = VK_IMAGE_VIEW_TYPE_2D; createInfo.format = swapChainImageFormat;
            createInfo.components = {VK_COMPONENT_SWIZZLE_IDENTITY, VK_COMPONENT_SWIZZLE_IDENTITY, VK_COMPONENT_SWIZZLE_IDENTITY, VK_COMPONENT_SWIZZLE_IDENTITY};
            createInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT; createInfo.subresourceRange.baseMipLevel = 0; createInfo.subresourceRange.levelCount = 1;
            createInfo.subresourceRange.baseArrayLayer = 0; createInfo.subresourceRange.layerCount = 1;
            if (vkCreateImageView(device, &createInfo, nullptr, &swapChainImageViews[i]) != VK_SUCCESS) throw std::runtime_error("failed to create image views!");
        }
    }
    void createRenderPass() {
        VkAttachmentDescription colorAttachment{}; colorAttachment.format = swapChainImageFormat; colorAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
        colorAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR; colorAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        colorAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE; colorAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        colorAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED; colorAttachment.finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
        VkAttachmentReference colorAttachmentRef{}; colorAttachmentRef.attachment = 0; colorAttachmentRef.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        VkSubpassDescription subpass{}; subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS; subpass.colorAttachmentCount = 1; subpass.pColorAttachments = &colorAttachmentRef;
        VkRenderPassCreateInfo renderPassInfo{VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO}; renderPassInfo.attachmentCount = 1; renderPassInfo.pAttachments = &colorAttachment;
        renderPassInfo.subpassCount = 1; renderPassInfo.pSubpasses = &subpass;
        if (vkCreateRenderPass(device, &renderPassInfo, nullptr, &renderPass) != VK_SUCCESS) throw std::runtime_error("failed to create render pass!");
    }
    void createDescriptorSetLayout() {
        VkDescriptorSetLayoutBinding samplerLayoutBinding{}; samplerLayoutBinding.binding = 0; samplerLayoutBinding.descriptorCount = 1;
        samplerLayoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER; samplerLayoutBinding.pImmutableSamplers = nullptr; samplerLayoutBinding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
        VkDescriptorSetLayoutCreateInfo layoutInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO}; layoutInfo.bindingCount = 1; layoutInfo.pBindings = &samplerLayoutBinding;
        if (vkCreateDescriptorSetLayout(device, &layoutInfo, nullptr, &descriptorSetLayout) != VK_SUCCESS) throw std::runtime_error("failed to create descriptor set layout!");
    }
    void createGraphicsPipeline() {
        std::string exeDir = getExecutableDir();
        auto vertShaderCode = readFile(exeDir + "/vert.spv"); 
        auto fragShaderCode = readFile(exeDir + "/frag.spv");
        VkShaderModule vertShaderModule = createShaderModule(vertShaderCode); 
        VkShaderModule fragShaderModule = createShaderModule(fragShaderCode);
        VkPipelineShaderStageCreateInfo vertShaderStageInfo{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
        vertShaderStageInfo.stage = VK_SHADER_STAGE_VERTEX_BIT; vertShaderStageInfo.module = vertShaderModule; vertShaderStageInfo.pName = "main";
        VkPipelineShaderStageCreateInfo fragShaderStageInfo{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
        fragShaderStageInfo.stage = VK_SHADER_STAGE_FRAGMENT_BIT; fragShaderStageInfo.module = fragShaderModule; fragShaderStageInfo.pName = "main";
        VkPipelineShaderStageCreateInfo shaderStages[] = {vertShaderStageInfo, fragShaderStageInfo};
        VkPipelineVertexInputStateCreateInfo vertexInputInfo{VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO};
        VkPipelineInputAssemblyStateCreateInfo inputAssembly{VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO}; inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
        VkViewport viewport{}; viewport.width = (float)WIDTH; viewport.height = (float)HEIGHT; viewport.maxDepth = 1.0f;
        VkRect2D scissor{}; scissor.extent = {WIDTH, HEIGHT};
        VkPipelineViewportStateCreateInfo viewportState{VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO}; viewportState.viewportCount = 1; viewportState.pViewports = &viewport; viewportState.scissorCount = 1; viewportState.pScissors = &scissor;
        VkPipelineRasterizationStateCreateInfo rasterizer{VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO}; rasterizer.depthClampEnable = VK_FALSE; rasterizer.rasterizerDiscardEnable = VK_FALSE;
        rasterizer.polygonMode = VK_POLYGON_MODE_FILL; rasterizer.lineWidth = 1.0f; rasterizer.cullMode = VK_CULL_MODE_NONE; rasterizer.frontFace = VK_FRONT_FACE_CLOCKWISE;
        VkPipelineMultisampleStateCreateInfo multisampling{VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO}; multisampling.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
        VkPipelineColorBlendAttachmentState colorBlendAttachment{}; colorBlendAttachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT; colorBlendAttachment.blendEnable = VK_FALSE;
        VkPipelineColorBlendStateCreateInfo colorBlending{VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO}; colorBlending.attachmentCount = 1; colorBlending.pAttachments = &colorBlendAttachment;
        VkPipelineDynamicStateCreateInfo dynamicState{VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO};
        std::vector<VkDynamicState> dynamicStates = {VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
        dynamicState.dynamicStateCount = static_cast<uint32_t>(dynamicStates.size()); dynamicState.pDynamicStates = dynamicStates.data();
        VkPipelineLayoutCreateInfo pipelineLayoutInfo{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO}; pipelineLayoutInfo.setLayoutCount = 1; pipelineLayoutInfo.pSetLayouts = &descriptorSetLayout;
        if (vkCreatePipelineLayout(device, &pipelineLayoutInfo, nullptr, &pipelineLayout) != VK_SUCCESS) throw std::runtime_error("failed to create pipeline layout!");
        VkGraphicsPipelineCreateInfo pipelineInfo{VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO};
        pipelineInfo.stageCount = 2; pipelineInfo.pStages = shaderStages; pipelineInfo.pVertexInputState = &vertexInputInfo; pipelineInfo.pInputAssemblyState = &inputAssembly;
        pipelineInfo.pViewportState = &viewportState; pipelineInfo.pRasterizationState = &rasterizer; pipelineInfo.pMultisampleState = &multisampling;
        pipelineInfo.pColorBlendState = &colorBlending; pipelineInfo.pDynamicState = &dynamicState; pipelineInfo.layout = pipelineLayout; pipelineInfo.renderPass = renderPass; pipelineInfo.subpass = 0;
        if (vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &graphicsPipeline) != VK_SUCCESS) throw std::runtime_error("failed to create graphics pipeline!");
        vkDestroyShaderModule(device, fragShaderModule, nullptr); vkDestroyShaderModule(device, vertShaderModule, nullptr);
    }
    void createFramebuffers() {
        swapChainFramebuffers.resize(swapChainImageViews.size());
        for (size_t i = 0; i < swapChainImageViews.size(); i++) {
            VkImageView attachments[] = {swapChainImageViews[i]};
            VkFramebufferCreateInfo framebufferInfo{VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO};
            framebufferInfo.renderPass = renderPass; framebufferInfo.attachmentCount = 1; framebufferInfo.pAttachments = attachments;
            framebufferInfo.width = swapChainExtent.width; framebufferInfo.height = swapChainExtent.height; framebufferInfo.layers = 1;
            if (vkCreateFramebuffer(device, &framebufferInfo, nullptr, &swapChainFramebuffers[i]) != VK_SUCCESS) throw std::runtime_error("failed to create framebuffer!");
        }
    }
    void createCommandPool() {
        VkCommandPoolCreateInfo poolInfo{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO}; poolInfo.queueFamilyIndex = 0; poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
        if (vkCreateCommandPool(device, &poolInfo, nullptr, &commandPool) != VK_SUCCESS) throw std::runtime_error("failed to create command pool!");
    }
    uint32_t findMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags properties) {
        VkPhysicalDeviceMemoryProperties memProperties; vkGetPhysicalDeviceMemoryProperties(physicalDevice, &memProperties);
        for (uint32_t i = 0; i < memProperties.memoryTypeCount; i++) if ((typeFilter & (1 << i)) && (memProperties.memoryTypes[i].propertyFlags & properties) == properties) return i;
        throw std::runtime_error("failed to find suitable memory type!");
    }
    void createStagingBuffer() {
        VkDeviceSize size = WIDTH * HEIGHT * 4; VkBufferCreateInfo bufferInfo{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
        bufferInfo.size = size; bufferInfo.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
        if (vkCreateBuffer(device, &bufferInfo, nullptr, &stagingBuffer) != VK_SUCCESS) throw std::runtime_error("failed to create buffer");
        VkMemoryRequirements memRequirements; vkGetBufferMemoryRequirements(device, stagingBuffer, &memRequirements);
        VkMemoryAllocateInfo allocInfo{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
        allocInfo.allocationSize = memRequirements.size; allocInfo.memoryTypeIndex = findMemoryType(memRequirements.memoryTypeBits, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        if (vkAllocateMemory(device, &allocInfo, nullptr, &stagingBufferMemory) != VK_SUCCESS) throw std::runtime_error("failed to allocate buffer memory");
        vkBindBufferMemory(device, stagingBuffer, stagingBufferMemory, 0);
    }
    void createTextureImage() {
        VkImageCreateInfo imageInfo{VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO};
        imageInfo.imageType = VK_IMAGE_TYPE_2D; imageInfo.extent.width = WIDTH; imageInfo.extent.height = HEIGHT; imageInfo.extent.depth = 1; imageInfo.mipLevels = 1; imageInfo.arrayLayers = 1;
        imageInfo.format = VK_FORMAT_R8G8B8A8_SRGB; imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL; imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED; imageInfo.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT; imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;
        if (vkCreateImage(device, &imageInfo, nullptr, &textureImage) != VK_SUCCESS) throw std::runtime_error("failed to create image");
        VkMemoryRequirements memRequirements; vkGetImageMemoryRequirements(device, textureImage, &memRequirements);
        VkMemoryAllocateInfo allocInfo{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
        allocInfo.allocationSize = memRequirements.size; allocInfo.memoryTypeIndex = findMemoryType(memRequirements.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
        if (vkAllocateMemory(device, &allocInfo, nullptr, &textureImageMemory) != VK_SUCCESS) throw std::runtime_error("failed to allocate image memory");
        vkBindImageMemory(device, textureImage, textureImageMemory, 0);
    }
    void createTextureImageView() {
        VkImageViewCreateInfo viewInfo{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
        viewInfo.image = textureImage; viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D; viewInfo.format = VK_FORMAT_R8G8B8A8_SRGB;
        viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT; viewInfo.subresourceRange.levelCount = 1; viewInfo.subresourceRange.layerCount = 1;
        if (vkCreateImageView(device, &viewInfo, nullptr, &textureImageView) != VK_SUCCESS) throw std::runtime_error("failed to create texture image view!");
    }
    void createTextureSampler() {
        VkSamplerCreateInfo samplerInfo{VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO};
        samplerInfo.magFilter = VK_FILTER_NEAREST; samplerInfo.minFilter = VK_FILTER_NEAREST;
        samplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT; samplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT; samplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT;
        samplerInfo.anisotropyEnable = VK_FALSE; samplerInfo.maxAnisotropy = 1.0f; samplerInfo.borderColor = VK_BORDER_COLOR_INT_OPAQUE_BLACK;
        samplerInfo.unnormalizedCoordinates = VK_FALSE; samplerInfo.compareEnable = VK_FALSE; samplerInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
        if (vkCreateSampler(device, &samplerInfo, nullptr, &textureSampler) != VK_SUCCESS) throw std::runtime_error("failed to create texture sampler!");
    }
    void createDescriptorPool() {
        VkDescriptorPoolSize poolSize{}; poolSize.type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER; poolSize.descriptorCount = static_cast<uint32_t>(MAX_FRAMES_IN_FLIGHT);
        VkDescriptorPoolCreateInfo poolInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO}; poolInfo.poolSizeCount = 1; poolInfo.pPoolSizes = &poolSize; poolInfo.maxSets = static_cast<uint32_t>(MAX_FRAMES_IN_FLIGHT);
        if (vkCreateDescriptorPool(device, &poolInfo, nullptr, &descriptorPool) != VK_SUCCESS) throw std::runtime_error("failed to create descriptor pool!");
    }
    void createDescriptorSets() {
        std::vector<VkDescriptorSetLayout> layouts(MAX_FRAMES_IN_FLIGHT, descriptorSetLayout);
        VkDescriptorSetAllocateInfo allocInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        allocInfo.descriptorPool = descriptorPool; allocInfo.descriptorSetCount = static_cast<uint32_t>(MAX_FRAMES_IN_FLIGHT); allocInfo.pSetLayouts = layouts.data();
        descriptorSets.resize(MAX_FRAMES_IN_FLIGHT);
        if (vkAllocateDescriptorSets(device, &allocInfo, descriptorSets.data()) != VK_SUCCESS) throw std::runtime_error("failed to allocate descriptor sets!");
        for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) {
            VkDescriptorImageInfo imageInfo{}; imageInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL; imageInfo.imageView = textureImageView; imageInfo.sampler = textureSampler;
            VkWriteDescriptorSet descriptorWrite{VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
            descriptorWrite.dstSet = descriptorSets[i]; descriptorWrite.dstBinding = 0; descriptorWrite.dstArrayElement = 0;
            descriptorWrite.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER; descriptorWrite.descriptorCount = 1; descriptorWrite.pImageInfo = &imageInfo;
            vkUpdateDescriptorSets(device, 1, &descriptorWrite, 0, nullptr);
        }
    }
    void createCommandBuffers() {
        commandBuffers.resize(MAX_FRAMES_IN_FLIGHT);
        VkCommandBufferAllocateInfo allocInfo{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
        allocInfo.commandPool = commandPool; allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY; allocInfo.commandBufferCount = (uint32_t)commandBuffers.size();
        if (vkAllocateCommandBuffers(device, &allocInfo, commandBuffers.data()) != VK_SUCCESS) throw std::runtime_error("failed to allocate command buffers!");
    }
    void createSyncObjects() {
        imageAvailableSemaphores.resize(MAX_FRAMES_IN_FLIGHT); renderFinishedSemaphores.resize(MAX_FRAMES_IN_FLIGHT); inFlightFences.resize(MAX_FRAMES_IN_FLIGHT);
        VkSemaphoreCreateInfo semaphoreInfo{VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO}; VkFenceCreateInfo fenceInfo{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO}; fenceInfo.flags = VK_FENCE_CREATE_SIGNALED_BIT;
        for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) {
            if (vkCreateSemaphore(device, &semaphoreInfo, nullptr, &imageAvailableSemaphores[i]) != VK_SUCCESS ||
                vkCreateSemaphore(device, &semaphoreInfo, nullptr, &renderFinishedSemaphores[i]) != VK_SUCCESS ||
                vkCreateFence(device, &fenceInfo, nullptr, &inFlightFences[i]) != VK_SUCCESS) throw std::runtime_error("failed to create synchronization objects!");
        }
    }
    VkShaderModule createShaderModule(const std::vector<char> &code) {
        VkShaderModuleCreateInfo createInfo{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO}; createInfo.codeSize = code.size(); createInfo.pCode = reinterpret_cast<const uint32_t *>(code.data());
        VkShaderModule shaderModule; if (vkCreateShaderModule(device, &createInfo, nullptr, &shaderModule) != VK_SUCCESS) throw std::runtime_error("failed to create shader module!");
        return shaderModule;
    }
    void mainLoop() { while (!glfwWindowShouldClose(window)) { glfwPollEvents(); processInput(); drawFrame(); } vkDeviceWaitIdle(device); }
    void drawFrame() {
        vkWaitForFences(device, 1, &inFlightFences[currentFrame], VK_TRUE, UINT64_MAX);
        uint32_t imageIndex;
        VkResult result = vkAcquireNextImageKHR(device, swapChain, UINT64_MAX, imageAvailableSemaphores[currentFrame], VK_NULL_HANDLE, &imageIndex);
        if (result == VK_ERROR_OUT_OF_DATE_KHR) { recreateSwapChain(); return; } else if (result != VK_SUCCESS && result != VK_SUBOPTIMAL_KHR) throw std::runtime_error("failed to acquire swap chain image!");
        vkResetFences(device, 1, &inFlightFences[currentFrame]);
        
        // Update Texture with CPU Rasterizer Result
        updateTexture();

        vkResetCommandBuffer(commandBuffers[currentFrame], 0);
        VkCommandBuffer cb = commandBuffers[currentFrame];
        VkCommandBufferBeginInfo beginInfo{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO}; vkBeginCommandBuffer(cb, &beginInfo);
        VkImageMemoryBarrier b1{VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER};
        b1.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED; b1.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
        b1.image = textureImage; b1.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
        b1.srcAccessMask = 0; b1.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
        vkCmdPipelineBarrier(cb, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 0, nullptr, 1, &b1);
        VkBufferImageCopy region{}; region.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1}; region.imageExtent = {WIDTH, HEIGHT, 1};
        vkCmdCopyBufferToImage(cb, stagingBuffer, textureImage, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);
        VkImageMemoryBarrier b2 = b1; b2.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL; b2.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        b2.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT; b2.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
        vkCmdPipelineBarrier(cb, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, 0, 0, nullptr, 0, nullptr, 1, &b2);
        VkRenderPassBeginInfo renderPassInfo{VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO};
        renderPassInfo.renderPass = renderPass; renderPassInfo.framebuffer = swapChainFramebuffers[imageIndex];
        renderPassInfo.renderArea.extent = swapChainExtent; VkClearValue clearColor = {{{0.0f, 0.0f, 0.0f, 1.0f}}};
        renderPassInfo.clearValueCount = 1; renderPassInfo.pClearValues = &clearColor;
        vkCmdBeginRenderPass(cb, &renderPassInfo, VK_SUBPASS_CONTENTS_INLINE);
        vkCmdBindPipeline(cb, VK_PIPELINE_BIND_POINT_GRAPHICS, graphicsPipeline);
        VkViewport viewport{}; viewport.width = (float)swapChainExtent.width; viewport.height = (float)swapChainExtent.height; viewport.maxDepth = 1.0f;
        vkCmdSetViewport(cb, 0, 1, &viewport); VkRect2D scissor{}; scissor.extent = swapChainExtent; vkCmdSetScissor(cb, 0, 1, &scissor);
        vkCmdBindDescriptorSets(cb, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 0, 1, &descriptorSets[currentFrame], 0, nullptr);
        vkCmdDraw(cb, 3, 1, 0, 0); vkCmdEndRenderPass(cb); vkEndCommandBuffer(cb);
        VkSubmitInfo submitInfo{VK_STRUCTURE_TYPE_SUBMIT_INFO};
        VkSemaphore waitSemaphores[] = {imageAvailableSemaphores[currentFrame]}; VkPipelineStageFlags waitStages[] = {VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT};
        submitInfo.waitSemaphoreCount = 1; submitInfo.pWaitSemaphores = waitSemaphores; submitInfo.pWaitDstStageMask = waitStages;
        submitInfo.commandBufferCount = 1; submitInfo.pCommandBuffers = &commandBuffers[currentFrame];
        VkSemaphore signalSemaphores[] = {renderFinishedSemaphores[currentFrame]}; submitInfo.signalSemaphoreCount = 1; submitInfo.pSignalSemaphores = signalSemaphores;
        if (vkQueueSubmit(graphicsQueue, 1, &submitInfo, inFlightFences[currentFrame]) != VK_SUCCESS) throw std::runtime_error("failed to submit draw command buffer!");
        VkPresentInfoKHR presentInfo{VK_STRUCTURE_TYPE_PRESENT_INFO_KHR};
        presentInfo.waitSemaphoreCount = 1; presentInfo.pWaitSemaphores = signalSemaphores;
        VkSwapchainKHR swapChains[] = {swapChain}; presentInfo.swapchainCount = 1; presentInfo.pSwapchains = swapChains; presentInfo.pImageIndices = &imageIndex;
        result = vkQueuePresentKHR(presentQueue, &presentInfo);
        if (result == VK_ERROR_OUT_OF_DATE_KHR || result == VK_SUBOPTIMAL_KHR || framebufferResized) { framebufferResized = false; recreateSwapChain(); }
        currentFrame = (currentFrame + 1) % MAX_FRAMES_IN_FLIGHT; vkQueueWaitIdle(presentQueue);
    }
    void cleanupSwapChain() {
        for (auto framebuffer : swapChainFramebuffers) vkDestroyFramebuffer(device, framebuffer, nullptr);
        for (auto imageView : swapChainImageViews) vkDestroyImageView(device, imageView, nullptr);
        vkDestroySwapchainKHR(device, swapChain, nullptr);
    }
    void recreateSwapChain() {
        int width = 0, height = 0; glfwGetFramebufferSize(window, &width, &height);
        while (width == 0 || height == 0) { glfwGetFramebufferSize(window, &width, &height); glfwWaitEvents(); }
        vkDeviceWaitIdle(device); cleanupSwapChain(); createSwapChain(); createImageViews(); createFramebuffers();
    }
    void cleanup() {
        cleanupSwapChain();
        vkDestroySampler(device, textureSampler, nullptr); vkDestroyImageView(device, textureImageView, nullptr);
        vkDestroyImage(device, textureImage, nullptr); vkFreeMemory(device, textureImageMemory, nullptr);
        vkDestroyBuffer(device, stagingBuffer, nullptr); vkFreeMemory(device, stagingBufferMemory, nullptr);
        vkDestroyDescriptorPool(device, descriptorPool, nullptr); vkDestroyDescriptorSetLayout(device, descriptorSetLayout, nullptr);
        vkDestroyPipeline(device, graphicsPipeline, nullptr); vkDestroyPipelineLayout(device, pipelineLayout, nullptr);
        vkDestroyRenderPass(device, renderPass, nullptr);
        for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) {
            vkDestroySemaphore(device, renderFinishedSemaphores[i], nullptr); vkDestroySemaphore(device, imageAvailableSemaphores[i], nullptr);
            vkDestroyFence(device, inFlightFences[i], nullptr);
        }
        vkDestroyCommandPool(device, commandPool, nullptr); vkDestroyDevice(device, nullptr);
        if (enableValidationLayers && debugMessenger != VK_NULL_HANDLE) DestroyDebugUtilsMessengerEXT(instance, debugMessenger, nullptr);
        vkDestroySurfaceKHR(instance, surface, nullptr); vkDestroyInstance(instance, nullptr);
        glfwDestroyWindow(window); glfwTerminate();
    }
    void DestroyDebugUtilsMessengerEXT(VkInstance instance, VkDebugUtilsMessengerEXT debugMessenger, const VkAllocationCallbacks *pAllocator) {
        auto func = (PFN_vkDestroyDebugUtilsMessengerEXT)vkGetInstanceProcAddr(instance, "vkDestroyDebugUtilsMessengerEXT");
        if (func != nullptr) func(instance, debugMessenger, pAllocator);
    }
};

int main(int argc, char **argv)
{
    HelloVulkanApplication app;
    const char *objFile = nullptr;
    int mode = 1;
    int scenario = 0;
    bool benchmark = false;
    bool modeSet = false; 

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        std::string lowerArg = arg;
        std::transform(lowerArg.begin(), lowerArg.end(), lowerArg.begin(), ::tolower);

        if (lowerArg == "benchmarkmode" || lowerArg == "yes" || lowerArg == "true" || lowerArg == "on") {
            benchmark = true; continue;
        }
        if (lowerArg == "no" || lowerArg == "false" || lowerArg == "off") {
            benchmark = false; continue;
        }
        if (arg.find(".obj") != std::string::npos) {
            objFile = argv[i]; continue;
        }
        try {
            int val = std::stoi(arg);
            if (!modeSet && val >= 1 && val <= 4) { mode = val; modeSet = true; } 
            else { scenario = val; }
        } catch (...) {}
    }

    if (!objFile) {
        std::cout << "[Warning] No .obj file specified, using default." << std::endl;
        std::cout << "Usage: ./VulkanApp.exe model.obj [mode] [scenario] [yes/no]" << std::endl;
    }
    
    try { app.run(objFile, mode, scenario, benchmark); } 
    catch (const std::exception &e) { std::cerr << "FATAL ERROR: " << e.what() << std::endl; return EXIT_FAILURE; }
    return EXIT_SUCCESS;
}