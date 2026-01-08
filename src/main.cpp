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
        res.m[1][1] = -1.0f / tanHalfFov; // Y轴反转适配 Vulkan
        res.m[2][2] = -(zfar + znear) / (zfar - znear);
        res.m[2][3] = -(2.0f * zfar * znear) / (zfar - znear);
        res.m[3][2] = -1.0f;
        return res;
    }
    static Mat4 scale(float s)
    {
        Mat4 res = identity();
        res.m[0][0] = s;
        res.m[1][1] = s;
        res.m[2][2] = s;
        return res;
    }
    static Mat4 translate(float x, float y, float z)
    {
        Mat4 res = identity();
        res.m[0][3] = x;
        res.m[1][3] = y;
        res.m[2][3] = z;
        return res;
    }
    static Mat4 rotateX(float angle)
    {
        Mat4 res = identity();
        float c = std::cos(angle), s = std::sin(angle);
        res.m[1][1] = c;
        res.m[1][2] = -s;
        res.m[2][1] = s;
        res.m[2][2] = c;
        return res;
    }
    static Mat4 rotateY(float angle)
    {
        Mat4 res = identity();
        float c = std::cos(angle), s = std::sin(angle);
        res.m[0][0] = c;
        res.m[0][2] = s;
        res.m[2][0] = -s;
        res.m[2][2] = c;
        return res;
    }
    static Mat4 rotateZ(float angle)
    {
        Mat4 res = identity();
        float c = std::cos(angle), s = std::sin(angle);
        res.m[0][0] = c;
        res.m[0][1] = -s;
        res.m[1][0] = s;
        res.m[1][1] = c;
        return res;
    }

    Vec3 transformPoint(const Vec3 &v) const
    {
        float x = m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z + m[0][3];
        float y = m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z + m[1][3];
        float z = m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z + m[2][3];
        float w = m[3][0] * v.x + m[3][1] * v.y + m[3][2] * v.z + m[3][3];
        if (w != 0.0f)
        {
            x /= w;
            y /= w;
            z /= w;
        }
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

struct Mesh
{
    std::vector<Vec3> vertices;
    std::vector<int> faces;
    Vec3 centerOffset = {0, 0, 0};
    float normalizeScale = 1.0f;
    float minY = std::numeric_limits<float>::max();
    float maxY = std::numeric_limits<float>::lowest();

    struct BVHNode {
        Vec3 minB, maxB; // 节点的 AABB 包围盒
        int left = -1;   // 左子节点索引
        int right = -1;  // 右子节点索引
        int triStart = -1; // 如果是叶子节点，指向 faces 的起始索引
        int triCount = 0;  // 叶子节点包含的三角形数量
        bool isLeaf() const { return left == -1 && right == -1; }
    };

    std::vector<BVHNode> bvhNodes;

    // 递归构建 BVH
    // triIndices: 当前节点包含的三角形索引列表
    int buildBVHRecursive(std::vector<int>& triIndices, int depth) {
        BVHNode node;
        
        // 1. 计算当前所有三角形的包围盒
        node.minB = {std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
        node.maxB = {std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest()};

        for (int ti : triIndices) {
            // 获取三角形三个顶点
            for (int k = 0; k < 3; k++) {
                Vec3 v = vertices[faces[ti * 3 + k]];
                if (v.x < node.minB.x) node.minB.x = v.x; if (v.x > node.maxB.x) node.maxB.x = v.x;
                if (v.y < node.minB.y) node.minB.y = v.y; if (v.y > node.maxB.y) node.maxB.y = v.y;
                if (v.z < node.minB.z) node.minB.z = v.z; if (v.z > node.maxB.z) node.maxB.z = v.z;
            }
        }

        // 2. 终止条件：三角形太少或深度太深，作为叶子节点
        if (triIndices.size() <= 16 || depth > 20) {
            node.triStart = (int)sortedFaces.size() / 3; // 记录在新列表中的位置
            node.triCount = (int)triIndices.size();
            
            // 将三角形索引写入排序后的列表 (为了内存连续性)
            for (int ti : triIndices) {
                sortedFaces.push_back(faces[ti * 3 + 0]);
                sortedFaces.push_back(faces[ti * 3 + 1]);
                sortedFaces.push_back(faces[ti * 3 + 2]);
            }
            
            bvhNodes.push_back(node);
            return (int)bvhNodes.size() - 1;
        }

        // 3. 分裂：选择最长轴的中点进行划分
        Vec3 size = node.maxB - node.minB;
        int axis = 0;
        if (size.y > size.x) axis = 1;
        if (size.z > size.y && size.z > size.x) axis = 2;

        float mid = (axis == 0 ? (node.minB.x + node.maxB.x) : (axis == 1 ? (node.minB.y + node.maxB.y) : (node.minB.z + node.maxB.z))) * 0.5f;

        std::vector<int> leftTris, rightTris;
        for (int ti : triIndices) {
            // 计算三角形重心
            float center = 0;
            for(int k=0; k<3; k++) {
                Vec3 v = vertices[faces[ti * 3 + k]];
                center += (axis == 0 ? v.x : (axis == 1 ? v.y : v.z));
            }
            center /= 3.0f;

            if (center < mid) leftTris.push_back(ti);
            else rightTris.push_back(ti);
        }

        // 极端情况处理：如果一边为空，强制设为叶子
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

        // 4. 递归构建子节点
        // 既然不是叶子，先把当前节点存入数组拿到 index (注意：递归可能会导致 vector 扩容，指针失效，所以存 index)
        int currIndex = (int)bvhNodes.size();
        bvhNodes.push_back(node);

        int lIdx = buildBVHRecursive(leftTris, depth + 1);
        int rIdx = buildBVHRecursive(rightTris, depth + 1);

        bvhNodes[currIndex].left = lIdx;
        bvhNodes[currIndex].right = rIdx;

        return currIndex;
    }

    std::vector<int> sortedFaces; // 构建 BVH 后重新排列的面索引

    void buildBVH() {
        if (vertices.empty()) return;
        std::cout << "Building BVH..." << std::endl;
        
        bvhNodes.clear();
        bvhNodes.reserve(faces.size() / 2); // 预分配避免频繁扩容
        sortedFaces.clear();
        sortedFaces.reserve(faces.size());

        std::vector<int> allTriIndices(faces.size() / 3);
        for(int i=0; i<allTriIndices.size(); i++) allTriIndices[i] = i;

        buildBVHRecursive(allTriIndices, 0);

        // 替换原始 faces 为排序后的 faces，提高缓存命中率
        faces = sortedFaces; 
        std::cout << "BVH Built. Nodes: " << bvhNodes.size() << std::endl;
    }


    static Mesh loadObj(const std::string &filename)
    {
        Mesh mesh;
        std::ifstream file(filename);

        Vec3 minB = {std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
        Vec3 maxB = {std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest()};

        if (!file.is_open())
        {
            std::cerr << "Failed to open OBJ file: " << filename << ". Loading default triangle." << std::endl;
            mesh.vertices = {{-1.0f, 1.0f, 0.0f}, {1.0f, 1.0f, 0.0f}, {0.0f, -1.0f, 0.0f}};
            mesh.faces = {0, 1, 2};
            return mesh;
        }

        std::string line;
        while (std::getline(file, line))
        {
            std::stringstream ss(line);
            std::string prefix;
            ss >> prefix;
            if (prefix == "v")
            {
                Vec3 v;
                ss >> v.x >> v.y >> v.z;
                mesh.vertices.push_back(v);

                if (v.x < minB.x) minB.x = v.x; if (v.x > maxB.x) maxB.x = v.x;
                if (v.y < minB.y) minB.y = v.y; if (v.y > maxB.y) maxB.y = v.y;
                if (v.z < minB.z) minB.z = v.z; if (v.z > maxB.z) maxB.z = v.z;
            }
            else if (prefix == "f")
            {
                std::string segment;
                for (int i = 0; i < 3; i++)
                {
                    ss >> segment;
                    size_t slashPos = segment.find('/');
                    int idx = std::stoi(segment.substr(0, slashPos));
                    mesh.faces.push_back(idx - 1);
                }
            }
        }
        mesh.minY = minB.y;
        mesh.maxY = maxB.y;

        if (!mesh.vertices.empty())
        {
            Vec3 center = {(minB.x + maxB.x) / 2.0f, (minB.y + maxB.y) / 2.0f, (minB.z + maxB.z) / 2.0f};
            mesh.centerOffset = {-center.x, -center.y, -center.z};
            float sizeX = maxB.x - minB.x;
            float sizeY = maxB.y - minB.y;
            float sizeZ = maxB.z - minB.z;
            float maxDim = std::max({sizeX, sizeY, sizeZ});
            float targetSize = 1.74f;
            if (maxDim > 0)
            {
                mesh.normalizeScale = targetSize / maxDim;
            }
            std::cout << "Model Bounds: " << minB.y << " to " << maxB.y << std::endl;
        }
        mesh.buildBVH();
        return mesh;
    }
};

// -----------------------------------------------------------
// 全局设置
// -----------------------------------------------------------
const uint32_t WIDTH = 1280;
const uint32_t HEIGHT = 720;
const int MAX_FRAMES_IN_FLIGHT = 2;

bool keys[1024] = {false};
float cameraX = 0.0f, cameraY = 0.0f, cameraZ = 3.5f;
float lastX = WIDTH / 2.0f, lastY = HEIGHT / 2.0f;
bool firstMouse = true;
bool leftMouseDown = false;
bool rightMouseDown = false;

int currentAxis = 1;                     
Vec3 modelRotation = {180.0f, 0.0f, 0.0f}; // 初始翻转180度
bool autoRotate = true;                  

// 渲染模式：1=Standard ZBuffer, 2=Scanline, 3=Hierarchy
int g_renderMode = 1; 

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

static std::vector<char> readFile(const std::string &filename)
{
    std::ifstream file(filename, std::ios::ate | std::ios::binary);
    if (!file.is_open())
        throw std::runtime_error("failed to open file: " + filename);
    size_t fileSize = (size_t)file.tellg();
    std::vector<char> buffer(fileSize);
    file.seekg(0);
    file.read(buffer.data(), fileSize);
    file.close();
    return buffer;
}

// -----------------------------------------------------------
// 应用程序类
// -----------------------------------------------------------
class HelloVulkanApplication
{
public:
    void run(const char *objFilename, int mode)
    {
        g_renderMode = mode;
        std::cout << "Starting with Render Mode: " << g_renderMode << std::endl;
        std::cout << "1: Standard Z-Buffer | 2: Scanline Z-Buffer | 3: Hierarchy Z-Buffer" << std::endl;

        if (objFilename)
            g_mesh = Mesh::loadObj(objFilename);
        else
            g_mesh = Mesh::loadObj("dummy.obj");

        initWindow();
        initVulkan();

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

    // --- Benchmark 变量 ---
    bool benchmarkMode = true;         // 默认开启跑分以便测试
    int benchmarkFrameCount = 0;       // 当前帧数计数
    const int TOTAL_TEST_FRAMES = 500; // 测试总帧数
    std::chrono::high_resolution_clock::time_point startTime;


    // HZB
    std::vector<std::vector<float>> hzb;
    std::vector<std::pair<int, int>> hzbDims;
    std::vector<Vec3> cachedProjectedVerts;

    float getHzbDepth(int level, int x, int y)
    {
        if (level >= hzb.size()) return 1.0f;
        int w = hzbDims[level].first;
        int h = hzbDims[level].second;
        x = std::clamp(x, 0, w - 1);
        y = std::clamp(y, 0, h - 1);
        return hzb[level][y * w + x];
    }

    static void framebufferResizeCallback(GLFWwindow *window, int width, int height)
    {
        auto app = reinterpret_cast<HelloVulkanApplication *>(glfwGetWindowUserPointer(window));
        app->framebufferResized = true;
    }

    static void keyCallback(GLFWwindow *window, int key, int scancode, int action, int mods)
    {
        if (key >= 0 && key < 1024) keys[key] = (action != GLFW_RELEASE);

        if (key == GLFW_KEY_Q && action == GLFW_PRESS)
        {
            currentAxis = (currentAxis + 1) % 3;
            std::cout << "Switch Axis: " << currentAxis << std::endl;
        }
        if (key == GLFW_KEY_SPACE && action == GLFW_PRESS)
        {
            autoRotate = !autoRotate;
            std::cout << "Auto Rotate: " << autoRotate << std::endl;
        }
        // 动态切换模式（可选）
        if (key == GLFW_KEY_1 && action == GLFW_PRESS) g_renderMode = 1;
        if (key == GLFW_KEY_2 && action == GLFW_PRESS) g_renderMode = 2;
        if (key == GLFW_KEY_3 && action == GLFW_PRESS) g_renderMode = 3;
    }

    static void scrollCallback(GLFWwindow *window, double xoffset, double yoffset)
    {
        float zoomSpeed = 0.2f;
        cameraZ -= (float)yoffset * zoomSpeed;
        if (cameraZ < 0.1f) cameraZ = 0.1f;
        if (cameraZ > 20.0f) cameraZ = 20.0f;
    }

    static void mouseButtonCallback(GLFWwindow *window, int button, int action, int mods)
    {
        if (button == GLFW_MOUSE_BUTTON_LEFT) {
            leftMouseDown = (action == GLFW_PRESS);
            if (leftMouseDown) autoRotate = false;
        }
        if (button == GLFW_MOUSE_BUTTON_RIGHT) {
            rightMouseDown = (action == GLFW_PRESS);
        }
    }

    static void cursorPosCallback(GLFWwindow *window, double xposIn, double yposIn)
    {
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

    static VKAPI_ATTR VkBool32 VKAPI_CALL debugCallback(VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity, VkDebugUtilsMessageTypeFlagsEXT messageType, const VkDebugUtilsMessengerCallbackDataEXT *pCallbackData, void *pUserData)
    {
        std::cerr << "validation layer: " << pCallbackData->pMessage << std::endl;
        return VK_FALSE;
    }

    void initWindow()
    {
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

    void processInput()
    {
        if (keys[GLFW_KEY_ESCAPE]) glfwSetWindowShouldClose(window, true);
    }

    // 重心坐标辅助函数
    Vec3 barycentric(Vec3 p, Vec3 a, Vec3 b, Vec3 c)
    {
        Vec3 v0 = b - a, v1 = c - a, v2 = p - a;
        float d00 = v0.dot(v0);
        float d01 = v0.dot(v1);
        float d11 = v1.dot(v1);
        float d20 = v2.dot(v0);
        float d21 = v2.dot(v1);
        float denom = d00 * d11 - d01 * d01;
        if (std::abs(denom) < 1e-5) return {-1, -1, -1};
        float v = (d11 * d20 - d01 * d21) / denom;
        float w = (d00 * d21 - d01 * d20) / denom;
        float u = 1.0f - v - w;
        return {u, v, w};
    }

    void buildHZB()
    {
        int w = WIDTH;
        int h = HEIGHT;

        if (hzb.empty()) {
            hzb.push_back(zBuffer); hzbDims.push_back({w, h});
        } else {
            hzb[0] = zBuffer; hzbDims[0] = {w, h};
        }

        int level = 0;
        while (w > 1 || h > 1)
        {
            int prevW = w; int prevH = h;
            w = (w + 1) / 2; h = (h + 1) / 2;
            level++;
            if (hzb.size() <= level) {
                hzb.push_back(std::vector<float>(w * h)); hzbDims.push_back({w, h});
            } else {
                if (hzb[level].size() != w * h) hzb[level].resize(w * h);
                hzbDims[level] = {w, h};
            }
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

    bool queryHZB(int minX, int maxX, int minY, int maxY, float minZ)
    {
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

    // --- 扫描线光栅化辅助函数 ---
    void scanlineRasterizeTri(Vec3 v0, Vec3 v1, Vec3 v2, uint8_t r, uint8_t g, uint8_t b)
    {
        // 1. 按 Y 坐标排序 (v0.y <= v1.y <= v2.y)
        if (v0.y > v1.y) std::swap(v0, v1);
        if (v0.y > v2.y) std::swap(v0, v2);
        if (v1.y > v2.y) std::swap(v1, v2);

        // 辅助结构体
        struct Edge {
            float x, dx; // 当前 x 和 x 的增量
            float z, dz; // 当前 z 和 z 的增量
        };

        auto initEdge = [](const Vec3& start, const Vec3& end) -> Edge {
            Edge e;
            e.x = start.x;
            e.z = start.z;
            float dy = end.y - start.y;
            if (dy > 0) {
                e.dx = (end.x - start.x) / dy;
                e.dz = (end.z - start.z) / dy;
            } else {
                e.dx = 0; e.dz = 0;
            }
            return e;
        };

        // 总高度
        int totalHeight = (int)(v2.y - v0.y);
        if (totalHeight == 0) return;

        // 分两段扫描: 上半部 (v0-v1) 和 下半部 (v1-v2)
        // 长边: v0 -> v2
        // 短边1: v0 -> v1
        // 短边2: v1 -> v2

        // 定义 lambda 绘制扫描线
        auto drawSpan = [&](int y, float xStart, float xEnd, float zStart, float zEnd) {
            if (y < 0 || y >= HEIGHT) return;
            
            // 确保 xStart < xEnd
            if (xStart > xEnd) {
                std::swap(xStart, xEnd);
                std::swap(zStart, zEnd);
            }

            int iXStart = (int)xStart;
            int iXEnd = (int)xEnd;
            
            // 简单的 Z 线性插值步长
            float width = xEnd - xStart;
            float dzStep = (width > 0) ? (zEnd - zStart) / width : 0;
            float currentZ = zStart;

            // 裁剪 X
            int startX = std::max(0, iXStart);
            int endX = std::min((int)WIDTH - 1, iXEnd);
            
            // 如果被裁掉了一部分，修正 Z 起始值
            if (startX > iXStart) {
                currentZ += (startX - iXStart) * dzStep;
            }

            for (int x = startX; x <= endX; x++) {
                int idx = y * WIDTH + x;
                if (currentZ < zBuffer[idx]) {
                    zBuffer[idx] = currentZ;
                    int pIdx = idx * 4;
                    pixels[pIdx + 0] = r;
                    pixels[pIdx + 1] = g;
                    pixels[pIdx + 2] = b;
                    pixels[pIdx + 3] = 255;
                }
                currentZ += dzStep;
            }
        };

        // 边相关数据
        Edge longEdge = initEdge(v0, v2);
        Edge shortEdge1 = initEdge(v0, v1);
        Edge shortEdge2 = initEdge(v1, v2);

        // --- Phase 1: 上半部分 (v0 到 v1) ---
        int yStart = (int)v0.y;
        int yEnd = (int)v1.y;
        
        float curX_Long = longEdge.x;
        float curZ_Long = longEdge.z;
        float curX_Short = shortEdge1.x;
        float curZ_Short = shortEdge1.z;

        for (int y = yStart; y < yEnd; y++) {
            drawSpan(y, curX_Long, curX_Short, curZ_Long, curZ_Short);
            curX_Long += longEdge.dx;
            curZ_Long += longEdge.dz;
            curX_Short += shortEdge1.dx;
            curZ_Short += shortEdge1.dz;
        }

        // --- Phase 2: 下半部分 (v1 到 v2) ---
        // 注意：长边的 X/Z 必须保持从上一阶段的积累，但短边切换到 Edge2
        // 修正长边在 Phase 2 的起点 (因为浮点误差，可能需要重新基于 v0 计算或者继续累加，这里继续累加)
        // 短边2从 v1 开始
        curX_Short = v1.x;
        curZ_Short = v1.z;
        
        yStart = (int)v1.y;
        yEnd = (int)v2.y;

        for (int y = yStart; y <= yEnd; y++) {
            drawSpan(y, curX_Long, curX_Short, curZ_Long, curZ_Short);
            curX_Long += longEdge.dx;
            curZ_Long += longEdge.dz;
            curX_Short += shortEdge2.dx;
            curZ_Short += shortEdge2.dz;
        }
    }

    // --- 核心光栅化函数 ---
    void softRasterize()
    {
        // 1. 基础状态更新
        float currentFrameTime = (float)glfwGetTime();
        float deltaTime = currentFrameTime - lastFrameTime;
        lastFrameTime = currentFrameTime;

        if (autoRotate) {
            float speed = 50.0f * deltaTime;
            if (currentAxis == 0) modelRotation.x += speed;
            else if (currentAxis == 1) modelRotation.y += speed;
            else if (currentAxis == 2) modelRotation.z += speed;
        }

        std::fill(pixels.begin(), pixels.end(), 30);
        std::fill(zBuffer.begin(), zBuffer.end(), 1.0f);

        // 2. 顶点变换 (Vertex Processing)
        // 所有模式共享这一步
        float radX = modelRotation.x * 3.14159f / 180.0f;
        float radY = modelRotation.y * 3.14159f / 180.0f;
        float radZ = modelRotation.z * 3.14159f / 180.0f;
        Mat4 matOffset = Mat4::translate(g_mesh.centerOffset.x, g_mesh.centerOffset.y, g_mesh.centerOffset.z);
        Mat4 matScale = Mat4::scale(g_mesh.normalizeScale);
        Mat4 matRot = Mat4::rotateZ(radZ) * Mat4::rotateY(radY) * Mat4::rotateX(radX);
        Mat4 matWorld = Mat4::translate(0, 0, 0);
        Mat4 view = Mat4::translate(-cameraX, -cameraY, -cameraZ);
        Mat4 proj = Mat4::perspective(45.0f * 3.14159f / 180.0f, (float)WIDTH / HEIGHT, 0.1f, 100.0f);
        Mat4 mvp = proj * view * matWorld * matRot * matScale * matOffset;

        if (cachedProjectedVerts.size() != g_mesh.vertices.size()) {
            cachedProjectedVerts.resize(g_mesh.vertices.size());
        }

        // Vertex Shader Stage
        #pragma omp parallel for
        for (int i = 0; i < (int)g_mesh.vertices.size(); i++) {
            cachedProjectedVerts[i] = mvp.transformPoint(g_mesh.vertices[i]);
        }

        Vec3 lightDir = Vec3{0.5f, 1.0f, 0.5f}.normalize();

        // 3. 根据模式选择光栅化策略
        if (g_renderMode == 1) 
        {
            // --- Mode 1: Standard Z-Buffer (Bounding Box) ---
            for (size_t i = 0; i < g_mesh.faces.size(); i += 3)
            {
                Vec3 p0 = cachedProjectedVerts[g_mesh.faces[i]];
                Vec3 p1 = cachedProjectedVerts[g_mesh.faces[i+1]];
                Vec3 p2 = cachedProjectedVerts[g_mesh.faces[i+2]];

                if (p0.z < 0 || p0.z > 1 || p1.z < 0 || p1.z > 1 || p2.z < 0 || p2.z > 1) continue;

                // 屏幕映射
                float sx0 = (p0.x + 1.0f) * 0.5f * WIDTH;  float sy0 = (1.0f - p0.y) * 0.5f * HEIGHT;
                float sx1 = (p1.x + 1.0f) * 0.5f * WIDTH;  float sy1 = (1.0f - p1.y) * 0.5f * HEIGHT;
                float sx2 = (p2.x + 1.0f) * 0.5f * WIDTH;  float sy2 = (1.0f - p2.y) * 0.5f * HEIGHT;

                int minX = std::max(0, (int)std::min({sx0, sx1, sx2}));
                int maxX = std::min((int)WIDTH - 1, (int)std::max({sx0, sx1, sx2}));
                int minY = std::max(0, (int)std::min({sy0, sy1, sy2}));
                int maxY = std::min((int)HEIGHT - 1, (int)std::max({sy0, sy1, sy2}));

                // 计算光照颜色 (Face Color)
                Vec3 v0 = g_mesh.vertices[g_mesh.faces[i]];
                Vec3 v1 = g_mesh.vertices[g_mesh.faces[i+1]];
                Vec3 v2 = g_mesh.vertices[g_mesh.faces[i+2]];
                Vec3 rawNormal = (v1 - v0).cross(v2 - v0).normalize();
                Vec3 rotNormal = matRot.transformPoint(rawNormal); 
                float intensity = std::max(0.0f, rotNormal.dot(lightDir));
                float lightFactor = 0.2f + 0.8f * intensity; 
                uint8_t colorVal = (uint8_t)std::clamp(255.0f * lightFactor, 0.0f, 255.0f);

                // 包围盒光栅化
                for (int y = minY; y <= maxY; y++) {
                    for (int x = minX; x <= maxX; x++) {
                        Vec3 P = {(float)x, (float)y, 0};
                        Vec3 bc = barycentric(P, {sx0, sy0, 0}, {sx1, sy1, 0}, {sx2, sy2, 0});
                        if (bc.x >= 0 && bc.y >= 0 && bc.z >= 0) {
                            float z = bc.x * p0.z + bc.y * p1.z + bc.z * p2.z;
                            int idx = y * WIDTH + x;
                            if (z < zBuffer[idx]) {
                                zBuffer[idx] = z;
                                int pIdx = idx * 4;
                                pixels[pIdx + 0] = colorVal; pixels[pIdx + 1] = colorVal; pixels[pIdx + 2] = colorVal; pixels[pIdx + 3] = 255;
                            }
                        }
                    }
                }
            }
        }
        else if (g_renderMode == 2)
        {
            // --- Mode 2: Scanline Z-Buffer ---
            for (size_t i = 0; i < g_mesh.faces.size(); i += 3)
            {
                Vec3 p0 = cachedProjectedVerts[g_mesh.faces[i]];
                Vec3 p1 = cachedProjectedVerts[g_mesh.faces[i+1]];
                Vec3 p2 = cachedProjectedVerts[g_mesh.faces[i+2]];

                if (p0.z < 0 || p0.z > 1 || p1.z < 0 || p1.z > 1 || p2.z < 0 || p2.z > 1) continue;

                // 屏幕映射 (Scanline 需要具体的浮点坐标)
                Vec3 v0_scr, v1_scr, v2_scr;
                v0_scr.x = (p0.x + 1.0f) * 0.5f * WIDTH;  v0_scr.y = (1.0f - p0.y) * 0.5f * HEIGHT; v0_scr.z = p0.z;
                v1_scr.x = (p1.x + 1.0f) * 0.5f * WIDTH;  v1_scr.y = (1.0f - p1.y) * 0.5f * HEIGHT; v1_scr.z = p1.z;
                v2_scr.x = (p2.x + 1.0f) * 0.5f * WIDTH;  v2_scr.y = (1.0f - p2.y) * 0.5f * HEIGHT; v2_scr.z = p2.z;

                // 计算颜色
                Vec3 v0 = g_mesh.vertices[g_mesh.faces[i]];
                Vec3 v1 = g_mesh.vertices[g_mesh.faces[i+1]];
                Vec3 v2 = g_mesh.vertices[g_mesh.faces[i+2]];
                Vec3 rawNormal = (v1 - v0).cross(v2 - v0).normalize();
                Vec3 rotNormal = matRot.transformPoint(rawNormal); 
                float intensity = std::max(0.0f, rotNormal.dot(lightDir));
                float lightFactor = 0.2f + 0.8f * intensity; 
                uint8_t colorVal = (uint8_t)std::clamp(255.0f * lightFactor, 0.0f, 255.0f);

                // 调用扫描线光栅化
                scanlineRasterizeTri(v0_scr, v1_scr, v2_scr, colorVal, colorVal, colorVal);
            }
        }
        else if (g_renderMode == 3)
        {
            // --- Mode 3: Full Hierarchical Z-Buffer (BVH + HZB) ---

            // Pass 3.1: Depth Pre-Pass (Optional, but good for stability)
            // 为了演示 BVH 的剔除威力，我们可以先把 Pre-Pass 关掉，或者只用 BVH 里的三角形做 Pre-Pass
            // 但为了代码简单，这里我们假设上一帧的 HZB 或者 Pass 1 已经建立好了
            // 在这个 Demo 里，我们采取 "每帧重绘" 策略：
            // 1. 先用 BVH 快速画一遍 Depth (只写 ZBuffer)
            // 2. 构建 HZB
            // 3. 再用 BVH + HZB Check 画颜色
            
            // 为了简化代码逻辑并体现 "剔除" 效果，我们这里采用一种更激进的策略：
            // 我们直接在 Color Pass 里做剔除。
            // 注意：严格的 Greene 算法需要先通过 "Octree 前向渲染" 来更新 ZBuffer/HZB。
            // 这里我们沿用你之前的逻辑：先用普通方法填 Z，建 HZB，然后用 BVH 剔除加速 Color Pass。

            // === Step A: 粗糙的 Pre-Z Pass (为了建立 HZB) ===
            // 简单起见，我们还是用线性扫描做 Pre-Z (或者你也可以把下面的 BVH 逻辑复制一份只写 Z)
            // 实际项目中，应该用 BVH 来做 Pre-Z。
            #pragma omp parallel for
            for (int i = 0; i < (int)g_mesh.faces.size(); i += 3)
            {
                // ... (保留你原来的 Pre-Z 代码，为了填充 initial Z-Buffer) ...
                Vec3 p0 = cachedProjectedVerts[g_mesh.faces[i]];
                Vec3 p1 = cachedProjectedVerts[g_mesh.faces[i+1]];
                Vec3 p2 = cachedProjectedVerts[g_mesh.faces[i+2]];
                if (p0.z < 0 || p0.z > 1 || p1.z < 0 || p1.z > 1 || p2.z < 0 || p2.z > 1) continue;
                float sx0 = (p0.x + 1.0f) * 0.5f * WIDTH;  float sy0 = (1.0f - p0.y) * 0.5f * HEIGHT;
                float sx1 = (p1.x + 1.0f) * 0.5f * WIDTH;  float sy1 = (1.0f - p1.y) * 0.5f * HEIGHT;
                float sx2 = (p2.x + 1.0f) * 0.5f * WIDTH;  float sy2 = (1.0f - p2.y) * 0.5f * HEIGHT;
                int minX = std::max(0, (int)std::min({sx0, sx1, sx2})); int maxX = std::min((int)WIDTH - 1, (int)std::max({sx0, sx1, sx2}));
                int minY = std::max(0, (int)std::min({sy0, sy1, sy2})); int maxY = std::min((int)HEIGHT - 1, (int)std::max({sy0, sy1, sy2}));
                for (int y = minY; y <= maxY; y++) {
                    for (int x = minX; x <= maxX; x++) {
                        Vec3 P = {(float)x, (float)y, 0};
                        Vec3 bc = barycentric(P, {sx0, sy0, 0}, {sx1, sy1, 0}, {sx2, sy2, 0});
                        if (bc.x >= 0 && bc.y >= 0 && bc.z >= 0) {
                            float z = bc.x * p0.z + bc.y * p1.z + bc.z * p2.z;
                            int idx = y * WIDTH + x;
                            if (z < zBuffer[idx]) zBuffer[idx] = z;
                        }
                    }
                }
            }

            // === Step B: Build HZB ===
            buildHZB();

            // === Step C: BVH Traversal Rendering ===
            // 使用栈代替递归，进行场景遍历
            std::vector<int> nodeStack;
            nodeStack.reserve(64);
            if (!g_mesh.bvhNodes.empty()) nodeStack.push_back(0); // Push Root



            while (!nodeStack.empty()) {
                int nodeId = nodeStack.back();
                nodeStack.pop_back();
                const auto& node = g_mesh.bvhNodes[nodeId];

                // 1. 变换节点包围盒 (AABB) 到屏幕空间
                Vec3 corners[8] = {
                    {node.minB.x, node.minB.y, node.minB.z}, {node.maxB.x, node.minB.y, node.minB.z},
                    {node.minB.x, node.maxB.y, node.minB.z}, {node.maxB.x, node.maxB.y, node.minB.z},
                    {node.minB.x, node.minB.y, node.maxB.z}, {node.maxB.x, node.minB.y, node.maxB.z},
                    {node.minB.x, node.maxB.y, node.maxB.z}, {node.maxB.x, node.maxB.y, node.maxB.z}
                };

                float minX = std::numeric_limits<float>::max(), maxX = std::numeric_limits<float>::lowest();
                float minY = std::numeric_limits<float>::max(), maxY = std::numeric_limits<float>::lowest();
                float minZ = std::numeric_limits<float>::max();

                bool isSafeToCull = true; // 标记：是否可以安全地进行剔除计算

                for(int k=0; k<8; k++) {
                    float x = corners[k].x, y = corners[k].y, z = corners[k].z;
                    // 手动计算 W 分量，判断点是否在摄像机后面
                    float w = mvp.m[3][0]*x + mvp.m[3][1]*y + mvp.m[3][2]*z + mvp.m[3][3];

                    // 【核心修复】如果点在近裁剪面附近或后面，包围盒计算无效，不能剔除
                    if (w < 0.01f) {
                        isSafeToCull = false;
                        break; 
                    }

                    // 手动透视除法
                    float invW = 1.0f / w;
                    float px = (mvp.m[0][0]*x + mvp.m[0][1]*y + mvp.m[0][2]*z + mvp.m[0][3]) * invW;
                    float py = (mvp.m[1][0]*x + mvp.m[1][1]*y + mvp.m[1][2]*z + mvp.m[1][3]) * invW;
                    float pz = (mvp.m[2][0]*x + mvp.m[2][1]*y + mvp.m[2][2]*z + mvp.m[2][3]) * invW;

                    float sx = (px + 1.0f) * 0.5f * WIDTH;
                    float sy = (1.0f - py) * 0.5f * HEIGHT;

                    if(sx < minX) minX = sx; if(sx > maxX) maxX = sx;
                    if(sy < minY) minY = sy; if(sy > maxY) maxY = sy;
                    if(pz < minZ) minZ = pz;
                }
                
                // 只有当包围盒完全在摄像机前方且有效时，才进行 HZB 查询
                if (isSafeToCull) {
                    // 粗糙视锥剔除
                    if (minX > WIDTH || maxX < 0 || minY > HEIGHT || maxY < 0 || minZ > 1.0f) {
                        continue; 
                    }
                    
                    // 限制屏幕坐标，防止 queryHZB 越界
                    int qMinX = std::max(0, (int)minX);
                    int qMaxX = std::min((int)WIDTH - 1, (int)maxX);
                    int qMinY = std::max(0, (int)minY);
                    int qMaxY = std::min((int)HEIGHT - 1, (int)maxY);

                    // HZB 遮挡查询
                    if (queryHZB(qMinX, qMaxX, qMinY, qMaxY, minZ)) {
                        continue; // 被遮挡，剔除！
                    }
                }

                // 3. 如果可见 (或者无法判断是否可见)
if (node.isLeaf()) {
                    // 渲染叶子节点内的所有三角形
                    for (int i = 0; i < node.triCount; i++) {
                        int faceIdx = (node.triStart + i) * 3;
                        
                        // 【核心修复】这里必须先查 g_mesh.faces 拿到顶点索引！
                        // 之前的写法 cachedProjectedVerts[faceIdx] 是严重的越界错误
                        Vec3 p0 = cachedProjectedVerts[g_mesh.faces[faceIdx]];
                        Vec3 p1 = cachedProjectedVerts[g_mesh.faces[faceIdx+1]];
                        Vec3 p2 = cachedProjectedVerts[g_mesh.faces[faceIdx+2]];

                        if (p0.z < 0 || p0.z > 1 || p1.z < 0 || p1.z > 1 || p2.z < 0 || p2.z > 1) continue;
                        
                        float sx0 = (p0.x + 1.0f) * 0.5f * WIDTH;  float sy0 = (1.0f - p0.y) * 0.5f * HEIGHT;
                        float sx1 = (p1.x + 1.0f) * 0.5f * WIDTH;  float sy1 = (1.0f - p1.y) * 0.5f * HEIGHT;
                        float sx2 = (p2.x + 1.0f) * 0.5f * WIDTH;  float sy2 = (1.0f - p2.y) * 0.5f * HEIGHT;

                        int minX_t = std::max(0, (int)std::min({sx0, sx1, sx2})); 
                        int maxX_t = std::min((int)WIDTH - 1, (int)std::max({sx0, sx1, sx2}));
                        int minY_t = std::max(0, (int)std::min({sy0, sy1, sy2})); 
                        int maxY_t = std::min((int)HEIGHT - 1, (int)std::max({sy0, sy1, sy2}));
                        
                        // Triangle HZB Check
                        float triMinZ = std::min({p0.z, p1.z, p2.z});
                        // 增加边界检查防止 queryHZB 内部出错
                        if (minX_t <= maxX_t && minY_t <= maxY_t) {
                            if (queryHZB(minX_t, maxX_t, minY_t, maxY_t, triMinZ)) continue;
                        }

                        // 计算法线与光照
                        Vec3 v0 = g_mesh.vertices[g_mesh.faces[faceIdx]];
                        Vec3 v1 = g_mesh.vertices[g_mesh.faces[faceIdx+1]];
                        Vec3 v2 = g_mesh.vertices[g_mesh.faces[faceIdx+2]];
                        Vec3 rawNormal = (v1 - v0).cross(v2 - v0).normalize();
                        Vec3 rotNormal = matRot.transformPoint(rawNormal); 
                        float intensity = std::max(0.0f, rotNormal.dot(lightDir));
                        float lightFactor = 0.2f + 0.8f * intensity; 
                        uint8_t colorVal = (uint8_t)std::clamp(255.0f * lightFactor, 0.0f, 255.0f);

                        // 光栅化
                        for (int y = minY_t; y <= maxY_t; y++) {
                            for (int x = minX_t; x <= maxX_t; x++) {
                                Vec3 P = {(float)x, (float)y, 0};
                                Vec3 bc = barycentric(P, {sx0, sy0, 0}, {sx1, sy1, 0}, {sx2, sy2, 0});
                                if (bc.x >= 0 && bc.y >= 0 && bc.z >= 0) {
                                    float z = bc.x * p0.z + bc.y * p1.z + bc.z * p2.z;
                                    int idx = y * WIDTH + x;
                                    if (z <= zBuffer[idx] + 0.00001f) {
                                        int pIdx = idx * 4;
                                        pixels[pIdx + 0] = colorVal; 
                                        pixels[pIdx + 1] = colorVal; 
                                        pixels[pIdx + 2] = colorVal; 
                                        pixels[pIdx + 3] = 255;
                                    }
                                }
                            }
                        }
                    }
                } else {
                    // 4. 中间节点：将子节点入栈
                    if(node.left != -1) nodeStack.push_back(node.left);
                    if(node.right != -1) nodeStack.push_back(node.right);
                }
            }
            // std::cout << "Culled Nodes: " << culledNodes << " / " << totalNodesChecked << std::endl;
        }
        // --- FPS 和 UI 显示 ---
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
        int cursorX = 10; int cursorY = 10; int spacing = 8;
        if (d1 > 0) { drawDigit(d1, cursorX, cursorY); cursorX += spacing; }
        if (d1 > 0 || d2 > 0) { drawDigit(d2, cursorX, cursorY); cursorX += spacing; }
        drawDigit(d3, cursorX, cursorY);

        // --- 绘制模式指示器 (在 FPS 下方绘制 1/2/3 个红点) ---
        int modeY = cursorY + 15;
        for(int m=0; m < g_renderMode; m++) {
             for(int py=0; py<4; py++) for(int px=0; px<4; px++)
                drawPixel(10 + m*6 + px, modeY + py, 255, 50, 50);
        }
    }

    void updateTexture()
    {
        softRasterize();
        void *data;
        vkMapMemory(device, stagingBufferMemory, 0, WIDTH * HEIGHT * 4, 0, &data);
        memcpy(data, pixels.data(), pixels.size());
        vkUnmapMemory(device, stagingBufferMemory);
    }

    void initVulkan()
    {
        createInstance();
        setupDebugMessenger();
        createSurface();
        pickPhysicalDevice();
        createLogicalDevice();
        createSwapChain();
        createImageViews();
        createRenderPass();
        createDescriptorSetLayout();
        createGraphicsPipeline();
        createCommandPool();
        createStagingBuffer();
        createTextureImage();
        createTextureImageView();
        createTextureSampler();
        createFramebuffers();
        createDescriptorPool();
        createDescriptorSets();
        createCommandBuffers();
        createSyncObjects();
    }

    // Vulkan Initialization (Boilerplate)
    void createInstance() {
        VkApplicationInfo appInfo{VK_STRUCTURE_TYPE_APPLICATION_INFO};
        appInfo.pApplicationName = "Soft Rasterizer";
        appInfo.apiVersion = VK_API_VERSION_1_0;
        VkInstanceCreateInfo createInfo{VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO};
        createInfo.pApplicationInfo = &appInfo;
        uint32_t glfwExtensionCount = 0;
        const char **glfwExtensions = glfwGetRequiredInstanceExtensions(&glfwExtensionCount);
        std::vector<const char *> extensions(glfwExtensions, glfwExtensions + glfwExtensionCount);
        if (enableValidationLayers) extensions.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
        createInfo.enabledExtensionCount = static_cast<uint32_t>(extensions.size());
        createInfo.ppEnabledExtensionNames = extensions.data();
        if (enableValidationLayers) {
            createInfo.enabledLayerCount = static_cast<uint32_t>(validationLayers.size());
            createInfo.ppEnabledLayerNames = validationLayers.data();
        } else {
            createInfo.enabledLayerCount = 0;
        }
        if (vkCreateInstance(&createInfo, nullptr, &instance) != VK_SUCCESS)
            throw std::runtime_error("failed to create instance!");
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
    void createSurface() {
        if (glfwCreateWindowSurface(instance, window, nullptr, &surface) != VK_SUCCESS)
            throw std::runtime_error("failed to create window surface!");
    }
    void pickPhysicalDevice() {
        uint32_t deviceCount = 0;
        vkEnumeratePhysicalDevices(instance, &deviceCount, nullptr);
        if (deviceCount == 0) throw std::runtime_error("failed to find GPUs with Vulkan support!");
        std::vector<VkPhysicalDevice> devices(deviceCount);
        vkEnumeratePhysicalDevices(instance, &deviceCount, devices.data());
        physicalDevice = devices[0];
    }
    void createLogicalDevice() {
        float queuePriority = 1.0f;
        VkDeviceQueueCreateInfo queueCreateInfo{VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO};
        queueCreateInfo.queueFamilyIndex = 0; queueCreateInfo.queueCount = 1; queueCreateInfo.pQueuePriorities = &queuePriority;
        VkDeviceCreateInfo createInfo{VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO};
        createInfo.queueCreateInfoCount = 1; createInfo.pQueueCreateInfos = &queueCreateInfo;
        createInfo.enabledExtensionCount = static_cast<uint32_t>(deviceExtensions.size());
        createInfo.ppEnabledExtensionNames = deviceExtensions.data();
        if (enableValidationLayers) {
            createInfo.enabledLayerCount = static_cast<uint32_t>(validationLayers.size());
            createInfo.ppEnabledLayerNames = validationLayers.data();
        }
        if (vkCreateDevice(physicalDevice, &createInfo, nullptr, &device) != VK_SUCCESS)
            throw std::runtime_error("failed to create logical device!");
        vkGetDeviceQueue(device, 0, 0, &graphicsQueue);
        vkGetDeviceQueue(device, 0, 0, &presentQueue);
    }
    void createSwapChain() {
        VkSurfaceCapabilitiesKHR capabilities;
        vkGetPhysicalDeviceSurfaceCapabilitiesKHR(physicalDevice, surface, &capabilities);
        VkSurfaceFormatKHR surfaceFormat = {VK_FORMAT_B8G8R8A8_SRGB, VK_COLOR_SPACE_SRGB_NONLINEAR_KHR};
        VkPresentModeKHR presentMode = VK_PRESENT_MODE_IMMEDIATE_KHR;
        VkExtent2D extent = {WIDTH, HEIGHT};
        uint32_t imageCount = capabilities.minImageCount + 1;
        if (capabilities.maxImageCount > 0 && imageCount > capabilities.maxImageCount) imageCount = capabilities.maxImageCount;
        VkSwapchainCreateInfoKHR createInfo{VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR};
        createInfo.surface = surface; createInfo.minImageCount = imageCount; createInfo.imageFormat = surfaceFormat.format;
        createInfo.imageColorSpace = surfaceFormat.colorSpace; createInfo.imageExtent = extent; createInfo.imageArrayLayers = 1;
        createInfo.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT; createInfo.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
        createInfo.preTransform = capabilities.currentTransform; createInfo.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
        createInfo.presentMode = presentMode; createInfo.clipped = VK_TRUE; createInfo.oldSwapchain = VK_NULL_HANDLE;
        if (vkCreateSwapchainKHR(device, &createInfo, nullptr, &swapChain) != VK_SUCCESS) throw std::runtime_error("failed to create swap chain!");
        vkGetSwapchainImagesKHR(device, swapChain, &imageCount, nullptr);
        swapChainImages.resize(imageCount);
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
        VkAttachmentDescription colorAttachment{};
        colorAttachment.format = swapChainImageFormat; colorAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
        colorAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR; colorAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        colorAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE; colorAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        colorAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED; colorAttachment.finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
        VkAttachmentReference colorAttachmentRef{};
        colorAttachmentRef.attachment = 0; colorAttachmentRef.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        VkSubpassDescription subpass{};
        subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS; subpass.colorAttachmentCount = 1; subpass.pColorAttachments = &colorAttachmentRef;
        VkRenderPassCreateInfo renderPassInfo{VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO};
        renderPassInfo.attachmentCount = 1; renderPassInfo.pAttachments = &colorAttachment;
        renderPassInfo.subpassCount = 1; renderPassInfo.pSubpasses = &subpass;
        if (vkCreateRenderPass(device, &renderPassInfo, nullptr, &renderPass) != VK_SUCCESS) throw std::runtime_error("failed to create render pass!");
    }
    void createDescriptorSetLayout() {
        VkDescriptorSetLayoutBinding samplerLayoutBinding{};
        samplerLayoutBinding.binding = 0; samplerLayoutBinding.descriptorCount = 1;
        samplerLayoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        samplerLayoutBinding.pImmutableSamplers = nullptr; samplerLayoutBinding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
        VkDescriptorSetLayoutCreateInfo layoutInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
        layoutInfo.bindingCount = 1; layoutInfo.pBindings = &samplerLayoutBinding;
        if (vkCreateDescriptorSetLayout(device, &layoutInfo, nullptr, &descriptorSetLayout) != VK_SUCCESS) throw std::runtime_error("failed to create descriptor set layout!");
    }
    void createGraphicsPipeline() {
        auto vertShaderCode = readFile("vert.spv");
        auto fragShaderCode = readFile("frag.spv");
        VkShaderModule vertShaderModule = createShaderModule(vertShaderCode);
        VkShaderModule fragShaderModule = createShaderModule(fragShaderCode);
        VkPipelineShaderStageCreateInfo vertShaderStageInfo{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
        vertShaderStageInfo.stage = VK_SHADER_STAGE_VERTEX_BIT; vertShaderStageInfo.module = vertShaderModule; vertShaderStageInfo.pName = "main";
        VkPipelineShaderStageCreateInfo fragShaderStageInfo{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
        fragShaderStageInfo.stage = VK_SHADER_STAGE_FRAGMENT_BIT; fragShaderStageInfo.module = fragShaderModule; fragShaderStageInfo.pName = "main";
        VkPipelineShaderStageCreateInfo shaderStages[] = {vertShaderStageInfo, fragShaderStageInfo};
        VkPipelineVertexInputStateCreateInfo vertexInputInfo{VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO};
        VkPipelineInputAssemblyStateCreateInfo inputAssembly{VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO};
        inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
        VkViewport viewport{}; viewport.width = (float)WIDTH; viewport.height = (float)HEIGHT; viewport.maxDepth = 1.0f;
        VkRect2D scissor{}; scissor.extent = {WIDTH, HEIGHT};
        VkPipelineViewportStateCreateInfo viewportState{VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO};
        viewportState.viewportCount = 1; viewportState.pViewports = &viewport;
        viewportState.scissorCount = 1; viewportState.pScissors = &scissor;
        VkPipelineRasterizationStateCreateInfo rasterizer{VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO};
        rasterizer.depthClampEnable = VK_FALSE; rasterizer.rasterizerDiscardEnable = VK_FALSE;
        rasterizer.polygonMode = VK_POLYGON_MODE_FILL; rasterizer.lineWidth = 1.0f;
        rasterizer.cullMode = VK_CULL_MODE_NONE; rasterizer.frontFace = VK_FRONT_FACE_CLOCKWISE;
        VkPipelineMultisampleStateCreateInfo multisampling{VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO};
        multisampling.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
        VkPipelineColorBlendAttachmentState colorBlendAttachment{};
        colorBlendAttachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
        colorBlendAttachment.blendEnable = VK_FALSE;
        VkPipelineColorBlendStateCreateInfo colorBlending{VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO};
        colorBlending.attachmentCount = 1; colorBlending.pAttachments = &colorBlendAttachment;
        VkPipelineDynamicStateCreateInfo dynamicState{VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO};
        std::vector<VkDynamicState> dynamicStates = {VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
        dynamicState.dynamicStateCount = static_cast<uint32_t>(dynamicStates.size()); dynamicState.pDynamicStates = dynamicStates.data();
        VkPipelineLayoutCreateInfo pipelineLayoutInfo{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
        pipelineLayoutInfo.setLayoutCount = 1; pipelineLayoutInfo.pSetLayouts = &descriptorSetLayout;
        if (vkCreatePipelineLayout(device, &pipelineLayoutInfo, nullptr, &pipelineLayout) != VK_SUCCESS) throw std::runtime_error("failed to create pipeline layout!");
        VkGraphicsPipelineCreateInfo pipelineInfo{VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO};
        pipelineInfo.stageCount = 2; pipelineInfo.pStages = shaderStages; pipelineInfo.pVertexInputState = &vertexInputInfo;
        pipelineInfo.pInputAssemblyState = &inputAssembly; pipelineInfo.pViewportState = &viewportState;
        pipelineInfo.pRasterizationState = &rasterizer; pipelineInfo.pMultisampleState = &multisampling;
        pipelineInfo.pColorBlendState = &colorBlending; pipelineInfo.pDynamicState = &dynamicState;
        pipelineInfo.layout = pipelineLayout; pipelineInfo.renderPass = renderPass; pipelineInfo.subpass = 0;
        if (vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &graphicsPipeline) != VK_SUCCESS) throw std::runtime_error("failed to create graphics pipeline!");
        vkDestroyShaderModule(device, fragShaderModule, nullptr); vkDestroyShaderModule(device, vertShaderModule, nullptr);
    }
    void createFramebuffers() {
        swapChainFramebuffers.resize(swapChainImageViews.size());
        for (size_t i = 0; i < swapChainImageViews.size(); i++) {
            VkImageView attachments[] = {swapChainImageViews[i]};
            VkFramebufferCreateInfo framebufferInfo{VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO};
            framebufferInfo.renderPass = renderPass; framebufferInfo.attachmentCount = 1;
            framebufferInfo.pAttachments = attachments; framebufferInfo.width = swapChainExtent.width; framebufferInfo.height = swapChainExtent.height;
            framebufferInfo.layers = 1;
            if (vkCreateFramebuffer(device, &framebufferInfo, nullptr, &swapChainFramebuffers[i]) != VK_SUCCESS) throw std::runtime_error("failed to create framebuffer!");
        }
    }
    void createCommandPool() {
        VkCommandPoolCreateInfo poolInfo{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
        poolInfo.queueFamilyIndex = 0; poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
        if (vkCreateCommandPool(device, &poolInfo, nullptr, &commandPool) != VK_SUCCESS) throw std::runtime_error("failed to create command pool!");
    }
    uint32_t findMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags properties) {
        VkPhysicalDeviceMemoryProperties memProperties; vkGetPhysicalDeviceMemoryProperties(physicalDevice, &memProperties);
        for (uint32_t i = 0; i < memProperties.memoryTypeCount; i++) {
            if ((typeFilter & (1 << i)) && (memProperties.memoryTypes[i].propertyFlags & properties) == properties) return i;
        } throw std::runtime_error("failed to find suitable memory type!");
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
        imageInfo.imageType = VK_IMAGE_TYPE_2D; imageInfo.extent.width = WIDTH; imageInfo.extent.height = HEIGHT;
        imageInfo.extent.depth = 1; imageInfo.mipLevels = 1; imageInfo.arrayLayers = 1;
        imageInfo.format = VK_FORMAT_R8G8B8A8_SRGB; imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
        imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED; imageInfo.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
        imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;
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
        VkShaderModuleCreateInfo createInfo{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
        createInfo.codeSize = code.size(); createInfo.pCode = reinterpret_cast<const uint32_t *>(code.data());
        VkShaderModule shaderModule;
        if (vkCreateShaderModule(device, &createInfo, nullptr, &shaderModule) != VK_SUCCESS) throw std::runtime_error("failed to create shader module!");
        return shaderModule;
    }
    void mainLoop() {
        while (!glfwWindowShouldClose(window)) {
            glfwPollEvents(); processInput(); drawFrame();
        } vkDeviceWaitIdle(device);
    }
    void drawFrame() {
        vkWaitForFences(device, 1, &inFlightFences[currentFrame], VK_TRUE, UINT64_MAX);
        uint32_t imageIndex;
        VkResult result = vkAcquireNextImageKHR(device, swapChain, UINT64_MAX, imageAvailableSemaphores[currentFrame], VK_NULL_HANDLE, &imageIndex);
        if (result == VK_ERROR_OUT_OF_DATE_KHR) { recreateSwapChain(); return; }
        else if (result != VK_SUCCESS && result != VK_SUBOPTIMAL_KHR) throw std::runtime_error("failed to acquire swap chain image!");
        vkResetFences(device, 1, &inFlightFences[currentFrame]);

        updateTexture();

        vkResetCommandBuffer(commandBuffers[currentFrame], 0);
        VkCommandBuffer cb = commandBuffers[currentFrame];
        VkCommandBufferBeginInfo beginInfo{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
        vkBeginCommandBuffer(cb, &beginInfo);
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
        renderPassInfo.renderArea.extent = swapChainExtent;
        VkClearValue clearColor = {{{0.0f, 0.0f, 0.0f, 1.0f}}};
        renderPassInfo.clearValueCount = 1; renderPassInfo.pClearValues = &clearColor;
        vkCmdBeginRenderPass(cb, &renderPassInfo, VK_SUBPASS_CONTENTS_INLINE);
        vkCmdBindPipeline(cb, VK_PIPELINE_BIND_POINT_GRAPHICS, graphicsPipeline);
        VkViewport viewport{}; viewport.width = (float)swapChainExtent.width; viewport.height = (float)swapChainExtent.height; viewport.maxDepth = 1.0f;
        vkCmdSetViewport(cb, 0, 1, &viewport);
        VkRect2D scissor{}; scissor.extent = swapChainExtent;
        vkCmdSetScissor(cb, 0, 1, &scissor);
        vkCmdBindDescriptorSets(cb, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 0, 1, &descriptorSets[currentFrame], 0, nullptr);
        vkCmdDraw(cb, 3, 1, 0, 0);
        vkCmdEndRenderPass(cb); vkEndCommandBuffer(cb);
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
        currentFrame = (currentFrame + 1) % MAX_FRAMES_IN_FLIGHT;
        vkQueueWaitIdle(presentQueue);

        // 【新增】Benchmark 逻辑
        if (benchmarkMode) {
            benchmarkFrameCount++;
            if (benchmarkFrameCount >= TOTAL_TEST_FRAMES) {
                auto endTime = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double, std::milli> elapsed = endTime - startTime;
                
                double totalTimeMs = elapsed.count();
                double avgTimeMs = totalTimeMs / TOTAL_TEST_FRAMES;
                double avgFPS = 1000.0 / avgTimeMs;

                // 打印特殊格式的 Tag 方便脚本抓取
                std::cout << "BENCHMARK_RESULT: Mode=" << g_renderMode 
                          << " FPS=" << avgFPS 
                          << " Time=" << avgTimeMs << "ms" << std::endl;
                
                glfwSetWindowShouldClose(window, true); // 触发退出
            }
        }
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

    // 参数解析
    if (argc > 1) {
        objFile = argv[1];
    }
    if (argc > 2) {
        try {
            mode = std::stoi(argv[2]);
            if(mode < 1 || mode > 3) mode = 1;
        } catch (...) {
            mode = 1;
        }
    }

    if (!objFile) {
        std::cout << "Usage: ./VulkanApp.exe model.obj [mode: 1,2,3]" << std::endl;
        std::cout << "No model specified, loading default triangle." << std::endl;
    }

    try {
        app.run(objFile, mode);
    } catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}