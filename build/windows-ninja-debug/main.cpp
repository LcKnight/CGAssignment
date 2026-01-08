#include <vulkan/vulkan.h>
#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

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

// -----------------------------------------------------------
// 简单的 3D 数学库 (用于软光栅)
// -----------------------------------------------------------
struct Vec3 {
    float x, y, z;
    Vec3 operator+(const Vec3& v) const { return {x + v.x, y + v.y, z + v.z}; }
    Vec3 operator-(const Vec3& v) const { return {x - v.x, y - v.y, z - v.z}; }
    Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
    float dot(const Vec3& v) const { return x * v.x + y * v.y + z * v.z; }
    Vec3 cross(const Vec3& v) const { return {y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x}; }
    Vec3 normalize() const {
        float len = std::sqrt(x*x + y*y + z*z);
        return (len > 0) ? (*this) * (1.0f / len) : Vec3{0,0,0};
    }
};

struct Mat4 {
    float m[4][4];
    static Mat4 identity() {
        Mat4 res = {0};
        for(int i=0;i<4;i++) res.m[i][i] = 1.0f;
        return res;
    }
    static Mat4 perspective(float fov, float aspect, float znear, float zfar) {
        Mat4 res = {0};
        float tanHalfFov = std::tan(fov / 2.0f);
        res.m[0][0] = 1.0f / (aspect * tanHalfFov);
        res.m[1][1] = 1.0f / tanHalfFov; // Vulkan Y is down, but we handle in viewport
        res.m[2][2] = -(zfar + znear) / (zfar - znear); // Z is -1 to 1 in GL/Vulkan clips
        res.m[2][3] = -(2.0f * zfar * znear) / (zfar - znear);
        res.m[3][2] = -1.0f;
        return res;
    }
    // 简单的 LookAt (针对 cameraX/Y/Z)
    static Mat4 translate(float x, float y, float z) {
        Mat4 res = identity();
        res.m[0][3] = x; res.m[1][3] = y; res.m[2][3] = z;
        return res;
    }
    // 简单的旋转 (绕Y轴)
    static Mat4 rotateY(float angle) {
        Mat4 res = identity();
        float c = std::cos(angle), s = std::sin(angle);
        res.m[0][0] = c; res.m[0][2] = s;
        res.m[2][0] = -s; res.m[2][2] = c;
        return res;
    }
    // 简单的旋转 (绕X轴)
    static Mat4 rotateX(float angle) {
        Mat4 res = identity();
        float c = std::cos(angle), s = std::sin(angle);
        res.m[1][1] = c; res.m[1][2] = -s;
        res.m[2][1] = s; res.m[2][2] = c;
        return res;
    }
    
    Vec3 transformPoint(const Vec3& v) const {
        float x = m[0][0]*v.x + m[0][1]*v.y + m[0][2]*v.z + m[0][3];
        float y = m[1][0]*v.x + m[1][1]*v.y + m[1][2]*v.z + m[1][3];
        float z = m[2][0]*v.x + m[2][1]*v.y + m[2][2]*v.z + m[2][3];
        float w = m[3][0]*v.x + m[3][1]*v.y + m[3][2]*v.z + m[3][3];
        if (w != 0.0f) { x/=w; y/=w; z/=w; }
        return {x, y, z};
    }
    
    Mat4 operator*(const Mat4& r) const {
        Mat4 res = {0};
        for(int i=0;i<4;i++) for(int j=0;j<4;j++) for(int k=0;k<4;k++)
            res.m[i][j] += m[i][k] * r.m[k][j];
        return res;
    }
};

// -----------------------------------------------------------
// Mesh 数据结构
// -----------------------------------------------------------
struct Mesh {
    std::vector<Vec3> vertices;
    std::vector<int> faces; // 每3个int为一个三角形的顶点索引
    
    // 简单的 OBJ 加载器
    static Mesh loadObj(const std::string& filename) {
        Mesh mesh;
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Failed to open OBJ file: " << filename << ". Loading default triangle." << std::endl;
            mesh.vertices = {{-1, 1, 0}, {1, 1, 0}, {0, -1, 0}};
            mesh.faces = {0, 1, 2};
            return mesh;
        }
        
        std::string line;
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string prefix;
            ss >> prefix;
            if (prefix == "v") {
                Vec3 v; ss >> v.x >> v.y >> v.z;
                mesh.vertices.push_back(v);
            } else if (prefix == "f") {
                // 仅支持 "f v1 v2 v3" 或 "f v1/vt/vn ..." 格式（只取顶点索引）
                std::string segment;
                for (int i = 0; i < 3; i++) {
                    ss >> segment;
                    // 处理 v/vt/vn 格式，只取第一个斜杠前的数字
                    size_t slashPos = segment.find('/');
                    int idx = std::stoi(segment.substr(0, slashPos));
                    mesh.faces.push_back(idx - 1); // OBJ 索引从1开始
                }
            }
        }
        std::cout << "Loaded mesh: " << mesh.vertices.size() << " vertices, " << mesh.faces.size()/3 << " triangles." << std::endl;
        return mesh;
    }
};

// -----------------------------------------------------------
// 全局设置
// -----------------------------------------------------------
const uint32_t WIDTH = 800;
const uint32_t HEIGHT = 600;
const int MAX_FRAMES_IN_FLIGHT = 2;

// 输入状态
bool keys[1024] = { false };
float cameraX = 0.0f, cameraY = 0.0f, cameraZ = 3.0f;
float lastX = 400, lastY = 300;
float yaw = -90.0f, pitch = 0.0f;
bool firstMouse = true;
bool rightMouseDown = false;

// 软光栅 Framebuffer
std::vector<unsigned char> pixels(WIDTH * HEIGHT * 4);
std::vector<float> zBuffer(WIDTH * HEIGHT); // 深度缓冲

// 模型
Mesh g_mesh;

const std::vector<const char*> validationLayers = { "VK_LAYER_KHRONOS_validation" };
const std::vector<const char*> deviceExtensions = { VK_KHR_SWAPCHAIN_EXTENSION_NAME };

#ifdef NDEBUG
const bool enableValidationLayers = false;
#else
const bool enableValidationLayers = true;
#endif

// -----------------------------------------------------------
// 辅助函数：读取文件 (用于读取 SPIR-V)
// -----------------------------------------------------------
static std::vector<char> readFile(const std::string& filename) {
    std::ifstream file(filename, std::ios::ate | std::ios::binary);
    if (!file.is_open()) throw std::runtime_error("failed to open file: " + filename);
    size_t fileSize = (size_t) file.tellg();
    std::vector<char> buffer(fileSize);
    file.seekg(0);
    file.read(buffer.data(), fileSize);
    file.close();
    return buffer;
}

// -----------------------------------------------------------
// 应用程序类
// -----------------------------------------------------------
class HelloVulkanApplication {
public:
    void run(const char* objFilename) {
        if(objFilename) g_mesh = Mesh::loadObj(objFilename);
        else g_mesh = Mesh::loadObj("dummy.obj"); // 会加载默认三角形
        
        initWindow();
        initVulkan();
        mainLoop();
        cleanup();
    }

private:
    GLFWwindow* window;
    VkInstance instance;
    VkDebugUtilsMessengerEXT debugMessenger;
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

    // --- 输入回调 ---
    static void framebufferResizeCallback(GLFWwindow* window, int width, int height) {
        auto app = reinterpret_cast<HelloVulkanApplication*>(glfwGetWindowUserPointer(window));
        app->framebufferResized = true;
    }
    static void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
        if (key >= 0 && key < 1024) keys[key] = (action != GLFW_RELEASE);
    }
    static void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
        if (button == GLFW_MOUSE_BUTTON_RIGHT) rightMouseDown = (action == GLFW_PRESS);
    }
    static void cursorPosCallback(GLFWwindow* window, double xposIn, double yposIn) {
        if (!rightMouseDown) return;
        float xpos = static_cast<float>(xposIn);
        float ypos = static_cast<float>(yposIn);
        if (firstMouse) { lastX = xpos; lastY = ypos; firstMouse = false; }
        float xoffset = (xpos - lastX) * 0.1f;
        float yoffset = (lastY - ypos) * 0.1f;
        lastX = xpos; lastY = ypos;
        yaw += xoffset; pitch += yoffset;
        if (pitch > 89.0f) pitch = 89.0f;
        if (pitch < -89.0f) pitch = -89.0f;
    }

    void initWindow() {
        glfwInit();
        glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
        glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);
        window = glfwCreateWindow(WIDTH, HEIGHT, "Soft Rasterizer (Obj Viewer)", nullptr, nullptr);
        glfwSetWindowUserPointer(window, this);
        glfwSetFramebufferSizeCallback(window, framebufferResizeCallback);
        glfwSetKeyCallback(window, keyCallback);
        glfwSetCursorPosCallback(window, cursorPosCallback);
        glfwSetMouseButtonCallback(window, mouseButtonCallback);
    }

    void initVulkan() {
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
        
        // --- 资源创建 (软光栅所需) ---
        createStagingBuffer();
        createTextureImage();
        createTextureImageView();
        createTextureSampler();
        
        createFramebuffers(); // 必须在 RenderPass 和 ImageView 创建之后
        
        createDescriptorPool();
        createDescriptorSets();
        createCommandBuffers();
        createSyncObjects();
    }
    void processInput() {
        float speed = 0.05f;
        // 简化的FPS相机移动
        float yawRad = yaw * 3.14159f / 180.0f;
        Vec3 forward = { std::cos(yawRad), 0.0f, std::sin(yawRad) };
        Vec3 right = { forward.z, 0.0f, -forward.x }; // 简单的垂直右向量
        
        if (keys[GLFW_KEY_W]) { cameraX += forward.x * speed; cameraZ += forward.z * speed; }
        if (keys[GLFW_KEY_S]) { cameraX -= forward.x * speed; cameraZ -= forward.z * speed; }
        if (keys[GLFW_KEY_A]) { cameraX -= right.x * speed; cameraZ -= right.z * speed; }
        if (keys[GLFW_KEY_D]) { cameraX += right.x * speed; cameraZ += right.z * speed; }
        if (keys[GLFW_KEY_Q]) cameraY -= speed;
        if (keys[GLFW_KEY_E]) cameraY += speed;
        if (keys[GLFW_KEY_ESCAPE]) glfwSetWindowShouldClose(window, true);
    }

    // --- 软光栅核心 ---
    void setPixel(int x, int y, float z, uint8_t r, uint8_t g, uint8_t b) {
        if (x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT) return;
        int idx = y * WIDTH + x;
        // 深度测试 (注意：这里使用 > 测试，假设 Z 指向屏幕外，越小越远，具体取决于投影矩阵)
        // 简单透视除法后 Z 范围 -1 到 1。这里假设越小越靠近相机 (NDC习惯) 或 越大越远
        // 我们用 1/w 作为深度缓冲比较好，或者直接用 NDC z
        if (z < zBuffer[idx]) {
            zBuffer[idx] = z;
            int pIdx = idx * 4;
            pixels[pIdx + 0] = r;
            pixels[pIdx + 1] = g;
            pixels[pIdx + 2] = b;
            pixels[pIdx + 3] = 255;
        }
    }

    // 计算重心坐标
    Vec3 barycentric(Vec3 p, Vec3 a, Vec3 b, Vec3 c) {
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

    void softRasterize() {
        // 1. 清屏
        std::fill(pixels.begin(), pixels.end(), 0);
        std::fill(zBuffer.begin(), zBuffer.end(), 1.0f); // 深度初始化为 1.0 (最远)

        // 2. 矩阵构建
        Mat4 model = Mat4::translate(0, -1.0f, 0); // 将模型向下移动一点
        // 简单的 View 矩阵 (Camera 逆变换)
        Mat4 viewTranslate = Mat4::translate(-cameraX, -cameraY, -cameraZ);
        Mat4 viewRotateX = Mat4::rotateX(-pitch * 3.14159f / 180.0f);
        Mat4 viewRotateY = Mat4::rotateY(-(yaw + 90.0f) * 3.14159f / 180.0f);
        Mat4 view = viewRotateX * viewRotateY * viewTranslate;
        
        Mat4 proj = Mat4::perspective(45.0f * 3.14159f / 180.0f, (float)WIDTH/HEIGHT, 0.1f, 100.0f);
        Mat4 mvp = proj * view * model; // MVP

        // 光照方向
        Vec3 lightDir = {0.5f, 1.0f, 0.3f};
        lightDir = lightDir.normalize();

        // 3. 遍历三角形
        for (size_t i = 0; i < g_mesh.faces.size(); i += 3) {
            Vec3 v0 = g_mesh.vertices[g_mesh.faces[i]];
            Vec3 v1 = g_mesh.vertices[g_mesh.faces[i+1]];
            Vec3 v2 = g_mesh.vertices[g_mesh.faces[i+2]];

            // 变换到裁剪空间
            Vec3 p0 = mvp.transformPoint(v0);
            Vec3 p1 = mvp.transformPoint(v1);
            Vec3 p2 = mvp.transformPoint(v2);

            // 简单的背面剔除 (Back-face Culling) 和 裁剪
            if (p0.z < 0 || p0.z > 1 || p1.z < 0 || p1.z > 1 || p2.z < 0 || p2.z > 1) continue;

            // 视口变换 (NDC -> Screen)
            float sx0 = (p0.x + 1.0f) * 0.5f * WIDTH;  float sy0 = (1.0f - p0.y) * 0.5f * HEIGHT;
            float sx1 = (p1.x + 1.0f) * 0.5f * WIDTH;  float sy1 = (1.0f - p1.y) * 0.5f * HEIGHT;
            float sx2 = (p2.x + 1.0f) * 0.5f * WIDTH;  float sy2 = (1.0f - p2.y) * 0.5f * HEIGHT;

            // 简单的 Flat Shading 光照 (利用屏幕空间法线粗略模拟，正确应在世界空间算)
            // 为了演示简单，直接用模型空间面法线计算光照
            Vec3 normal = (v1 - v0).cross(v2 - v0).normalize();
            float intensity = std::max(0.0f, normal.dot(lightDir));
            uint8_t c = (uint8_t)(intensity * 255);
            if (c < 50) c = 50; // 环境光

            // 包围盒
            int minX = std::max(0, (int)std::min({sx0, sx1, sx2}));
            int maxX = std::min((int)WIDTH - 1, (int)std::max({sx0, sx1, sx2}));
            int minY = std::max(0, (int)std::min({sy0, sy1, sy2}));
            int maxY = std::min((int)HEIGHT - 1, (int)std::max({sy0, sy1, sy2}));

            for (int y = minY; y <= maxY; y++) {
                for (int x = minX; x <= maxX; x++) {
                    Vec3 P = {(float)x, (float)y, 0};
                    Vec3 A = {sx0, sy0, 0}; Vec3 B = {sx1, sy1, 0}; Vec3 C = {sx2, sy2, 0};
                    
                    Vec3 bc = barycentric(P, A, B, C);
                    if (bc.x >= 0 && bc.y >= 0 && bc.z >= 0) {
                        // 插值深度 Z
                        float z = bc.x * p0.z + bc.y * p1.z + bc.z * p2.z;
                        setPixel(x, y, z, c, c, c); // 灰色渲染
                    }
                }
            }
        }
    }

    void updateTexture() {
        softRasterize();
        void* data;
        vkMapMemory(device, stagingBufferMemory, 0, WIDTH * HEIGHT * 4, 0, &data);
        memcpy(data, pixels.data(), pixels.size());
        vkUnmapMemory(device, stagingBufferMemory);
    }

    // -----------------------------------------------------------
    // Vulkan 初始化实现 (填充部分)
    // -----------------------------------------------------------
    void createInstance() {
        VkApplicationInfo appInfo{VK_STRUCTURE_TYPE_APPLICATION_INFO};
        appInfo.pApplicationName = "Soft Rasterizer";
        appInfo.apiVersion = VK_API_VERSION_1_0;

        VkInstanceCreateInfo createInfo{VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO};
        createInfo.pApplicationInfo = &appInfo;

        uint32_t glfwExtensionCount = 0;
        const char** glfwExtensions = glfwGetRequiredInstanceExtensions(&glfwExtensionCount);
        std::vector<const char*> extensions(glfwExtensions, glfwExtensions + glfwExtensionCount);
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

    void setupDebugMessenger() { if (!enableValidationLayers) return; /* 简化：省略具体 debug 回调实现，仅声明 */ }

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
        physicalDevice = devices[0]; // 简单选择第一个
    }

    void createLogicalDevice() {
        // 简化：假设 Graphics 和 Present 是同一个队列 (大部分显卡如此)
        float queuePriority = 1.0f;
        VkDeviceQueueCreateInfo queueCreateInfo{VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO};
        queueCreateInfo.queueFamilyIndex = 0; // 假设 0 号队列族支持所有
        queueCreateInfo.queueCount = 1;
        queueCreateInfo.pQueuePriorities = &queuePriority;

        VkDeviceCreateInfo createInfo{VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO};
        createInfo.queueCreateInfoCount = 1;
        createInfo.pQueueCreateInfos = &queueCreateInfo;
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
        VkPresentModeKHR presentMode = VK_PRESENT_MODE_FIFO_KHR;
        VkExtent2D extent = {WIDTH, HEIGHT};

        uint32_t imageCount = capabilities.minImageCount + 1;
        if (capabilities.maxImageCount > 0 && imageCount > capabilities.maxImageCount)
            imageCount = capabilities.maxImageCount;

        VkSwapchainCreateInfoKHR createInfo{VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR};
        createInfo.surface = surface;
        createInfo.minImageCount = imageCount;
        createInfo.imageFormat = surfaceFormat.format;
        createInfo.imageColorSpace = surfaceFormat.colorSpace;
        createInfo.imageExtent = extent;
        createInfo.imageArrayLayers = 1;
        createInfo.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
        createInfo.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
        createInfo.preTransform = capabilities.currentTransform;
        createInfo.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
        createInfo.presentMode = presentMode;
        createInfo.clipped = VK_TRUE;
        createInfo.oldSwapchain = VK_NULL_HANDLE;

        if (vkCreateSwapchainKHR(device, &createInfo, nullptr, &swapChain) != VK_SUCCESS)
            throw std::runtime_error("failed to create swap chain!");

        vkGetSwapchainImagesKHR(device, swapChain, &imageCount, nullptr);
        swapChainImages.resize(imageCount);
        vkGetSwapchainImagesKHR(device, swapChain, &imageCount, swapChainImages.data());
        swapChainImageFormat = surfaceFormat.format;
        swapChainExtent = extent;
    }

    void createImageViews() {
        swapChainImageViews.resize(swapChainImages.size());
        for (size_t i = 0; i < swapChainImages.size(); i++) {
            VkImageViewCreateInfo createInfo{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
            createInfo.image = swapChainImages[i];
            createInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
            createInfo.format = swapChainImageFormat;
            createInfo.components.r = VK_COMPONENT_SWIZZLE_IDENTITY;
            createInfo.components.g = VK_COMPONENT_SWIZZLE_IDENTITY;
            createInfo.components.b = VK_COMPONENT_SWIZZLE_IDENTITY;
            createInfo.components.a = VK_COMPONENT_SWIZZLE_IDENTITY;
            createInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            createInfo.subresourceRange.baseMipLevel = 0;
            createInfo.subresourceRange.levelCount = 1;
            createInfo.subresourceRange.baseArrayLayer = 0;
            createInfo.subresourceRange.layerCount = 1;
            if (vkCreateImageView(device, &createInfo, nullptr, &swapChainImageViews[i]) != VK_SUCCESS)
                throw std::runtime_error("failed to create image views!");
        }
    }

    void createRenderPass() {
        VkAttachmentDescription colorAttachment{};
        colorAttachment.format = swapChainImageFormat;
        colorAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
        colorAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        colorAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        colorAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        colorAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        colorAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        colorAttachment.finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

        VkAttachmentReference colorAttachmentRef{};
        colorAttachmentRef.attachment = 0;
        colorAttachmentRef.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

        VkSubpassDescription subpass{};
        subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
        subpass.colorAttachmentCount = 1;
        subpass.pColorAttachments = &colorAttachmentRef;

        VkRenderPassCreateInfo renderPassInfo{VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO};
        renderPassInfo.attachmentCount = 1;
        renderPassInfo.pAttachments = &colorAttachment;
        renderPassInfo.subpassCount = 1;
        renderPassInfo.pSubpasses = &subpass;

        if (vkCreateRenderPass(device, &renderPassInfo, nullptr, &renderPass) != VK_SUCCESS)
            throw std::runtime_error("failed to create render pass!");
    }

    void createDescriptorSetLayout() {
        VkDescriptorSetLayoutBinding samplerLayoutBinding{};
        samplerLayoutBinding.binding = 0;
        samplerLayoutBinding.descriptorCount = 1;
        samplerLayoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        samplerLayoutBinding.pImmutableSamplers = nullptr;
        samplerLayoutBinding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

        VkDescriptorSetLayoutCreateInfo layoutInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
        layoutInfo.bindingCount = 1;
        layoutInfo.pBindings = &samplerLayoutBinding;

        if (vkCreateDescriptorSetLayout(device, &layoutInfo, nullptr, &descriptorSetLayout) != VK_SUCCESS)
            throw std::runtime_error("failed to create descriptor set layout!");
    }

    void createGraphicsPipeline() {
        auto vertShaderCode = readFile("vert.spv");
        auto fragShaderCode = readFile("frag.spv");

        VkShaderModule vertShaderModule = createShaderModule(vertShaderCode);
        VkShaderModule fragShaderModule = createShaderModule(fragShaderCode);

        VkPipelineShaderStageCreateInfo vertShaderStageInfo{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
        vertShaderStageInfo.stage = VK_SHADER_STAGE_VERTEX_BIT;
        vertShaderStageInfo.module = vertShaderModule;
        vertShaderStageInfo.pName = "main";

        VkPipelineShaderStageCreateInfo fragShaderStageInfo{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
        fragShaderStageInfo.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
        fragShaderStageInfo.module = fragShaderModule;
        fragShaderStageInfo.pName = "main";

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
        rasterizer.polygonMode = VK_POLYGON_MODE_FILL; rasterizer.lineWidth = 1.0f; rasterizer.cullMode = VK_CULL_MODE_NONE;
        rasterizer.frontFace = VK_FRONT_FACE_CLOCKWISE;

        VkPipelineMultisampleStateCreateInfo multisampling{VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO};
        multisampling.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

        VkPipelineColorBlendAttachmentState colorBlendAttachment{};
        colorBlendAttachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
        colorBlendAttachment.blendEnable = VK_FALSE;

        VkPipelineColorBlendStateCreateInfo colorBlending{VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO};
        colorBlending.attachmentCount = 1; colorBlending.pAttachments = &colorBlendAttachment;

        // Dynamic State for Viewport (optional, keeping static here for brevity)
        VkPipelineDynamicStateCreateInfo dynamicState{VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO};
        std::vector<VkDynamicState> dynamicStates = { VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR };
        dynamicState.dynamicStateCount = static_cast<uint32_t>(dynamicStates.size());
        dynamicState.pDynamicStates = dynamicStates.data();

        VkPipelineLayoutCreateInfo pipelineLayoutInfo{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
        pipelineLayoutInfo.setLayoutCount = 1;
        pipelineLayoutInfo.pSetLayouts = &descriptorSetLayout;
        
        if (vkCreatePipelineLayout(device, &pipelineLayoutInfo, nullptr, &pipelineLayout) != VK_SUCCESS)
            throw std::runtime_error("failed to create pipeline layout!");

        VkGraphicsPipelineCreateInfo pipelineInfo{VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO};
        pipelineInfo.stageCount = 2; pipelineInfo.pStages = shaderStages;
        pipelineInfo.pVertexInputState = &vertexInputInfo;
        pipelineInfo.pInputAssemblyState = &inputAssembly;
        pipelineInfo.pViewportState = &viewportState;
        pipelineInfo.pRasterizationState = &rasterizer;
        pipelineInfo.pMultisampleState = &multisampling;
        pipelineInfo.pColorBlendState = &colorBlending;
        pipelineInfo.pDynamicState = &dynamicState;
        pipelineInfo.layout = pipelineLayout;
        pipelineInfo.renderPass = renderPass;
        pipelineInfo.subpass = 0;

        if (vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &graphicsPipeline) != VK_SUCCESS)
            throw std::runtime_error("failed to create graphics pipeline!");

        vkDestroyShaderModule(device, fragShaderModule, nullptr);
        vkDestroyShaderModule(device, vertShaderModule, nullptr);
    }

    void createFramebuffers() {
        swapChainFramebuffers.resize(swapChainImageViews.size());
        for (size_t i = 0; i < swapChainImageViews.size(); i++) {
            VkImageView attachments[] = { swapChainImageViews[i] };
            VkFramebufferCreateInfo framebufferInfo{VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO};
            framebufferInfo.renderPass = renderPass;
            framebufferInfo.attachmentCount = 1;
            framebufferInfo.pAttachments = attachments;
            framebufferInfo.width = swapChainExtent.width;
            framebufferInfo.height = swapChainExtent.height;
            framebufferInfo.layers = 1;
            if (vkCreateFramebuffer(device, &framebufferInfo, nullptr, &swapChainFramebuffers[i]) != VK_SUCCESS)
                throw std::runtime_error("failed to create framebuffer!");
        }
    }

    void createCommandPool() {
        VkCommandPoolCreateInfo poolInfo{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
        poolInfo.queueFamilyIndex = 0; // Assuming 0
        poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
        if (vkCreateCommandPool(device, &poolInfo, nullptr, &commandPool) != VK_SUCCESS)
            throw std::runtime_error("failed to create command pool!");
    }
    
    // --- 资源创建 ---
    uint32_t findMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags properties) {
        VkPhysicalDeviceMemoryProperties memProperties;
        vkGetPhysicalDeviceMemoryProperties(physicalDevice, &memProperties);
        for (uint32_t i = 0; i < memProperties.memoryTypeCount; i++) {
            if ((typeFilter & (1 << i)) && (memProperties.memoryTypes[i].propertyFlags & properties) == properties) return i;
        }
        throw std::runtime_error("failed to find suitable memory type!");
    }

    void createStagingBuffer() {
        VkDeviceSize size = WIDTH * HEIGHT * 4;
        VkBufferCreateInfo bufferInfo{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
        bufferInfo.size = size; bufferInfo.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
        if (vkCreateBuffer(device, &bufferInfo, nullptr, &stagingBuffer) != VK_SUCCESS) throw std::runtime_error("failed to create buffer");
        VkMemoryRequirements memRequirements; vkGetBufferMemoryRequirements(device, stagingBuffer, &memRequirements);
        VkMemoryAllocateInfo allocInfo{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
        allocInfo.allocationSize = memRequirements.size;
        allocInfo.memoryTypeIndex = findMemoryType(memRequirements.memoryTypeBits, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        if (vkAllocateMemory(device, &allocInfo, nullptr, &stagingBufferMemory) != VK_SUCCESS) throw std::runtime_error("failed to allocate buffer memory");
        vkBindBufferMemory(device, stagingBuffer, stagingBufferMemory, 0);
    }
    
    void createTextureImage() {
        VkImageCreateInfo imageInfo{VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO};
        imageInfo.imageType = VK_IMAGE_TYPE_2D; imageInfo.extent.width = WIDTH; imageInfo.extent.height = HEIGHT;
        imageInfo.extent.depth = 1; imageInfo.mipLevels = 1; imageInfo.arrayLayers = 1;
        imageInfo.format = VK_FORMAT_R8G8B8A8_SRGB; imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
        imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        imageInfo.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
        imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;
        if (vkCreateImage(device, &imageInfo, nullptr, &textureImage) != VK_SUCCESS) throw std::runtime_error("failed to create image");
        VkMemoryRequirements memRequirements; vkGetImageMemoryRequirements(device, textureImage, &memRequirements);
        VkMemoryAllocateInfo allocInfo{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
        allocInfo.allocationSize = memRequirements.size;
        allocInfo.memoryTypeIndex = findMemoryType(memRequirements.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
        if (vkAllocateMemory(device, &allocInfo, nullptr, &textureImageMemory) != VK_SUCCESS) throw std::runtime_error("failed to allocate image memory");
        vkBindImageMemory(device, textureImage, textureImageMemory, 0);
    }

    void createTextureImageView() {
        VkImageViewCreateInfo viewInfo{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
        viewInfo.image = textureImage; viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
        viewInfo.format = VK_FORMAT_R8G8B8A8_SRGB;
        viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        viewInfo.subresourceRange.levelCount = 1; viewInfo.subresourceRange.layerCount = 1;
        if (vkCreateImageView(device, &viewInfo, nullptr, &textureImageView) != VK_SUCCESS) throw std::runtime_error("failed to create texture image view!");
    }

    void createTextureSampler() {
        VkSamplerCreateInfo samplerInfo{VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO};
        samplerInfo.magFilter = VK_FILTER_NEAREST; samplerInfo.minFilter = VK_FILTER_NEAREST; // 使用 Nearest 保持软光栅像素感
        samplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT; samplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;
        samplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT;
        samplerInfo.anisotropyEnable = VK_FALSE; samplerInfo.maxAnisotropy = 1.0f;
        samplerInfo.borderColor = VK_BORDER_COLOR_INT_OPAQUE_BLACK;
        samplerInfo.unnormalizedCoordinates = VK_FALSE; samplerInfo.compareEnable = VK_FALSE;
        samplerInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
        if (vkCreateSampler(device, &samplerInfo, nullptr, &textureSampler) != VK_SUCCESS) throw std::runtime_error("failed to create texture sampler!");
    }

    void createDescriptorPool() {
        VkDescriptorPoolSize poolSize{}; poolSize.type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER; poolSize.descriptorCount = static_cast<uint32_t>(MAX_FRAMES_IN_FLIGHT);
        VkDescriptorPoolCreateInfo poolInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
        poolInfo.poolSizeCount = 1; poolInfo.pPoolSizes = &poolSize; poolInfo.maxSets = static_cast<uint32_t>(MAX_FRAMES_IN_FLIGHT);
        if (vkCreateDescriptorPool(device, &poolInfo, nullptr, &descriptorPool) != VK_SUCCESS) throw std::runtime_error("failed to create descriptor pool!");
    }

    void createDescriptorSets() {
        std::vector<VkDescriptorSetLayout> layouts(MAX_FRAMES_IN_FLIGHT, descriptorSetLayout);
        VkDescriptorSetAllocateInfo allocInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        allocInfo.descriptorPool = descriptorPool; allocInfo.descriptorSetCount = static_cast<uint32_t>(MAX_FRAMES_IN_FLIGHT);
        allocInfo.pSetLayouts = layouts.data();
        descriptorSets.resize(MAX_FRAMES_IN_FLIGHT);
        if (vkAllocateDescriptorSets(device, &allocInfo, descriptorSets.data()) != VK_SUCCESS) throw std::runtime_error("failed to allocate descriptor sets!");
        for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) {
            VkDescriptorImageInfo imageInfo{}; imageInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
            imageInfo.imageView = textureImageView; imageInfo.sampler = textureSampler;
            VkWriteDescriptorSet descriptorWrite{VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
            descriptorWrite.dstSet = descriptorSets[i]; descriptorWrite.dstBinding = 0;
            descriptorWrite.dstArrayElement = 0; descriptorWrite.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            descriptorWrite.descriptorCount = 1; descriptorWrite.pImageInfo = &imageInfo;
            vkUpdateDescriptorSets(device, 1, &descriptorWrite, 0, nullptr);
        }
    }
    
    void createCommandBuffers() {
        commandBuffers.resize(MAX_FRAMES_IN_FLIGHT);
        VkCommandBufferAllocateInfo allocInfo{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
        allocInfo.commandPool = commandPool; allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        allocInfo.commandBufferCount = (uint32_t)commandBuffers.size();
        if (vkAllocateCommandBuffers(device, &allocInfo, commandBuffers.data()) != VK_SUCCESS) throw std::runtime_error("failed to allocate command buffers!");
    }
    
    void createSyncObjects() {
        imageAvailableSemaphores.resize(MAX_FRAMES_IN_FLIGHT); renderFinishedSemaphores.resize(MAX_FRAMES_IN_FLIGHT); inFlightFences.resize(MAX_FRAMES_IN_FLIGHT);
        VkSemaphoreCreateInfo semaphoreInfo{VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO};
        VkFenceCreateInfo fenceInfo{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO}; fenceInfo.flags = VK_FENCE_CREATE_SIGNALED_BIT;
        for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) {
            if (vkCreateSemaphore(device, &semaphoreInfo, nullptr, &imageAvailableSemaphores[i]) != VK_SUCCESS ||
                vkCreateSemaphore(device, &semaphoreInfo, nullptr, &renderFinishedSemaphores[i]) != VK_SUCCESS ||
                vkCreateFence(device, &fenceInfo, nullptr, &inFlightFences[i]) != VK_SUCCESS) throw std::runtime_error("failed to create synchronization objects!");
        }
    }
    
    VkShaderModule createShaderModule(const std::vector<char>& code) {
        VkShaderModuleCreateInfo createInfo{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
        createInfo.codeSize = code.size(); createInfo.pCode = reinterpret_cast<const uint32_t*>(code.data());
        VkShaderModule shaderModule;
        if (vkCreateShaderModule(device, &createInfo, nullptr, &shaderModule) != VK_SUCCESS) throw std::runtime_error("failed to create shader module!");
        return shaderModule;
    }

    void mainLoop() {
        while (!glfwWindowShouldClose(window)) {
            glfwPollEvents();
            processInput();
            drawFrame();
        }
        vkDeviceWaitIdle(device);
    }
    
    void drawFrame() {
        vkWaitForFences(device, 1, &inFlightFences[currentFrame], VK_TRUE, UINT64_MAX);
        uint32_t imageIndex;
        VkResult result = vkAcquireNextImageKHR(device, swapChain, UINT64_MAX, imageAvailableSemaphores[currentFrame], VK_NULL_HANDLE, &imageIndex);
        if (result == VK_ERROR_OUT_OF_DATE_KHR) { recreateSwapChain(); return; }
        else if (result != VK_SUCCESS && result != VK_SUBOPTIMAL_KHR) throw std::runtime_error("failed to acquire swap chain image!");
        vkResetFences(device, 1, &inFlightFences[currentFrame]);
        
        updateTexture(); // CPU 光栅化 并 上传
        
        vkResetCommandBuffer(commandBuffers[currentFrame], 0);
        VkCommandBuffer cb = commandBuffers[currentFrame];
        VkCommandBufferBeginInfo beginInfo{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
        vkBeginCommandBuffer(cb, &beginInfo);
        
        // Image Barrier: Texture undefined -> Transfer Dst
        VkImageMemoryBarrier b1{VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER};
        b1.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED; b1.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
        b1.image = textureImage; b1.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
        b1.srcAccessMask = 0; b1.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
        vkCmdPipelineBarrier(cb, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 0, nullptr, 1, &b1);
        
        VkBufferImageCopy region{}; region.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1}; region.imageExtent = {WIDTH, HEIGHT, 1};
        vkCmdCopyBufferToImage(cb, stagingBuffer, textureImage, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);
        
        // Image Barrier: Transfer Dst -> Shader Read
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
        vkCmdEndRenderPass(cb);
        vkEndCommandBuffer(cb);
        
        VkSubmitInfo submitInfo{VK_STRUCTURE_TYPE_SUBMIT_INFO};
        VkSemaphore waitSemaphores[] = {imageAvailableSemaphores[currentFrame]};
        VkPipelineStageFlags waitStages[] = {VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT};
        submitInfo.waitSemaphoreCount = 1; submitInfo.pWaitSemaphores = waitSemaphores; submitInfo.pWaitDstStageMask = waitStages;
        submitInfo.commandBufferCount = 1; submitInfo.pCommandBuffers = &commandBuffers[currentFrame];
        VkSemaphore signalSemaphores[] = {renderFinishedSemaphores[currentFrame]};
        submitInfo.signalSemaphoreCount = 1; submitInfo.pSignalSemaphores = signalSemaphores;
        if (vkQueueSubmit(graphicsQueue, 1, &submitInfo, inFlightFences[currentFrame]) != VK_SUCCESS) throw std::runtime_error("failed to submit draw command buffer!");
        
        VkPresentInfoKHR presentInfo{VK_STRUCTURE_TYPE_PRESENT_INFO_KHR};
        presentInfo.waitSemaphoreCount = 1; presentInfo.pWaitSemaphores = signalSemaphores;
        VkSwapchainKHR swapChains[] = {swapChain};
        presentInfo.swapchainCount = 1; presentInfo.pSwapchains = swapChains; presentInfo.pImageIndices = &imageIndex;
        result = vkQueuePresentKHR(presentQueue, &presentInfo);
        if (result == VK_ERROR_OUT_OF_DATE_KHR || result == VK_SUBOPTIMAL_KHR || framebufferResized) { framebufferResized = false; recreateSwapChain(); }
        currentFrame = (currentFrame + 1) % MAX_FRAMES_IN_FLIGHT;
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
            vkDestroySemaphore(device, renderFinishedSemaphores[i], nullptr); vkDestroySemaphore(device, imageAvailableSemaphores[i], nullptr); vkDestroyFence(device, inFlightFences[i], nullptr);
        }
        vkDestroyCommandPool(device, commandPool, nullptr); vkDestroyDevice(device, nullptr);
        if (enableValidationLayers) DestroyDebugUtilsMessengerEXT(instance, debugMessenger, nullptr);
        vkDestroySurfaceKHR(instance, surface, nullptr); vkDestroyInstance(instance, nullptr);
        glfwDestroyWindow(window); glfwTerminate();
    }
    void DestroyDebugUtilsMessengerEXT(VkInstance instance, VkDebugUtilsMessengerEXT debugMessenger, const VkAllocationCallbacks* pAllocator) {
        auto func = (PFN_vkDestroyDebugUtilsMessengerEXT) vkGetInstanceProcAddr(instance, "vkDestroyDebugUtilsMessengerEXT");
        if (func != nullptr) func(instance, debugMessenger, pAllocator);
    }
};

int main(int argc, char** argv) {
    HelloVulkanApplication app;
    const char* objFile = nullptr;
    if (argc > 1) {
        objFile = argv[1];
    } else {
        std::cout << "Usage: ./VulkanApp.exe model.obj" << std::endl;
        std::cout << "No model specified, loading default triangle." << std::endl;
    }

    try {
        app.run(objFile);
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}