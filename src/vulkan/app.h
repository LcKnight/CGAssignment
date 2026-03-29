#pragma once
#define NOMINMAX
#include <vulkan/vulkan.h>
#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

#include "../math/vec.h"  // shared Vec3, Mat4

#include <vector>
#include <string>
#include <optional>
#include <chrono>
#include <atomic>

// ----------------------------------------------------------------
// Mesh  (CPU-side geometry for soft rasterizer + HZB)
// ----------------------------------------------------------------
struct Mesh {
    std::vector<Vec3> vertices;
    std::vector<int>  faces;
    Vec3  centerOffset   = {0,0,0};
    float normalizeScale = 1.f;
    float minY = 1e30f, maxY = -1e30f;

    struct BVHNode {
        Vec3 minB, maxB;
        int  left=-1, right=-1, triStart=-1, triCount=0;
        bool isLeaf() const { return left==-1 && right==-1; }
    };
    std::vector<BVHNode> bvhNodes;
    std::vector<int>     sortedFaces;

    void buildBVH();
    static Mesh loadObj(const std::string& filename);

private:
    int buildBVHRecursive(std::vector<int>& triIndices, int depth);
};

// ----------------------------------------------------------------
// HelloVulkanApplication
// ----------------------------------------------------------------
class HelloVulkanApplication {
public:
    void run(const char* objFilename, int mode, int scenario, bool isBenchmark);

private:
    // ---- GLFW / Vulkan handles ----
    GLFWwindow*              window        = nullptr;
    VkInstance               instance      = VK_NULL_HANDLE;
    VkDebugUtilsMessengerEXT debugMessenger= VK_NULL_HANDLE;
    VkSurfaceKHR             surface       = VK_NULL_HANDLE;
    VkPhysicalDevice         physicalDevice= VK_NULL_HANDLE;
    VkDevice                 device        = VK_NULL_HANDLE;
    VkQueue                  graphicsQueue = VK_NULL_HANDLE;
    VkQueue                  presentQueue  = VK_NULL_HANDLE;
    VkSwapchainKHR           swapChain     = VK_NULL_HANDLE;
    std::vector<VkImage>     swapChainImages;
    VkFormat                 swapChainImageFormat{};
    VkExtent2D               swapChainExtent{};
    std::vector<VkImageView> swapChainImageViews;
    std::vector<VkFramebuffer> swapChainFramebuffers;
    VkRenderPass             renderPass        = VK_NULL_HANDLE;
    VkPipelineLayout         pipelineLayout    = VK_NULL_HANDLE;
    VkPipeline               graphicsPipeline  = VK_NULL_HANDLE;
    VkDescriptorPool         descriptorPool    = VK_NULL_HANDLE;
    VkDescriptorSetLayout    descriptorSetLayout=VK_NULL_HANDLE;
    std::vector<VkDescriptorSet> descriptorSets;
    VkCommandPool            commandPool       = VK_NULL_HANDLE;
    std::vector<VkCommandBuffer> commandBuffers;
    VkBuffer                 stagingBuffer     = VK_NULL_HANDLE;
    VkDeviceMemory           stagingBufferMemory=VK_NULL_HANDLE;
    VkImage                  textureImage      = VK_NULL_HANDLE;
    VkDeviceMemory           textureImageMemory= VK_NULL_HANDLE;
    VkImageView              textureImageView  = VK_NULL_HANDLE;
    VkSampler                textureSampler    = VK_NULL_HANDLE;
    std::vector<VkSemaphore> imageAvailableSemaphores;
    std::vector<VkSemaphore> renderFinishedSemaphores;
    std::vector<VkFence>     inFlightFences;
    uint32_t currentFrame        = 0;
    bool     framebufferResized  = false;
    float    lastFrameTime       = 0.f;

    // ---- Benchmark ----
    bool  benchmarkMode     = false;
    int   benchmarkScenario = 0;
    int   benchmarkFrameCount = 0;
    static constexpr int TOTAL_TEST_FRAMES = 1200;
    std::chrono::high_resolution_clock::time_point startTime;
    std::vector<double> frameTimeHistory;

    // ---- HZB ----
    std::vector<std::vector<float>>       hzb;
    std::vector<std::pair<int,int>>       hzbDims;
    std::vector<Vec3>                     cachedProjectedVerts;

    // ---- Helpers ----
    float getHzbDepth(int level, int x, int y);
    void  buildHZB();
    bool  queryHZB(int minX, int maxX, int minY, int maxY, float minZ);

    uint64_t scanlineRasterizeTri(Vec3 v0, Vec3 v1, Vec3 v2,
                                  uint8_t r, uint8_t g, uint8_t b);
    Vec3 barycentric(Vec3 p, Vec3 a, Vec3 b, Vec3 c);
    void softRasterize();

    double get_ms(std::chrono::high_resolution_clock::time_point s,
                  std::chrono::high_resolution_clock::time_point e);

    // ---- Vulkan init / loop ----
    void initWindow();
    void initVulkan();
    void mainLoop();
    void drawFrame();
    void cleanup();
    void processInput();
    void updateTexture();

    void createInstance();
    void setupDebugMessenger();
    void createSurface();
    void pickPhysicalDevice();
    void createLogicalDevice();
    void createSwapChain();
    void createImageViews();
    void createRenderPass();
    void createDescriptorSetLayout();
    void createGraphicsPipeline();
    void createFramebuffers();
    void createCommandPool();
    void createTextureResources();
    void createDescriptorPool();
    void createDescriptorSets();
    void createCommandBuffers();
    void createSyncObjects();
    void recreateSwapChain();
    void cleanupSwapChain();

    bool isDeviceSuitable(VkPhysicalDevice dev);
    bool checkDeviceExtensionSupport(VkPhysicalDevice dev);
    bool checkValidationLayerSupport();
    std::vector<const char*> getRequiredExtensions();
    uint32_t findMemoryType(uint32_t filter, VkMemoryPropertyFlags props);
    void createBuffer(VkDeviceSize size, VkBufferUsageFlags usage,
                      VkMemoryPropertyFlags props,
                      VkBuffer& buf, VkDeviceMemory& mem);

    // GLFW callbacks (static)
    static void framebufferResizeCallback(GLFWwindow*, int, int);
    static void keyCallback(GLFWwindow*, int, int, int, int);
    static void scrollCallback(GLFWwindow*, double, double);
    static void mouseButtonCallback(GLFWwindow*, int, int, int);
    static void cursorPosCallback(GLFWwindow*, double, double);
    static VKAPI_ATTR VkBool32 VKAPI_CALL debugCallback(
        VkDebugUtilsMessageSeverityFlagBitsEXT,
        VkDebugUtilsMessageTypeFlagsEXT,
        const VkDebugUtilsMessengerCallbackDataEXT*, void*);

    void DestroyDebugUtilsMessengerEXT(VkInstance inst,
        VkDebugUtilsMessengerEXT dbg,
        const VkAllocationCallbacks* alloc);

    struct QueueFamilyIndices {
        std::optional<uint32_t> graphicsFamily;
        std::optional<uint32_t> presentFamily;
        bool isComplete() const {
            return graphicsFamily.has_value() && presentFamily.has_value();
        }
    };
    struct SwapChainSupportDetails {
        VkSurfaceCapabilitiesKHR        capabilities{};
        std::vector<VkSurfaceFormatKHR> formats;
        std::vector<VkPresentModeKHR>   presentModes;
    };
    QueueFamilyIndices      findQueueFamilies(VkPhysicalDevice dev);
    SwapChainSupportDetails querySwapChainSupport(VkPhysicalDevice dev);
    VkSurfaceFormatKHR      chooseSwapSurfaceFormat(const std::vector<VkSurfaceFormatKHR>&);
    VkPresentModeKHR        chooseSwapPresentMode(const std::vector<VkPresentModeKHR>&);
    VkExtent2D              chooseSwapExtent(const VkSurfaceCapabilitiesKHR&);
    VkShaderModule          createShaderModule(const std::vector<char>&);

    static std::vector<char> readFile(const std::string& filename);
};