虽然我后面不会提交这一版，但是考虑到后面可能有人会看到这一版说不定作为某种参考。

这一初始版本为了更方便和ai的拉扯和部分代码重写所以直接all in one file，也是经典的手写软光栅作为vertex shader和 fragment shader，模式3使用了Hierarchy Z-Buffer + BVH，然后最终绘制到屏幕上时采用了vulkan作为api，起到一个锻炼学习的效果。


## 配置说明

```bash
#1. 修改 CMakePresets.json 中的 VULKAN_SDK 和 GLFW_PATH 为你的实际路径
#2. 使用 CMake 生成工程 (以 Ninja 为例)
cmake --preset windows-ninja-release
#3. 编译
cmake --build build/windows-ninja-release
```

**Vulkan 的角色**：说明 Vulkan 在这里不负责 3D 变换，而是作为一个**高性能的帧缓冲显示器**。它负责将 CPU 端计算好的内存块（Soft Rasterizer Result）映射为显存纹理并推向屏幕。
## 执行说明
- `./VulkanApp model.obj 1`: 基础渲染 
- `./VulkanApp model.obj 2`:  BVH+Scanline-ZBuffer 
- `./VulkanApp model.obj 3`:  HZB + BVH 遮挡剔除
鼠标滚轮缩放，鼠标左键按住旋转，按q切换旋转轴。

请确保./目录下有两个spv文件和你的obj文件，因为代码是这样写的 bro，后面再改了。

python是一个很简单的benchmark 后面再改了

最终结果来看在100k面片层级来说 bvh和hzb的开销远远大于他们带来的性能提升

但是我在开启了Openmp之后，效果就有了比较显著的不同
`bool benchmarkMode = false;`benchmarkMode用来启停是否是跑分模式，跑分模式会跑固定的帧数