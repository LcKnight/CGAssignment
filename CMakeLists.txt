cmake_minimum_required(VERSION 3.10)

project(VulkanHZB)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ========================================================
# 1. 查找依赖 (依赖 Presets 设置的环境变量)
# ========================================================

# FindVulkan 模块会自动利用环境变量 VULKAN_SDK
find_package(Vulkan REQUIRED)

# GLFW 查找逻辑
find_package(glfw3 CONFIG QUIET)
if (NOT glfw3_FOUND)
    # 如果找不到 Config，尝试使用 Presets 传入的变量 GLFW_PATH
    if (DEFINED GLFW_PATH)
        message(STATUS "Using GLFW from: ${GLFW_PATH}")
        include_directories(${GLFW_PATH}/include)
        link_directories(${GLFW_PATH}/lib-vc2022) 
    else()
        message(WARNING "GLFW not found and GLFW_PATH not defined!")
    endif()
endif()

# ========================================================
# 2. Shader 编译逻辑
# ========================================================

# 利用环境变量 VULKAN_SDK 查找编译器
find_program(GLSLC_EXECUTABLE glslc 
    HINTS "$ENV{VULKAN_SDK}/Bin" 
    REQUIRED
)
message(STATUS "Using GLSL Compiler: ${GLSLC_EXECUTABLE}")

set(SHADER_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/shaders")
set(SHADER_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}") 

function(compile_shader SHADER_FILENAME SPV_FILENAME)
    add_custom_command(
        OUTPUT "${SHADER_BINARY_DIR}/${SPV_FILENAME}"
        COMMAND ${GLSLC_EXECUTABLE} "${SHADER_SOURCE_DIR}/${SHADER_FILENAME}" -o "${SHADER_BINARY_DIR}/${SPV_FILENAME}"
        DEPENDS "${SHADER_SOURCE_DIR}/${SHADER_FILENAME}"
        COMMENT "Compiling ${SHADER_FILENAME}..."
    )
    add_custom_target("Compile_${SPV_FILENAME}" DEPENDS "${SHADER_BINARY_DIR}/${SPV_FILENAME}")
    if(NOT TARGET Shaders)
        add_custom_target(Shaders ALL)
    endif()
    add_dependencies(Shaders "Compile_${SPV_FILENAME}")
endfunction()

# 编译顶点和片段着色器
compile_shader("shader.vert" "vert.spv")
compile_shader("shader.frag" "frag.spv")

# ========================================================
# 3. 生成可执行文件
# ========================================================

add_executable(VulkanApp src/main.cpp)
add_dependencies(VulkanApp Shaders) 

# 链接库
if (glfw3_FOUND)
    target_link_libraries(VulkanApp PRIVATE Vulkan::Vulkan glfw)
else()
    target_link_libraries(VulkanApp PRIVATE Vulkan::Vulkan glfw3)
endif()