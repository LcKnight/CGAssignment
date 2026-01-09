cmake_minimum_required(VERSION 3.10)

project(VulkanHZB)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 定义 ThirdParty 路径 (相对于项目根目录)
set(THIRDPARTY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty")

# 查找 OpenMP
find_package(OpenMP REQUIRED)

# ========================================================
# 1. 配置 Vulkan (优先使用 ThirdParty)
# ========================================================

# 指定 glslc 编译器路径
set(GLSLC_PATH "${THIRDPARTY_DIR}/Vulkan/Bin/glslc.exe")

if(EXISTS "${GLSLC_PATH}")
    message(STATUS "Using Local glslc: ${GLSLC_PATH}")
    set(GLSLC_EXECUTABLE "${GLSLC_PATH}")
else()
    # 如果本地没有，再尝试从系统环境变量找
    find_program(GLSLC_EXECUTABLE glslc HINTS "$ENV{VULKAN_SDK}/Bin" REQUIRED)
endif()

# 指定 Vulkan 头文件和库路径
set(Vulkan_INCLUDE_DIR "${THIRDPARTY_DIR}/Vulkan/Include")
set(Vulkan_LIBRARY "${THIRDPARTY_DIR}/Vulkan/Lib/vulkan-1.lib")

# 如果 ThirdParty 存在，强制使用它；否则尝试系统查找
if(EXISTS "${Vulkan_INCLUDE_DIR}" AND EXISTS "${Vulkan_LIBRARY}")
    message(STATUS "Using Local Vulkan SDK from ThirdParty")
    # 手动创建 Vulkan::Vulkan 目标，模仿 find_package 的行为
    if(NOT TARGET Vulkan::Vulkan)
        add_library(Vulkan::Vulkan UNKNOWN IMPORTED)
        set_target_properties(Vulkan::Vulkan PROPERTIES
            IMPORTED_LOCATION "${Vulkan_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${Vulkan_INCLUDE_DIR}"
        )
    endif()
else()
    message(WARNING "Local Vulkan not found, trying system SDK...")
    find_package(Vulkan REQUIRED)
endif()

# ========================================================
# 2. 配置 GLFW (优先使用 ThirdParty)
# ========================================================

set(GLFW_INCLUDE_DIR "${THIRDPARTY_DIR}/GLFW/include")
set(GLFW_LIBRARY_DIR "${THIRDPARTY_DIR}/GLFW/lib")

if(EXISTS "${GLFW_INCLUDE_DIR}")
    message(STATUS "Using Local GLFW from: ${THIRDPARTY_DIR}/GLFW")
    include_directories("${GLFW_INCLUDE_DIR}")
    link_directories("${GLFW_LIBRARY_DIR}")
    
    # 假设静态链接 GLFW
    set(GLFW_LIBRARIES glfw3) 
else()
    # 回退到系统查找
    find_package(glfw3 CONFIG REQUIRED)
    set(GLFW_LIBRARIES glfw)
endif()

# ========================================================
# 3. Shader 编译逻辑
# ========================================================

set(SHADER_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/shaders")
set(SHADER_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}") 

function(compile_shader SHADER_FILENAME SPV_FILENAME)
    add_custom_command(
        OUTPUT "${SHADER_BINARY_DIR}/${SPV_FILENAME}"
        COMMAND "${GLSLC_EXECUTABLE}" "${SHADER_SOURCE_DIR}/${SHADER_FILENAME}" -o "${SHADER_BINARY_DIR}/${SPV_FILENAME}"
        DEPENDS "${SHADER_SOURCE_DIR}/${SHADER_FILENAME}"
        COMMENT "Compiling ${SHADER_FILENAME}..."
    )
    add_custom_target("Compile_${SPV_FILENAME}" DEPENDS "${SHADER_BINARY_DIR}/${SPV_FILENAME}")
    if(NOT TARGET Shaders)
        add_custom_target(Shaders ALL)
    endif()
    add_dependencies(Shaders "Compile_${SPV_FILENAME}")
endfunction()

compile_shader("shader.vert" "vert.spv")
compile_shader("shader.frag" "frag.spv")

# ========================================================
# 4. 生成可执行文件
# ========================================================

add_executable(VulkanApp src/main.cpp)
add_dependencies(VulkanApp Shaders) 

# 链接库
target_link_libraries(VulkanApp PRIVATE Vulkan::Vulkan ${GLFW_LIBRARIES})

if(OpenMP_CXX_FOUND)
    target_link_libraries(VulkanApp PUBLIC OpenMP::OpenMP_CXX)
endif()

# 针对 Windows 即使是静态链接 GLFW 也需要的一些系统库
if(WIN32)
    target_link_libraries(VulkanApp PRIVATE gdi32 user32 shell32)
endif()