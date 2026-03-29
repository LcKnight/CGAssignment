cmake_minimum_required(VERSION 3.10)
project(VulkanHZB)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(THIRDPARTY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty")

# ============================================================
# OpenMP
# ============================================================
find_package(OpenMP)

# ============================================================
# Vulkan
# ============================================================
if(WIN32)
    set(GLSLC_PATH "${THIRDPARTY_DIR}/Vulkan/Bin/glslc.exe")
    if(EXISTS "${GLSLC_PATH}")
        message(STATUS "Using Local glslc: ${GLSLC_PATH}")
        set(GLSLC_EXECUTABLE "${GLSLC_PATH}")
    else()
        find_program(GLSLC_EXECUTABLE glslc HINTS "$ENV{VULKAN_SDK}/Bin" REQUIRED)
    endif()

    set(Vulkan_INCLUDE_DIR "${THIRDPARTY_DIR}/Vulkan/Include")
    set(Vulkan_LIBRARY    "${THIRDPARTY_DIR}/Vulkan/Lib/vulkan-1.lib")
    if(EXISTS "${Vulkan_INCLUDE_DIR}" AND EXISTS "${Vulkan_LIBRARY}")
        message(STATUS "Using Local Vulkan SDK from ThirdParty")
        if(NOT TARGET Vulkan::Vulkan)
            add_library(Vulkan::Vulkan UNKNOWN IMPORTED)
            set_target_properties(Vulkan::Vulkan PROPERTIES
                IMPORTED_LOCATION             "${Vulkan_LIBRARY}"
                INTERFACE_INCLUDE_DIRECTORIES "${Vulkan_INCLUDE_DIR}")
        endif()
    else()
        message(WARNING "Local Vulkan not found, trying system SDK...")
        find_package(Vulkan REQUIRED)
    endif()
else()
    find_package(Vulkan REQUIRED)
    find_program(GLSLC_EXECUTABLE glslc REQUIRED)
    message(STATUS "Found System glslc: ${GLSLC_EXECUTABLE}")
endif()

# ============================================================
# GLFW
# ============================================================
if(WIN32)
    if(MSVC)
        set(GLFW_LIBRARY_DIR "${THIRDPARTY_DIR}/GLFW/lib-msvc")
    else()
        set(GLFW_LIBRARY_DIR "${THIRDPARTY_DIR}/GLFW/lib-mingw")
    endif()
    set(GLFW_INCLUDE_DIR "${THIRDPARTY_DIR}/GLFW/include")
    if(EXISTS "${GLFW_LIBRARY_DIR}")
        include_directories("${GLFW_INCLUDE_DIR}")
        link_directories("${GLFW_LIBRARY_DIR}")
        set(GLFW_LIBRARIES glfw3)
    else()
        find_package(glfw3 CONFIG REQUIRED)
        set(GLFW_LIBRARIES glfw)
    endif()
else()
    find_package(glfw3 REQUIRED)
    set(GLFW_LIBRARIES glfw)
endif()

# ============================================================
# Shader compilation
# ============================================================
set(SHADER_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/shaders")
set(SHADER_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}")

function(compile_shader SHADER_FILENAME SPV_FILENAME)
    add_custom_command(
        OUTPUT  "${SHADER_BINARY_DIR}/${SPV_FILENAME}"
        COMMAND "${GLSLC_EXECUTABLE}" "${SHADER_SOURCE_DIR}/${SHADER_FILENAME}"
                -o "${SHADER_BINARY_DIR}/${SPV_FILENAME}"
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

# ============================================================
# CoreLib  —  shared math + scene + sampling + output
# (No Vulkan dependency, no main(), no stb/tinyobj defines here)
# ============================================================
add_library(CoreLib STATIC
    src/scene/bvh.cpp
    src/scene/loader.cpp      # defines TINYOBJLOADER_IMPLEMENTATION + STB_IMAGE_IMPLEMENTATION
    src/sampling/brdf.cpp
    src/sampling/light.cpp
    src/output/tonemap.cpp    # defines STB_IMAGE_WRITE_IMPLEMENTATION
)
target_include_directories(CoreLib PUBLIC
    src
    "${THIRDPARTY_DIR}/tinyobjloader"
    "${THIRDPARTY_DIR}/stb"
)
if(OpenMP_CXX_FOUND)
    target_link_libraries(CoreLib PUBLIC OpenMP::OpenMP_CXX)
endif()

# ============================================================
# PathTracer
# ============================================================
add_executable(PathTracer
    src/pt_main.cpp
    src/integrator/pathtracer.cpp
)
target_link_libraries(PathTracer PRIVATE CoreLib)
# stb/tinyobj headers already visible via CoreLib's include dirs

# ============================================================
# VulkanApp
# ============================================================
add_executable(VulkanApp
    src/vk_main.cpp
    src/vulkan/app.cpp
)
add_dependencies(VulkanApp Shaders)
target_include_directories(VulkanApp PRIVATE src)
target_link_libraries(VulkanApp PRIVATE
    CoreLib
    Vulkan::Vulkan
    ${GLFW_LIBRARIES}
)
if(WIN32)
    target_link_libraries(VulkanApp PRIVATE gdi32 user32 shell32)
endif()
