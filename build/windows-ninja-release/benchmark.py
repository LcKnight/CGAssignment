import subprocess
import re
import sys
import os

# --- 配置 ---
EXE_PATH = "VulkanApp.exe"
MODEL_PATH = "./city.obj" # 请根据你的实际 obj 路径修改这里！
MODES = {
    1: "Standard Z-Buffer",
    2: "Scanline Z-Buffer",
    3: "Hierarchical Z-Buffer (HZB + BVH)"
}

def run_test(mode):
    print(f"正在测试模式 {mode}: {MODES[mode]} ...", end="", flush=True)
    
    cmd = [EXE_PATH, MODEL_PATH, str(mode)]
    
    try:
        # 运行程序并捕获输出
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=30) # 30秒超时防止卡死
        output = result.stdout
        
        # 使用正则提取 C++ 打印的 "BENCHMARK_RESULT: ... FPS=xxx"
        match = re.search(r"FPS=([\d\.]+)", output)
        if match:
            fps = float(match.group(1))
            print(f" 完成. FPS: {fps:.2f}")
            return fps
        else:
            print(" 失败. 未找到性能数据。")
            print("程序输出片段:", output[-200:]) # 打印最后200字符用于调试
            return None
            
    except subprocess.TimeoutExpired:
        print(" 超时! 程序可能卡死了。")
        return None
    except Exception as e:
        print(f" 出错: {e}")
        return None

def main():
    if not os.path.exists(EXE_PATH):
        print(f"错误: 找不到 {EXE_PATH}，请确保脚本在 build/release 目录下运行。")
        return

    results = {}
    
    print("="*60)
    print(f"开始性能测试 - 模型: {MODEL_PATH}")
    print("="*60)

    # 依次运行三种模式
    for mode in [1, 2, 3]:
        fps = run_test(mode)
        if fps is not None:
            results[mode] = fps
        else:
            results[mode] = 0.0

    # --- 生成报告 ---
    print("\n" + "="*60)
    print(f"{'算法模式':<35} | {'FPS':<10} | {'相对加速比 (vs 模式1)':<15}")
    print("-" * 65)
    
    base_fps = results.get(1, 1.0) # 避免除以0
    
    for mode in [1, 2, 3]:
        fps = results.get(mode, 0.0)
        name = MODES[mode]
        
        if mode == 1:
            ratio = "1.00x (基准)"
        else:
            if base_fps > 0:
                r = fps / base_fps
                ratio = f"{r:.2f}x"
            else:
                ratio = "N/A"
        
        print(f"{name:<35} | {fps:<10.2f} | {ratio:<15}")
    print("="*60)

    # --- 自动分析 ---
    print("\n【性能分析结论】")
    if results[3] > results[1]:
        print(f"1. HZB (完整模式) 相比简单模式带来了 {results[3]/results[1]:.1f} 倍的性能提升。")
        print("   原因: BVH 层次包围盒有效地剔除了大量被遮挡的几何体，")
        print("         避免了对不可见三角形进行昂贵的光栅化和着色计算。")
    else:
        print("1. HZB 模式没有带来明显的性能提升，甚至可能更慢。")
        print("   可能原因: 场景过于简单(遮挡少)，或者物体都挤在屏幕前方，")
        print("   导致 BVH 构建和遍历的开销超过了剔除带来的收益。")

if __name__ == "__main__":
    main()