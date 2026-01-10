import subprocess
import os
import glob
import matplotlib.pyplot as plt
import sys
import argparse
import re
from statistics import mean

# --- 配置区域 ---
POSSIBLE_BUILD_DIRS = [
    "build/windows-msvc-release",
    "build/windows-mingw-release", 
    "build/Release", "build/Debug", "build", "."
]
EXE_NAME = "VulkanApp.exe"
ASSETS_DIR = "assets"
OUTPUT_DIR = "benchmark_report"

# 算法模式定义
MODES = {
    1: "Z-Buffer (Baseline)",
    2: "Scanline",
    3: "HZB Simple",
    4: "HZB Complete"
}

# 场景定义
SCENARIOS = {
    0: "Base Scenario",
    1: "Heavy Instancing",
    2: "Small Occlusion",
    3: "Large Occlusion"
}

def find_executable():
    """查找可执行文件"""
    for d in POSSIBLE_BUILD_DIRS:
        path = os.path.join(d, EXE_NAME)
        if os.path.exists(path): return os.path.abspath(path)
    return None

def count_faces(obj_path):
    """统计 OBJ 文件的面片数 (三角形数)"""
    count = 0
    try:
        with open(obj_path, 'r', encoding='utf-8', errors='ignore') as f:
            for line in f:
                if line.startswith('f '):
                    count += 1
    except Exception as e:
        print(f"[Warn] Could not read {obj_path}: {e}")
    return count

def scan_and_sort_models(assets_dir):
    """扫描 assets 目录并按面片数排序"""
    if not os.path.exists(assets_dir):
        print(f"[Error] Assets dir '{assets_dir}' not found.")
        return []
    
    models = []
    print(f"Scanning models in '{assets_dir}'...")
    for file_path in glob.glob(os.path.join(assets_dir, "*.obj")):
        faces = count_faces(file_path)
        models.append({"path": file_path, "name": os.path.basename(file_path), "faces": faces})
    
    # 按面片数从小到大排序
    models.sort(key=lambda x: x["faces"])
    
    print(f"Found {len(models)} models:")
    for m in models:
        print(f"  - {m['name']:<20} | Faces: {m['faces']:,}")
    print("-" * 50)
    return models

def parse_profile_line(line):
    """
    解析 C++ 输出的 Profile 行:
    [Profile Mode 4] Clear: 1.58ms | Vertex: 0.02ms | PrePass: 8.66ms | HZB: 12.03ms | Cull: 0.28ms | Raster: 8.51ms | Total: 30.80ms
    """
    data = {}
    # 使用正则提取所有 "Key: Valuems"
    matches = re.findall(r'(\w+):\s*([\d\.]+)ms', line)
    for key, val in matches:
        data[key] = float(val)
    return data

def run_benchmark(exe_path, model_path, mode, scenario, is_extreme):
    """运行单个测试用例，返回 FPS 历史数据和阶段耗时平均值"""
    cmd = [exe_path, os.path.abspath(model_path), str(mode), str(scenario), "benchmarkmode"]
    work_dir = os.path.dirname(exe_path)
    
    fps_history = []
    stage_samples = {
        "Clear": [], "Vertex": [], "PrePass": [], 
        "HZB": [], "Cull": [], "Raster": [], "Total": []
    }

    try:
        # 启动进程
        process = subprocess.Popen(
            cmd, cwd=work_dir, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
        )
        
        while True:
            line = process.stdout.readline()
            if not line: break
            line = line.strip()
            
            # 1. 解析 BENCH_DATA (用于画图)
            if line.startswith("BENCH_DATA"):
                parts = line.split(',')
                if len(parts) >= 3:
                    # BENCH_DATA, frame, ms, fragments
                    ms = float(parts[2])
                    if ms > 0:
                        fps_history.append(1000.0 / ms)

            # 2. 解析 Profile 数据 (用于阶段耗时统计)
            # 你的 C++ 需要输出类似 [Profile Mode X] ... 的行
            elif line.startswith("[Profile Mode"):
                stages = parse_profile_line(line)
                for k, v in stages.items():
                    if k in stage_samples:
                        stage_samples[k].append(v)
            
            elif line == "BENCHMARK_RESULT_Q1:": # 结束标志之一
                break

        process.wait()
        
    except Exception as e:
        print(f"Execution Error: {e}")
        return None, None

    # 计算平均耗时
    avg_stages = {}
    for k, v in stage_samples.items():
        if len(v) > 0:
            # 去掉前几帧的不稳定数据，取平均
            valid_samples = v[5:] if len(v) > 10 else v
            avg_stages[k] = mean(valid_samples) if valid_samples else 0.0
        else:
            avg_stages[k] = 0.0
            
    return fps_history, avg_stages

def generate_report(exe_path, models, extreme_mode):
    """主逻辑：遍历模型 -> 遍历模式 -> 收集数据 -> 输出报告"""
    
    if not os.path.exists(OUTPUT_DIR):
        os.makedirs(OUTPUT_DIR)

    # 决定测试哪些 Scenario
    scenarios_to_test = [0]
    if extreme_mode:
        scenarios_to_test = [0, 1, 2, 3]

    for model in models:
        print(f"\n[{'='*20} Testing Model: {model['name']} ({model['faces']:,} faces) {'='*20}]")
        
        # 准备画图数据: { mode_id: [fps_list] }
        model_fps_data = {} 

        # 针对 Scenario 0 打印详细耗时表
        print(f"\n--- Stage Breakdown (Scenario 0) ---")
        header = f"{'Mode':<20} | {'Clear':<7} | {'Vertex':<7} | {'PrePass':<7} | {'HZB':<7} | {'Cull':<7} | {'Raster':<7} | {'Total (ms)':<10} | {'FPS':<6}"
        print(header)
        print("-" * len(header))

        for mode_id, mode_name in MODES.items():
            # 默认只针对 Scenario 0 做详细阶段测试
            fps_hist, stages = run_benchmark(exe_path, model['path'], mode_id, 0, extreme_mode)
            
            if fps_hist:
                model_fps_data[mode_id] = fps_hist
                avg_fps = mean(fps_hist)
                
                # 打印表格行
                print(f"{mode_name:<20} | {stages['Clear']:<7.2f} | {stages['Vertex']:<7.2f} | {stages['PrePass']:<7.2f} | "
                      f"{stages['HZB']:<7.2f} | {stages['Cull']:<7.2f} | {stages['Raster']:<7.2f} | "
                      f"{stages['Total']:<10.2f} | {avg_fps:<6.1f}")
            else:
                print(f"{mode_name:<20} | FAILED TO RUN")

        # 画图：针对该 Model，不同 Mode 的 FPS 曲线 (仅 Scenario 0)
        plt.figure(figsize=(12, 6))
        for mode_id, fps_list in model_fps_data.items():
            plt.plot(fps_list, label=MODES[mode_id], linewidth=1.5)
        
        plt.title(f"Performance: {model['name']} ({model['faces']:,} tris) - Scenario 0")
        plt.xlabel("Frame")
        plt.ylabel("FPS")
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.tight_layout()
        plot_path = os.path.join(OUTPUT_DIR, f"{model['name']}_fps.png")
        plt.savefig(plot_path, dpi=120)
        plt.close()
        print(f"Saved FPS chart: {plot_path}")

        # 如果开启 Extreme，测试其他 Scenario (仅输出 Total Time 简报)
        if extreme_mode:
            print(f"\n--- Extreme Scenarios Summary (Total Time ms) ---")
            print(f"{'Mode':<20} | " + " | ".join([f"Scen {i}" for i in scenarios_to_test]))
            for mode_id, mode_name in MODES.items():
                row_str = f"{mode_name:<20} | "
                for s_id in scenarios_to_test:
                    _, stages = run_benchmark(exe_path, model['path'], mode_id, s_id, True)
                    if stages and stages['Total'] > 0:
                        row_str += f"{stages['Total']:<8.1f} | "
                    else:
                        row_str += f"{'N/A':<8} | "
                print(row_str)

def main():
    parser = argparse.ArgumentParser(description="Vulkan Soft Rasterizer Benchmark Tool")
    parser.add_argument("--extreme", action="store_true", help="Run all scenarios (0-3), otherwise run only Scenario 0")
    parser.add_argument("--dir", type=str, default=ASSETS_DIR, help="Directory containing .obj files")
    args = parser.parse_args()

    exe_path = find_executable()
    if not exe_path:
        print("[Error] VulkanApp.exe not found in build directories.")
        sys.exit(1)
    
    print(f"Target Executable: {exe_path}")
    
    # 1. 扫描并排序模型
    models = scan_and_sort_models(args.dir)
    if not models:
        print("[Error] No models found to test.")
        sys.exit(1)

    # 2. 生成报告
    generate_report(exe_path, models, args.extreme)
    
    print("\nBenchmark Suite Completed.")

if __name__ == "__main__":
    main()