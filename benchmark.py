import subprocess
import os
import glob
import matplotlib.pyplot as plt
import sys

# --- 配置区域 ---
POSSIBLE_BUILD_DIRS = [
    "build/windows-ninja-release", 
    "build/Release", "build/Debug", "build", "."
]
EXE_NAME = "VulkanApp.exe"
MODEL_PATH = "assets/city.obj"
OUTPUT_DIR = "output"  # [新增] 输出目录

# 算法模式
MODES = {
    1: "Standard Z-Buffer",
    2: "Scanline Z-Buffer",
    3: "HZB Normal (Mode 3)",
    4: "HZB Complete (Mode 4)"
}

# 场景
SCENARIOS = [
    {"id": 0, "name": "Low Poly"},
    {"id": 1, "name": "Heavy"},
    {"id": 2, "name": "Small Occ"},
    {"id": 3, "name": "Large Occ"}
]

def find_executable():
    for d in POSSIBLE_BUILD_DIRS:
        path = os.path.join(d, EXE_NAME)
        if os.path.exists(path): return os.path.abspath(path)
    return None

def run_test(exe_path, model_path, mode, scenario):
    print(f"   Running Mode {mode}...", end="", flush=True)
    
    cmd = [exe_path, os.path.abspath(model_path), str(mode), str(scenario), "benchmarkmode"]
    work_dir = os.path.dirname(exe_path)

    # 存储: (Frame, TimeMs, FragmentCount)
    data_points = []
    
    try:
        process = subprocess.Popen(
            cmd, cwd=work_dir, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
        )
        
        while True:
            line = process.stdout.readline()
            if not line: break
            line = line.strip()
            
            # 解析: BENCH_DATA,frame,ms,fragments
            if line.startswith("BENCH_DATA"):
                parts = line.split(',')
                if len(parts) >= 4:
                    frame_id = int(parts[1])
                    frame_ms = float(parts[2])
                    frag_count = int(parts[3])
                    
                    if frame_id > 10 and frame_ms > 0:
                        fps = 1000.0 / frame_ms
                        data_points.append((frame_id, fps, frame_ms, frag_count))
                        
            elif line == "BENCHMARK_DONE":
                break
        
        process.wait()
        avg_fps = sum(d[1] for d in data_points) / len(data_points) if data_points else 0
        print(f" Done. Avg FPS: {avg_fps:.1f}")
        return data_points
        
    except Exception as e:
        print(f" Error: {e}")
        return []

def plot_scenario_results(exe_path, model_path, scenario):
    print(f"\nPlotting Scenario: {scenario['name']}")
    
    # 确保输出目录存在
    if not os.path.exists(OUTPUT_DIR):
        os.makedirs(OUTPUT_DIR)

    # 1. 绘制 FPS vs Frame
    plt.figure(figsize=(14, 8))
    colors = {1: 'tab:blue', 2: 'tab:orange', 3: 'tab:green', 4: 'tab:red'}
    
    # 存储所有模式的数据以备画第二张图
    all_mode_data = {}

    for mode_id in [1, 2, 3, 4]:
        data = run_test(exe_path, model_path, mode_id, scenario['id'])
        if not data: continue
        all_mode_data[mode_id] = data
            
        frames = [d[0] for d in data]
        fps_val = [d[1] for d in data]
        
        if len(fps_val) > 20:
            window = 10
            fps_val = [sum(fps_val[i:i+window])/window for i in range(len(fps_val)-window)]
            frames = frames[:-window]
        
        plt.plot(frames, fps_val, color=colors[mode_id], label=MODES[mode_id], linewidth=2)

    plt.title(f"FPS: {scenario['name']} ({os.path.basename(model_path)})", fontsize=16)
    plt.xlabel("Frame Index", fontsize=12)
    plt.ylabel("FPS", fontsize=12)
    plt.grid(True, linestyle='--', alpha=0.5)
    plt.legend()
    
    out_file_fps = os.path.join(OUTPUT_DIR, f"chart_fps_{scenario['name'].replace(' ', '_')}.png")
    plt.savefig(out_file_fps, dpi=150)
    print(f"Saved: {out_file_fps}")
    plt.close()

    # 2. 绘制 Time vs Fragment Count (Scatter Plot)
    plt.figure(figsize=(14, 8))
    
    for mode_id, data in all_mode_data.items():
        # d[3] is fragments, d[2] is time_ms
        # 为了避免点太密集，可以采样或者画透明点
        fragments = [d[3] / 1000000.0 for d in data] # 单位: 百万片元
        times = [d[2] for d in data]
        
        plt.scatter(fragments, times, color=colors[mode_id], label=MODES[mode_id], alpha=0.5, s=10)

    plt.title(f"Time vs Load: {scenario['name']}", fontsize=16)
    plt.xlabel("Processed Fragments (Millions)", fontsize=12)
    plt.ylabel("Frame Time (ms)", fontsize=12)
    plt.grid(True, linestyle='--', alpha=0.5)
    plt.legend()
    
    out_file_scatter = os.path.join(OUTPUT_DIR, f"chart_scatter_{scenario['name'].replace(' ', '_')}.png")
    plt.savefig(out_file_scatter, dpi=150)
    print(f"Saved: {out_file_scatter}")
    plt.close()

def main():
    exe_path = find_executable()
    if not exe_path:
        print("Error: VulkanApp.exe not found.")
        return

    if not os.path.exists(MODEL_PATH):
        print(f"Error: Model {MODEL_PATH} not found.")
        return

    for scenario in SCENARIOS:
        plot_scenario_results(exe_path, MODEL_PATH, scenario)

if __name__ == "__main__":
    main()