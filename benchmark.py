import subprocess
import re
import os
import matplotlib.pyplot as plt

# --- 配置 ---
EXE_PATH = "VulkanApp.exe"
MODEL_PATH = "./house_t.obj"  # 请确保这里是你的高面数模型路径
IS_EXTREME = True
MODES = {
    1: "Standard Z-Buffer",
    2: "Scanline Z-Buffer",
    3: "Hierarchical Z-Buffer"
}

def run_test(mode):
    mode_name = MODES[mode]
    if IS_EXTREME:
        mode_name += " (Extreme)"
    print(f"Running {mode_name} ... ", end="", flush=True)
    
    # [修改] 构建命令，如果有 IS_EXTREME 则追加参数
    cmd = [EXE_PATH, MODEL_PATH, str(mode)]
    if IS_EXTREME:
        cmd.append("extreme")
    
    # 存储结果: [(frame_id, fps), ...]
    data_points = []
    
    try:
        # 启动进程，实时读取输出
        process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        
        while True:
            line = process.stdout.readline()
            if not line:
                break
                
            line = line.strip()
            # 匹配格式: BENCH_DATA,frame,ms
            if line.startswith("BENCH_DATA"):
                parts = line.split(',')
                if len(parts) == 3:
                    frame_id = int(parts[1])
                    frame_time_ms = float(parts[2])
                    
                    # 过滤掉前 10 帧的不稳定数据
                    if frame_id > 10:
                        fps = 1000.0 / frame_time_ms if frame_time_ms > 0 else 0
                        data_points.append((frame_id, fps))
                        
            elif line == "BENCHMARK_DONE":
                break
        
        process.wait()
        print(f"Done. Captured {len(data_points)} frames.")
        return data_points
        
    except Exception as e:
        print(f"Error: {e}")
        return []

def main():
    if not os.path.exists(EXE_PATH):
        print(f"Error: Executable {EXE_PATH} not found.")
        return

    all_results = {}

    # 1. 运行所有模式
    for mode in [1, 2, 3]:
        data = run_test(mode)
        all_results[mode] = data

    # 2. 绘图
    plt.figure(figsize=(12, 6))
    
    for mode in [1, 2, 3]:
        data = all_results[mode]
        if not data:
            continue
            
        frames = [d[0] for d in data]
        fps_list = [d[1] for d in data]
        
        # 平滑曲线 (可选，Moving Average)
        # fps_list = [sum(fps_list[i:i+5])/5 for i in range(len(fps_list)-5)]
        # frames = frames[:-5]
        
        plt.plot(frames, fps_list, label=f"Mode {mode}: {MODES[mode]}")

    plt.xlabel("Frame Index (Camera Path: Zoom Out -> Rotate -> Zoom In)")
    plt.ylabel("FPS")
    plt.title(f"Performance Analysis: {MODEL_PATH}")
    plt.legend()
    plt.grid(True, linestyle='--', alpha=0.7)
    
    # 保存图片
    chart_filename = "benchmark_chart.png"
    plt.savefig(chart_filename, dpi=150)
    print(f"\nChart saved to {chart_filename}")
    
    # 3. 简要统计
    print("\n--- Summary ---")
    for mode in [1, 2, 3]:
        data = all_results[mode]
        if data:
            avg_fps = sum(d[1] for d in data) / len(data)
            print(f"{MODES[mode]}: Avg FPS = {avg_fps:.2f}")

if __name__ == "__main__":
    main()