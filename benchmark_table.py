import subprocess
import os
import sys

# --- 配置区域 ---
POSSIBLE_BUILD_DIRS = [
    "build/windows-ninja-release", 
    "build/Release", "build/Debug", "build", "."
]
EXE_NAME = "VulkanApp.exe"
MODEL_PATH = "assets/city.obj" 
OUTPUT_DIR = "output" # [新增] 输出目录

MODES = [
    {"id": 2, "name": "Scanline Z-buffer"},
    {"id": 4, "name": "HZB Complete (Mode 4)"}, 
    {"id": 3, "name": "HZB Normal (Mode 3)"},   
    {"id": 1, "name": "Z-buffer (Baseline)"}
]

SCENARIOS = [
    {"id": 0, "name": "Low Poly (1x)"},
    {"id": 1, "name": "Heavy (20x)"},
    {"id": 2, "name": "Small Occ (5x)"},
    {"id": 3, "name": "Large Occ (5x)"}
]

def find_executable():
    for d in POSSIBLE_BUILD_DIRS:
        path = os.path.join(d, EXE_NAME)
        if os.path.exists(path): return os.path.abspath(path)
    return None

def run_benchmark(exe_path, model_path, mode, scenario):
    cmd = [exe_path, os.path.abspath(model_path), str(mode), str(scenario), "benchmarkmode"]
    work_dir = os.path.dirname(exe_path)
    
    try:
        process = subprocess.run(
            cmd, 
            cwd=work_dir, 
            capture_output=True, 
            text=True, 
            timeout=60 
        )
        
        for line in process.stdout.splitlines():
            if line.startswith("BENCHMARK_RESULT_Q1:"):
                return float(line.split(":")[1])
        return None

    except Exception as e:
        print(f"\n[Exception] {e}")
        return None

def main():
    exe_path = find_executable()
    if not exe_path: return
    if not os.path.exists(MODEL_PATH): return

    if not os.path.exists(OUTPUT_DIR):
        os.makedirs(OUTPUT_DIR)

    output_file = os.path.join(OUTPUT_DIR, "benchmark_table.txt")
    
    with open(output_file, "w") as f:
        # 重定向 stdout 到文件同时也打印到控制台
        class Tee(object):
            def __init__(self, *files): self.files = files
            def write(self, obj):
                for f in self.files: f.write(obj); f.flush()
            def flush(self):
                for f in self.files: f.flush()
        
        original_stdout = sys.stdout
        sys.stdout = Tee(sys.stdout, f)

        print(f"Target: {exe_path}")
        print("-" * 115)
        print(f"{'Algorithm':<30} | " + " | ".join([f"{s['name']:<18}" for s in SCENARIOS]))
        print("-" * 115)

        baseline_times = {}

        for mode in MODES:
            row_str = f"{mode['name']:<30} | "
            for scenario in SCENARIOS:
                # 进度条只打到 stderr，不写入文件
                print(f"Testing {mode['name']} - {scenario['name']}...", end="\r", file=sys.stderr)
                
                q1 = run_benchmark(exe_path, MODEL_PATH, mode['id'], scenario['id'])
                
                if q1 is not None:
                    if mode['id'] == 1: baseline_times[scenario['id']] = q1
                    cell = f"{q1:.1f}ms"
                    if mode['id'] != 1 and scenario['id'] in baseline_times:
                        base = baseline_times[scenario['id']]
                        if q1 > 0.001: cell += f" ({base/q1:.1f}x)"
                    row_str += f"{cell:<18} | "
                else:
                    row_str += f"{'FAIL':<18} | "
            
            print(row_str)

        print("\nDone.")
        sys.stdout = original_stdout # 恢复 stdout

if __name__ == "__main__":
    main()