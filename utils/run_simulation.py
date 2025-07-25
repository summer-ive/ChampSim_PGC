from pathlib import Path
import subprocess
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
from collections import deque
import argparse

# 並列ジョブ数
NJOBS = 126

# 設定
BASE_DIR = Path(__file__).parent.parent
TRACES_DIR = BASE_DIR / "traces" / "DPC-3"
FILTER_FILE = BASE_DIR / "utils" / "high_mpki_traces.txt"
LOG_BASE_DIR = BASE_DIR / "logs"
BIN_PATH = BASE_DIR / "bin" / "champsim"
CMD_ARGS = ["--warmup-instructions", "200000000", "--simulation-instructions", "500000000"]


# トレース実行関数
def run_trace(trace_path: Path, log_dir: Path):
    basename = trace_path.stem  # .xz除去
    log_path = log_dir / f"{basename}.log"
    print(f"{datetime.now().strftime('%H:%M:%S')} [ChampSim] Running {basename}")

    try:
        with open(log_path, "w") as log_file:
            subprocess.run(
                ["nice", "-n", "6", str(BIN_PATH), *CMD_ARGS, str(trace_path)],
                stdout=log_file,
                stderr=subprocess.STDOUT,
                check=True,
            )
    except subprocess.CalledProcessError as e:
        print(f"Error running {basename}: {e}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("prefetcher_name")
    args = parser.parse_args()

    # 出力ディレクトリ作成
    log_dir = LOG_BASE_DIR / str(args.prefetcher_name)
    log_dir.mkdir(parents=True, exist_ok=True)

    # トレース一覧取得
    trace_files = deque(TRACES_DIR.glob("*.xz"))

    # フィルター取得
    with open(FILTER_FILE, "r", encoding="utf-8") as f:
        trace_filter: set[str] = set(line.strip() for line in f.readlines())

    for _ in range(len(trace_files)):
        trace_file = trace_files.popleft()
        if trace_file.stem in trace_filter:
            trace_files.append(trace_file)

    # 並列実行
    with ThreadPoolExecutor(max_workers=NJOBS) as executor:
        executor.map(run_trace, trace_files, [log_dir] * len(trace_files))


if __name__ == "__main__":
    main()
