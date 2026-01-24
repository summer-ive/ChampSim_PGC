from pathlib import Path
import subprocess
from concurrent.futures import ThreadPoolExecutor, as_completed
import os
from datetime import datetime
from collections import deque
import argparse

# 並列ジョブ数
NJOBS = 126

# 設定
BASE_DIR = Path(__file__).parent.parent
TRACES_DIR = BASE_DIR.parent / "traces" / "dp3_traces"
FILTER_FILE = BASE_DIR / "utils" / "high_mpki_traces.txt"
LOG_BASE_DIR = BASE_DIR / "log" / "test"
BASE_BIN_PATH = BASE_DIR / "bin"
CMD_ARGS = ["--warmup-instructions", "200000000", "--simulation-instructions", "100000000"]

TEST_TRACES = [
    "437.leslie3d-232B.champsimtrace",
    "437.leslie3d-271B.champsimtrace",
    "481.wrf-196B.champsimtrace",
    "603.bwaves_s-2931B.champsimtrace",
]

VERSIONS = [
    # "256B", "1KB",
    "4KB",
    # "16KB", "64KB", "256KB", "1MB", "2MB", "4MB", "16MB", "64MB", "256MB"
]


# トレース実行関数
def run_trace_batch(trace_path: Path, log_dir: Path, version: str, prefetcher_name: str):
    basename = trace_path.stem  # .xz除去
    log_path = log_dir / f"{basename}.log"
    bin_path = str(BASE_BIN_PATH / prefetcher_name / ("champsim_" + version))
    print(f"{datetime.now().strftime('%H:%M:%S')} [ChampSim] Running {basename}")

    try:
        with open(log_path, "w") as log_file:
            subprocess.run(
                [bin_path, *CMD_ARGS, str(trace_path)],
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

    # トレース一覧取得
    trace_files = deque(TRACES_DIR.glob("*.xz"))

    # フィルター取得
    with open(FILTER_FILE, "r", encoding="utf-8") as f:
        trace_filter: set[str] = set(line.strip() for line in f.readlines())

    filtered_traces = []
    for _ in range(len(trace_files)):
        trace_file = trace_files.popleft()
        if trace_file.stem in trace_filter and trace_file.stem in TEST_TRACES:
            filtered_traces.append(trace_file)

    jobs = []

    for version in VERSIONS:
        # 出力ディレクトリ作成
        log_dir = LOG_BASE_DIR / str(args.prefetcher_name) / version
        log_dir.mkdir(parents=True, exist_ok=True)

        # 一括実行ジョブ作成
        for trace_file in trace_files:
            jobs.append((version, trace_file, log_dir))

    jobs_count = len(jobs)
    workers_count = max(1, min(NJOBS, os.cpu_count() or 1, jobs_count))
    print(f"{datetime.now():%H:%M:%S} Launching {jobs_count} jobs with {workers_count} workers")

    # 並列実行
    with ThreadPoolExecutor(max_workers=workers_count) as executor:
        futures = []
        for version, trace_file, log_dir in jobs:
            futures.append(
                executor.submit(
                    run_trace_batch,
                    trace_path=trace_file,
                    log_dir=log_dir,
                    version=version,
                    prefetcher_name=str(args.prefetcher_name),
                )
            )

        done = 0
        for future in as_completed(futures):
            try:
                future.result()
            except Exception as e:
                print(f"[{datetime.now():%H:%M:%S}] [ERR] {type(e).__name__}: {e}")
            finally:
                done += 1
                if done % 10 == 0 or done == jobs_count:
                    print(f"{datetime.now():%H:%M:%S} Progress: {done}/{jobs_count} finished")


if __name__ == "__main__":
    main()
