from pathlib import Path
import subprocess
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime
from collections import deque
import argparse
import os

# 並列ジョブ数
NJOBS = 32

# 設定
BASE_DIR = Path(__file__).parent.parent
TRACES_DIR = BASE_DIR.parent / "trace" / "dp3_traces"
FILTER_FILE = BASE_DIR / "util" / "high_mpki_traces.txt"
EXCLUSION_FILE = BASE_DIR / "util" / "exclusion_traces.txt"
LOG_BASE_DIR = BASE_DIR / "log"
BASE_BIN_PATH = BASE_DIR / "bin"
CMD_ARGS = ["--warmup-instructions", "200000000", "--simulation-instructions", "500000000"]


# トレース実行関数
def run_trace(trace_path: Path, prefetcher_name: str, log_dir: Path, nice: int):
    basename = trace_path.stem  # .xz除去
    log_path = log_dir / f"{basename}.log"
    bin_path = str(BASE_BIN_PATH / prefetcher_name / "champsim")
    print(f"{datetime.now().strftime('%H:%M:%S')} [INFO] Running {basename}")

    try:
        with open(log_path, "w") as log_file:
            subprocess.run(
                ["nice", "-n", str(nice), str(bin_path), *CMD_ARGS, str(trace_path)],
                stdout=log_file,
                stderr=subprocess.STDOUT,
                check=True,
            )
    except subprocess.CalledProcessError as e:
        print(f"[{datetime.now():%H:%M:%S}] [ERROR] Error running {basename}: {e}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("prefetcher_name", nargs="+")
    args = parser.parse_args()

    # トレース一覧取得
    trace_files = deque(TRACES_DIR.glob("*.xz"))

    # フィルター取得
    with open(FILTER_FILE, "r", encoding="utf-8") as f:
        trace_filter: set[str] = set(line.strip() + ".champsimtrace" for line in f.readlines())

    with open(EXCLUSION_FILE, "r", encoding="utf-8") as f:
        exclusion_set: set[str] = set(line.strip() + ".champsimtrace" for line in f.readlines())

    for _ in range(len(trace_files)):
        trace_file = trace_files.popleft()
        if trace_file.stem in trace_filter and trace_file.stem not in exclusion_set:
            trace_files.append(trace_file)

    jobs = []

    # 出力ディレクトリ作成
    for prefetcher_name in args.prefetcher_name:
        log_dir = LOG_BASE_DIR / str(prefetcher_name)
        log_dir.mkdir(parents=True, exist_ok=True)

        # 一括実行ジョブ作成
        for trace_file in trace_files:
            jobs.append((prefetcher_name, log_dir, trace_file))

    jobs_count = len(jobs)
    workers_count = max(1, min(NJOBS, os.cpu_count() or 1, jobs_count))
    nice = 0
    if 64 >= workers_count > 32:
        nice = 3
    elif workers_count > 64:
        nice = 6
    print(f"{datetime.now():%H:%M:%S} [INFO] Launching {jobs_count} jobs with {workers_count} workers")

    # 並列実行
    with ThreadPoolExecutor(max_workers=workers_count) as executor:
        futures = []
        for prefetcher_name, log_dir, trace_file in jobs:
            futures.append(
                executor.submit(
                    run_trace, trace_path=trace_file, prefetcher_name=prefetcher_name, log_dir=log_dir, nice=nice
                )
            )

        done = 0
        result = None
        for future in as_completed(futures):
            try:
                result = future.result()
            except Exception as e:
                print(f"[{datetime.now():%H:%M:%S}] [ERROR] {type(e).__name__}: {e}")
            finally:
                done += 1
                print(f"{datetime.now():%H:%M:%S} [INFO] Progress: {done}/{jobs_count} Complete {result}")


if __name__ == "__main__":
    main()
