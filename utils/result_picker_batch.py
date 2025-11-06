from pathlib import Path
import os
import re
import csv

INIT_DIR: Path = Path(__file__).parent
LOG_DIR: Path = INIT_DIR.parent / "logs"
OUTPUT_DIR: Path = INIT_DIR / "extracted"


def pickup(data):
    prefetcher_names = os.listdir(LOG_DIR)
    for prefetcher_name in prefetcher_names:
        prefetcher_log_dir = LOG_DIR / prefetcher_name
        if not os.path.isdir(prefetcher_log_dir):
            continue
        data[prefetcher_name] = {}
        versions = os.listdir(prefetcher_log_dir)
        for version in versions:
            version_log_dir = prefetcher_log_dir / version
            if not os.path.isdir(version_log_dir):
                continue

            data[prefetcher_name][version] = []
            target_logs = os.listdir(version_log_dir)
            for target_log_name in target_logs:
                target_log_path = version_log_dir / target_log_name
                if os.path.isfile(target_log_path) and os.path.splitext(target_log_name)[1] == ".log":
                    # ログからデータを抽出
                    target_log_path: Path = LOG_DIR / prefetcher_name / version / target_log_name
                    trace: str = target_log_path.stem
                    with open(target_log_path, "r", encoding="utf-8") as f:
                        log_data = f.readlines()

                    ipc: float | None = None
                    instruction_count: int | None = None
                    llc_load_miss: int | None = None
                    llc_load_mpki: float | None = None
                    st_unit_size: float | None = None
                    prefetch_count: int | None = None
                    page_cross_count: int | None = None
                    true_pgc_count: int | None = None
                    true_pgc_allowed_count: int | None = None
                    discarded_pgc_count: int | None = None
                    l2c_prefetch_count: int | None = None
                    llc_prefetch_count: int | None = None
                    useful_pgc_count: int | None = None

                    for line in log_data:
                        match = re.search(r"CPU 0 cumulative IPC:\s*([\d.]+)\s+instructions:\s*(\d+)", line)
                        if match is not None:  # IPCを抽出
                            ipc = float(match.group(1))
                            instruction_count = int(match.group(2))

                        match = re.search(r"cpu0->LLC LOAD\s+ACCESS:.*?MISS:\s*(\d+)", line)
                        if match is not None:  # LLC MPKIを抽出
                            llc_load_miss = int(match.group(1))
                            llc_load_mpki = (
                                llc_load_miss / (instruction_count / 1000)
                                if llc_load_miss is not None and instruction_count is not None
                                else None
                            )

                        match = re.search(r"\[SPP\] signature-table unit size: 2^(\d+)", line)
                        if match is not None:
                            st_unit_size = int(match.group(1))

                        match = re.search(r"\[SPP\] total prefetches: (\d+)", line)
                        if match is not None:
                            prefetch_count = int(match.group(1))

                        match = re.search(r"\[SPP\] page-crossing count: (\d+)", line)
                        if match is not None:
                            page_cross_count = int(match.group(1))

                        match = re.search(r"\[SPP\] true page-crossing count: (\d+)", line)
                        if match is not None:
                            true_pgc_count = int(match.group(1))

                        match = re.search(r"\[SPP\] allowed true page-crossing count: (\d+)", line)
                        if match is not None:
                            true_pgc_allowed_count = int(match.group(1))

                        match = re.search(r"\[SPP\] discarded page-crossing count: (\d+)", line)
                        if match is not None:
                            discarded_pgc_count = int(match.group(1))

                        match = re.search(r"\[SPP\] l2c prefetches: (\d+)", line)
                        if match is not None:
                            l2c_prefetch_count = int(match.group(1))

                        match = re.search(r"\[SPP\] llc prefetches: (\d+)", line)
                        if match is not None:
                            llc_prefetch_count = int(match.group(1))

                        match = re.search(r"\[SPP\] useful pgc count: (\d+)", line)
                        if match is not None:
                            useful_pgc_count = int(match.group(1))

                    result: dict[str, str | int | float | None] = {
                        "trace": trace,
                        "ipc": ipc,
                        "instruction_count": instruction_count,
                        "llc_load_mpki": llc_load_mpki,
                    }
                    if st_unit_size is not None:
                        result["st_unit_size"] = st_unit_size
                    if prefetch_count is not None:
                        result["prefetch_count"] = prefetch_count
                    if page_cross_count is not None:
                        result["page_cross_count"] = page_cross_count
                    if true_pgc_count is not None:
                        result["true_pgc_count"] = true_pgc_count
                    if true_pgc_allowed_count is not None:
                        result["true_pgc_allowed_count"] = true_pgc_allowed_count
                    if discarded_pgc_count is not None:
                        result["discarded_pgc_count"] = discarded_pgc_count
                    if l2c_prefetch_count is not None:
                        result["l2c_prefetch_count"] = l2c_prefetch_count
                    if llc_prefetch_count is not None:
                        result["llc_prefetch_count"] = llc_prefetch_count
                    if useful_pgc_count is not None:
                        result["useful_pgc_count"] = useful_pgc_count

                    data[prefetcher_name][version].append(result)


def main():
    data: dict[str, dict[str, list[dict[str, object]]]] = {}
    pickup(data)
    OUTPUT_DIR.mkdir(exist_ok=True, parents=True)
    for prefetcher_name in data.keys():
        for version in data[prefetcher_name].keys():
            output_path: Path = OUTPUT_DIR / prefetcher_name / f"{version}.csv"
            output_path.parent.mkdir(exist_ok=True, parents=True)
            result = data[prefetcher_name][version]
            with open(output_path, mode="w", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=result[0].keys())
                writer.writeheader()
                writer.writerows(result)


if __name__ == "__main__":
    main()
