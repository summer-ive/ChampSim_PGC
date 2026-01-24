from pathlib import Path
import os
import re
import csv

INIT_DIR: Path = Path(__file__).parent
LOG_DIR: Path = INIT_DIR.parent / "log"
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

                    result: dict[str, object] = {"trace": trace}

                    for line in log_data:
                        match = re.search(r"Number of CPUs: (\d+)", line)
                        if match is not None:
                            result["cpu_count"] = int(match.group(1))

                        match = re.search(r"Page size: (\d+)", line)
                        if match is not None:
                            result["page_size"] = int(match.group(1))

                        match = re.search(r"Warmup Instructions: (\d+)", line)
                        if match is not None:
                            result["warmup_instruction_count"] = int(match.group(1))

                        match = re.search(
                            r"Off-chip DRAM Size: (\d+) GiB Channels: (\d+) Width: (\d+)-bit Data Rate: (\d+) MT/s",
                            line,
                        )
                        if match is not None:
                            result["dram_size_GiB"] = int(match.group(1))
                            result["dram_channels"] = int(match.group(2))
                            result["dram_width_bit"] = int(match.group(3))
                            result["dram_data_rate_MT/s"] = int(match.group(4))

                        match = re.search(r"CPU 0 cumulative IPC: ([\d.]+) instructions: (\d+) cycles: (\d+)", line)
                        if match is not None:  # IPCを抽出
                            result["ipc"] = float(match.group(1))
                            result["simulation_instruction_count"] = int(match.group(2))
                            result["simulation_cycle_count"] = int(match.group(3))

                        match = re.search(r"cpu0->LLC LOAD\s+ACCESS:.*?MISS:\s*(\d+)", line)
                        if match is not None:  # LLC MPKIを抽出
                            result["llc_load_miss"] = int(match.group(1))
                            result["llc_load_mpki"] = (
                                result["llc_load_miss"] / (result["instruction_count"] / 1000)  # type: ignore
                                if result["llc_load_miss"] is not None and result["instruction_count"] is not None
                                else None
                            )

                        match = re.search(r"\[SPP\] signature-table unit size: 2^(\d+)", line)
                        if match is not None:
                            result["st_unit_size"] = int(match.group(1))

                        match = re.search(r"\[SPP\] total prefetches: (\d+)", line)
                        if match is not None:
                            result["prefetch_count"] = int(match.group(1))
                        match = re.search(r"\[SPP\] page-crossing count: (\d+)", line)
                        if match is not None:
                            result["page_cross_count"] = int(match.group(1))

                        match = re.search(r"\[SPP\] true page-crossing count: (\d+)", line)
                        if match is not None:
                            result["true_pgc_count"] = int(match.group(1))
                        match = re.search(r"\[SPP\] allowed true page-crossing count: (\d+)", line)
                        if match is not None:
                            result["true_pgc_allowed_count"] = int(match.group(1))

                        match = re.search(r"\[SPP\] discarded page-crossing count: (\d+)", line)
                        if match is not None:
                            result["discarded_pgc_count"] = int(match.group(1))
                        match = re.search(r"\[SPP\] l2c prefetches: (\d+)", line)
                        if match is not None:
                            result["l2c_prefetch_count"] = int(match.group(1))

                        match = re.search(r"\[SPP\] llc prefetches: (\d+)", line)
                        if match is not None:
                            result["llc_prefetch_count"] = int(match.group(1))
                        match = re.search(r"\[SPP\] useful pgc count: (\d+)", line)
                        if match is not None:
                            result["useful_pgc_count"] = int(match.group(1))

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
