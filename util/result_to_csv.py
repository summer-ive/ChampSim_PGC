from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import csv
import re
from typing import Any, Iterable

DEFAULT_IDENTITY = {
    "prefetcher": "unknown_prefetcher",
    "GHR_ON": False,
    "REGION_SIZE": "4KB",
    "workload": "unknown_workload",
}

# ====== settings (your repo layout) ======
RESULT_DIR: Path = Path(__file__).parent.parent.parent / "file" / "result"
DEFAULT_LOG_DIR: Path = RESULT_DIR / "log"
DEFAULT_CSV_DIR: Path = RESULT_DIR / "csv"
DEFAULT_METRICS_OUTPUT_PATH: Path = DEFAULT_CSV_DIR / "result_metrics.csv"
DEFAULT_PGC_DIST_OUTPUT_PATH: Path = DEFAULT_CSV_DIR / "result_pgc_distance.csv"


# ====== regex: "=== Simulation ===" 以降をできるだけ全部拾う ======
RE_TRACE = re.compile(r"^CPU\s+0\s+runs\s+(.+)$")
RE_ROI_IPC = re.compile(r"^CPU\s+0\s+cumulative\s+IPC:\s*([\d.]+)\s+instructions:\s*(\d+)\s+cycles:\s*(\d+)\s*$")
RE_BRANCH_SUMMARY = re.compile(
    r"^CPU\s+0\s+Branch\s+Prediction\s+Accuracy:\s*([\d.]+)%\s+MPKI:\s*([\d.]+)\s+Average\s+ROB\s+Occupancy\s+at\s+Mispredict:\s*([\d.]+)\s*$"
)
RE_BRANCH_TYPE = re.compile(r"^(BRANCH_[A-Z0-9_]+):\s*([\d.]+)\s*$")

# cache/tlb stats line
RE_CACHE_LINE = re.compile(
    r"^(?P<name>\S+)\s+"
    r"(?P<kind>TOTAL|LOAD|RFO|PREFETCH|WRITE|TRANSLATION)\s+"
    r"ACCESS:\s*(?P<access>\d+)\s+"
    r"HIT:\s*(?P<hit>\d+)\s+"
    r"MISS:\s*(?P<miss>\d+)\s+"
    r"MSHR_MERGE:\s*(?P<mshr_merge>\d+)\s*$"
)

RE_PREFETCH_REQ = re.compile(
    r"^(?P<name>\S+)\s+PREFETCH\s+REQUESTED:\s*(?P<requested>\d+)\s+"
    r"ISSUED:\s*(?P<issued>\d+)\s+"
    r"USEFUL:\s*(?P<useful>\d+)\s+"
    r"USELESS:\s*(?P<useless>\d+)\s*$"
)

RE_AVG_MISS_LAT = re.compile(r"^(?P<name>\S+)\s+AVERAGE\s+MISS\s+LATENCY:\s*(?P<lat>[\d.]+)\s+cycles\s*$")

# DRAM section
RE_DRAM_CH = re.compile(r"^Channel\s+(?P<ch>\d+)\s+(?P<queue>RQ|WQ)\s+(?P<key>[A-Z0-9_ ]+):\s*(?P<val>[\d.]+)\s*$")
RE_DRAM_INDENT = re.compile(r"^\s+(?P<key>[A-Z0-9_ ]+):\s*(?P<val>[\d.]+)\s*$")
RE_DRAM_REFRESH = re.compile(r"^Channel\s+(?P<ch>\d+)\s+REFRESHES\s+ISSUED:\s*(?P<val>\d+)\s*$")

# SPP
RE_SPP_KV = re.compile(r"^\[SPP\]\s+(?P<key>[^:]+):\s*(?P<val>-?\d+)\s*$")

RE_PGC_DIST_HEADER = re.compile(r"^\[SPP\]\s+(?P<scope>.+?)\s+pgc distance:\s*$")
RE_PGC_DIST_LINE = re.compile(r"^\s*distance\s+(?P<dist>-?\d+):\s*(?P<count>\d+)\s*$")


def sanitize(s: str) -> str:
    """column-safe-ish name"""
    return (
        s.strip()
        .replace("->", "_to_")
        .replace("/", "_")
        .replace(".", "_")
        .replace(":", "_")
        .replace("-", "_")
        .replace("__", "_")
        .replace(" ", "_")
    )


def workload_from_trace_path(trace_path: str) -> str:
    """
    traces/DPC-3/400.perlbench-41B.champsimtrace.xz
      -> 400.perlbench-41B
    """
    name = Path(trace_path).name
    for suf in [".champsimtrace.xz", ".champsimtrace", ".xz", ".trace", ".txt", ".log"]:
        if name.endswith(suf):
            name = name[: -len(suf)]
            break
    return name


def find_marker_index(lines: list[str], marker: str = "=== Simulation ===") -> int | None:
    for i, line in enumerate(lines):
        if line.strip() == marker:
            return i
    return None


def parse_log(log_path: Path) -> tuple[dict[str, Any], list[tuple[str, int, int]]]:
    """
    1 log -> dict of metrics (wide dict)
    """
    lines = log_path.read_text(encoding="utf-8", errors="replace").splitlines()
    idx = find_marker_index(lines)
    if idx is None:
        return {}, []

    out: dict[str, Any] = {}

    in_branch_table = False
    in_dram = False
    dram_context: tuple[int, str] | None = None  # (ch, queue)
    current_pgc_scope: str | None = None
    pgc_dist_rows: list[tuple[str, int, int]] = []  # (scope, dist, count)

    for line in lines[idx + 1 :]:
        # trace path
        m = RE_TRACE.search(line)
        if m:
            out["workload"] = workload_from_trace_path(m.group(1).strip())
            continue

        # ROI IPC line
        m = RE_ROI_IPC.search(line)
        if m:
            out["roi_ipc"] = float(m.group(1))
            out["roi_instructions"] = int(m.group(2))
            out["roi_cycles"] = int(m.group(3))
            continue

        # Branch summary
        m = RE_BRANCH_SUMMARY.search(line)
        if m:
            out["branch_prediction_accuracy"] = float(m.group(1))
            out["branch_mpki"] = float(m.group(2))
            out["branch_avg_rob_occupancy_at_mispredict"] = float(m.group(3))
            continue

        # Branch type table
        if line.strip() == "Branch type MPKI":
            in_branch_table = True
            continue
        if in_branch_table:
            m = RE_BRANCH_TYPE.search(line.strip())
            if m:
                out[f"branch_type_mpki_{m.group(1)}".lower()] = float(m.group(2))
                continue
            if line.strip() == "":
                in_branch_table = False
            continue

        # DRAM section start/end
        if line.strip() == "DRAM Statistics":
            in_dram = True
            dram_context = None
            continue

        if in_dram:
            # refresh
            m = RE_DRAM_REFRESH.search(line)
            if m:
                ch = int(m.group("ch"))
                out[f"dram_ch{ch}_refreshes_issued"] = int(m.group("val"))
                continue

            # channel header line
            m = RE_DRAM_CH.search(line)
            if m:
                ch = int(m.group("ch"))
                queue = m.group("queue").lower()
                key = sanitize(m.group("key").strip().lower())
                val_raw = m.group("val")
                val: float | int = float(val_raw) if "." in val_raw else int(val_raw)
                out[f"dram_ch{ch}_{queue}_{key}"] = val
                dram_context = (ch, queue)
                continue

            # indented continuation lines (ROW_BUFFER_MISS etc)
            m = RE_DRAM_INDENT.search(line)
            if m and dram_context is not None:
                ch, queue = dram_context
                key = sanitize(m.group("key").strip().lower())
                val_raw = m.group("val")
                val: float | int = float(val_raw) if "." in val_raw else int(val_raw)
                out[f"dram_ch{ch}_{queue}_{key}"] = val
                continue

            # end of DRAM section
            if line.startswith("[SPP]"):
                in_dram = False
                dram_context = None
            # (not continuing here to allow other parsers like SPP to process)

        # Cache/TLB stats
        m = RE_CACHE_LINE.search(line)
        if m:
            name = sanitize(m.group("name"))
            kind = m.group("kind").lower()
            base = f"{name}_{kind}"
            out[f"{base}_access"] = int(m.group("access"))
            out[f"{base}_hit"] = int(m.group("hit"))
            out[f"{base}_miss"] = int(m.group("miss"))
            out[f"{base}_mshr_merge"] = int(m.group("mshr_merge"))
            continue

        # Prefetch request stats
        m = RE_PREFETCH_REQ.search(line)
        if m:
            name = sanitize(m.group("name"))
            base = f"{name}_prefetch"
            out[f"{base}_requested"] = int(m.group("requested"))
            out[f"{base}_issued"] = int(m.group("issued"))
            out[f"{base}_useful"] = int(m.group("useful"))
            out[f"{base}_useless"] = int(m.group("useless"))
            continue

        # Average miss latency
        m = RE_AVG_MISS_LAT.search(line)
        if m:
            name = sanitize(m.group("name"))
            out[f"{name}_avg_miss_latency_cycles"] = float(m.group("lat"))
            continue

        # SPP key/value
        m = RE_SPP_KV.search(line)
        if m:
            current_pgc_scope = None
            key = sanitize(m.group("key").strip().lower().replace(" ", "_"))
            out[f"spp_{key}"] = int(m.group("val"))
            continue

        # SPP distance distribution
        m = RE_PGC_DIST_HEADER.search(line)
        if m:
            current_pgc_scope = sanitize(m.group("scope").strip().lower()) + "_pgc"
            continue
        m = RE_PGC_DIST_LINE.search(line)
        if m and current_pgc_scope is not None:
            dist = int(m.group("dist"))
            count = int(m.group("count"))
            pgc_dist_rows.append((current_pgc_scope, dist, count))
            continue

    # derived metrics
    if "roi_instructions" in out:
        instr = int(out["roi_instructions"])
        denom = instr / 1000.0 if instr else None
        miss_key = "cpu0_to_LLC_load_miss"  # sanitize(cpu0->LLC LOAD miss) -> cpu0_to_LLC_load_miss
        if denom and miss_key in out:
            out["llc_load_mpki"] = float(out[miss_key]) / denom

    return out, pgc_dist_rows


@dataclass
class LogIdentity:
    prefetcher: str
    ghr: bool
    signature_region_size: str
    workload: str


def infer_identity_from_path(log_path: Path, parsed: dict[str, Any], log_dir: Path) -> LogIdentity:
    identity = LogIdentity(
        DEFAULT_IDENTITY["prefetcher"],
        DEFAULT_IDENTITY["GHR_ON"],
        DEFAULT_IDENTITY["REGION_SIZE"],
        DEFAULT_IDENTITY["workload"],
    )
    parts = log_path.relative_to(log_dir).parts
    for i in range(len(parts)):
        if i == 0:
            identity.prefetcher = parts[0] if len(parts) >= 1 else "unknown_prefetcher"
        elif i == len(parts) - 1:
            identity.workload = parsed.get("workload") or "unknown_workload"
        elif parts[i].lower().startswith("ghr_"):
            if parts[i].lower() == "ghr_on":
                identity.ghr = True
            elif parts[i].lower() == "ghr_off":
                identity.ghr = False
        elif parts[i].lower().startswith("st_unit_"):
            identity.signature_region_size = parts[i].split("_")[2]  # st_unit_<signature_region_size>

    return identity


def to_tidy_metrics_rows(identity: LogIdentity, metrics: dict[str, Any]) -> list[dict[str, Any]]:
    """
    wide dict -> tidy rows
    """
    rows: list[dict[str, Any]] = []
    for k, v in metrics.items():
        # treat metadata not as a metric but as a column name
        if k in {"workload"}:
            continue

        # skip None / empty values
        if v is None:
            continue

        # skip complex values like dict/list (temporarily)
        if isinstance(v, (dict, list, tuple, set)):
            continue

        rows.append(
            {
                "prefetcher": identity.prefetcher,
                "ghr": identity.ghr,
                "signature_region_size": identity.signature_region_size,
                "workload": identity.workload,
                "metric": k,
                "value": v,
            }
        )
    return rows


def to_tidy_pgc_dist_rows(identity: LogIdentity, pgc_dist_rows: list[tuple[str, int, int]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for scope, dist, count in pgc_dist_rows:
        rows.append(
            {
                "prefetcher": identity.prefetcher,
                "ghr": identity.ghr,
                "signature_region_size": identity.signature_region_size,
                "workload": identity.workload,
                "scope": scope,
                "distance": dist,
                "count": count,
            }
        )
    return rows


def iter_log_files(log_dir: Path) -> Iterable[Path]:
    """
    patterns of log directory:
    - logs/<prefetcher>/<signature_region_size>/*.log
    - logs/<prefetcher>/*.log
    the latter is 4KB signature region size by default.
    """
    yield from log_dir.rglob("*.log")


def main(
    log_dir: Path = DEFAULT_LOG_DIR,
    out_metrics_csv: Path = DEFAULT_METRICS_OUTPUT_PATH,
    out_pgc_dist_csv: Path = DEFAULT_PGC_DIST_OUTPUT_PATH,
) -> None:
    print("[INFO] Processing logs...")
    print(f"[INFO] metrics: {log_dir} -> {out_metrics_csv}")
    print(f"[INFO] pgc_dist: {log_dir} -> {out_pgc_dist_csv}")

    log_dir = log_dir.resolve()
    out_metrics_csv.parent.mkdir(parents=True, exist_ok=True)
    out_pgc_dist_csv.parent.mkdir(parents=True, exist_ok=True)

    all_metrics_rows: list[dict[str, Any]] = []
    all_pgc_dist_rows: list[dict[str, Any]] = []
    skipped_logs: list[Path] = []

    for log_path in iter_log_files(log_dir):
        parsed_metrics, parsed_pgc_dist_rows = parse_log(log_path)
        if not parsed_metrics:
            skipped_logs.append(log_path)
            continue

        identity = infer_identity_from_path(log_path, parsed_metrics, log_dir)
        metrics_rows = to_tidy_metrics_rows(identity, parsed_metrics)
        pgc_dist_rows = to_tidy_pgc_dist_rows(identity, parsed_pgc_dist_rows)
        all_metrics_rows.extend(metrics_rows)
        all_pgc_dist_rows.extend(pgc_dist_rows)

    if not all_metrics_rows:
        print(f"[INFO] No rows generated. (skipped={len(skipped_logs)})")
        return

    # write CSV
    with out_metrics_csv.open("w", encoding="utf-8", newline="") as f:
        fieldnames_metrics = ["prefetcher", "ghr", "signature_region_size", "workload", "metric", "value"]
        w = csv.DictWriter(f, fieldnames=fieldnames_metrics)
        w.writeheader()
        w.writerows(all_metrics_rows)
    with out_pgc_dist_csv.open("w", encoding="utf-8", newline="") as f:
        fieldnames_pgc = ["prefetcher", "ghr", "signature_region_size", "workload", "scope", "distance", "count"]
        w = csv.DictWriter(f, fieldnames=fieldnames_pgc)
        w.writeheader()
        w.writerows(all_pgc_dist_rows)

    print(f"[INFO] Skipped logs without marker={len(skipped_logs)}")
    for skipped_log in skipped_logs:
        print(f"  [INFO] Skipped: {skipped_log}")
    print(f"[INFO] Complete writing {len(all_metrics_rows)} rows to {out_metrics_csv}")
    print(f"[INFO] Complete writing {len(all_pgc_dist_rows)} rows to {out_pgc_dist_csv}")


if __name__ == "__main__":
    main()
