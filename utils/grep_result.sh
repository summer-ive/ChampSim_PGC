#!bin/bash
set -euo pipefail

LOG_DIR="../logs/no_prefetcher"
OUT="spp_total_prefetches.txt"

: > "$OUT"
for f in "$LOG_DIR"/*.log; do
  # マッチ行を一つだけ取って sed で数字を抜く
  num=$(grep -m1 '^\[SPP\] total prefetches:' "$f" \
        | sed -E 's/.*total prefetches: *([0-9]+).*/\1/')
  echo "$num" >> "$OUT"
done

echo "-> extracted values saved in $OUT"

OUT="spp_page_cross_count.txt"

: > "$OUT"
for f in "$LOG_DIR"/*.log; do
  # マッチ行を一つだけ取って sed で数字を抜く
  num=$(grep -m1 '^\[SPP\] page-crossing count:' "$f" \
        | sed -E 's/.*page-crossing count: *([0-9]+).*/\1/')
  echo "$num" >> "$OUT"
done

echo "-> extracted values saved in $OUT"

OUT="spp_ipc.txt"

: > "$OUT"
# 各ログファイルから IPC 値を抜き出して書き込む
for f in "$LOG_DIR"/*.log; do
  # "CPU 0 cumulative IPC:" 行を探して 4 番目のフィールド（IPC 値）を取得
  ipc=$(grep -m1 '^CPU 0 cumulative IPC:' "$f" | awk '{print $5}')
  echo "$ipc" >> "$OUT"
done

echo "-> extracted values saved in $OUT"