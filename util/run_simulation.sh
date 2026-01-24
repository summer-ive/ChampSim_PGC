#!bin/bash
set -euo pipefail

# 並列ジョブ数
NJOBS=126

# トレースとログのディレクトリ
TRACES_DIR="../traces/DPC-3"
LOG_DIR="../log/no_prefetcher"
CMD="--warmup-instructions 200000000 --simulation-instructions 500000000"

# 出力先ディレクトリを作成
mkdir -p "$LOG_DIR"

# *.trace を見つけて xargs で並列実行
find "$TRACES_DIR" -name '*.xz' | \
xargs -n1 -P"$NJOBS" -I{} bash -c '
  set -euo pipefail
  TRACE="$1"
  BASENAME=$(basename "$TRACE" .xz)
  echo "[$(date +%T)] $$ Running trace: $BASENAME"
  nice -n 6 ../bin/champsim '"$CMD"' "$TRACE" > "'"$LOG_DIR"'/$BASENAME.log" 2>&1
' _ {}