#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
if [[ $# -lt 1 ]]; then
  echo '用法: bash play.sh logs/rsl_rl/casbot02_loco_teacher/<run>/model_20000.pt [--viewer native|viser]'
  exit 2
fi
CHECKPOINT_PATH="$1"
shift
if [[ ! -f "$CHECKPOINT_PATH" ]]; then
  echo "找不到检查点: $CHECKPOINT_PATH"
  exit 2
fi
exec uv run --no-sync play Casbot02-Loco-Teacher-Flat --checkpoint-file "$CHECKPOINT_PATH" --num-envs 1 "$@"
