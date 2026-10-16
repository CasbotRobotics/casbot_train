#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
exec uv run --no-sync train Casbot02-Loco-Teacher-Flat "$@"
