#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

resolve_uv() {
  if command -v uv >/dev/null 2>&1; then
    command -v uv
    return 0
  fi
  if [ -x "$HOME/.local/bin/uv" ]; then
    printf '%s\n' "$HOME/.local/bin/uv"
    return 0
  fi
  return 1
}

if ! UV_BIN="$(resolve_uv)"; then
  echo "uv is required. Run ./scripts/install.sh first." >&2
  exit 1
fi

cd "$ROOT"
exec "$UV_BIN" run python kinematics_server.py
