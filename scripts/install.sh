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
  curl -fsSL https://astral.sh/uv/install.sh | sh
  UV_BIN="$(resolve_uv)"
fi

if ! command -v npm >/dev/null 2>&1; then
  echo "npm is required but was not found." >&2
  exit 1
fi

cd "$ROOT"
"$UV_BIN" sync
npm ci
npm run build

echo "Install complete. Start with: ./scripts/start.sh"
