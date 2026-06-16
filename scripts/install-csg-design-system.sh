#!/usr/bin/env bash
set -euo pipefail

# Local adapter for Rida2000/csg-design-system
# Installs DESIGN.md + assets, then writes tool-specific project guidance
# for Claude Code, Cursor, Codex, OpenCode, and Crush.

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SOURCE_REPO="${CSG_DESIGN_SYSTEM_SOURCE:-D:/Code/csg-design-system-temp}"

normalize_source_path() {
  local path="$1"

  if [ -d "$path" ]; then
    printf '%s\n' "$path"
    return 0
  fi

  if [[ "$path" =~ ^([A-Za-z]):[\\/](.*)$ ]]; then
    local drive="${BASH_REMATCH[1],,}"
    local rest="${BASH_REMATCH[2]//\\//}"
    printf '/mnt/%s/%s\n' "$drive" "$rest"
    return 0
  fi

  printf '%s\n' "$path"
}

SOURCE_REPO="$(normalize_source_path "$SOURCE_REPO")"

if [ ! -d "$SOURCE_REPO" ]; then
  echo "CSG design system source not found: $SOURCE_REPO" >&2
  echo "Set CSG_DESIGN_SYSTEM_SOURCE to a local checkout of Rida2000/csg-design-system." >&2
  exit 1
fi

copy_file() {
  local src="$1"
  local dest="$2"
  mkdir -p "$(dirname "$dest")"
  cp "$src" "$dest"
}

install_base() {
  echo "Installing DESIGN.md and logo assets..."
  copy_file "$SOURCE_REPO/DESIGN.md" "$ROOT/DESIGN.md"

  mkdir -p "$ROOT/assets/logo"
  local logos=(
    "sensecraft-duel.svg"
    "sensecraft-mono-bright.svg"
    "sensecraft-mono-dark.svg"
    "sensecraft-mono-primary.svg"
    "sensecraft-duel.png"
    "sensecraft-mono-bright.png"
    "sensecraft-mono-dark.png"
    "sensecraft-mono-primary.png"
  )
  for name in "${logos[@]}"; do
    copy_file "$SOURCE_REPO/assets/logo/$name" "$ROOT/assets/logo/$name"
  done
}

install_claude() {
  echo "Installing Claude Code agents..."
  local claude_dir="$HOME/.claude/agents"
  mkdir -p "$claude_dir"
  local agents=(
    "csg-maintenance.md"
    "csg-component-builder.md"
    "csg-design-reviewer.md"
    "csg-figma-sync.md"
    "csg-tokenizer.md"
    "design-bridge.md"
    "csg-frontend-developer.md"
    "csg-ui-designer.md"
  )
  for name in "${agents[@]}"; do
    copy_file "$SOURCE_REPO/agents/$name" "$claude_dir/$name"
  done
}

install_cursor() {
  echo "Installing Cursor rules..."
  copy_file "$SOURCE_REPO/.cursorrules" "$ROOT/.cursorrules"

  local rules=(
    "csg-component-builder.mdc"
    "csg-design-reviewer.mdc"
    "csg-maintenance.mdc"
    "csg-figma-sync.mdc"
  )
  for name in "${rules[@]}"; do
    copy_file "$SOURCE_REPO/cursor/$name" "$ROOT/.cursor/rules/$name"
  done
}

install_codex() {
  echo "Installing Codex project instructions..."
  copy_file "$SOURCE_REPO/AGENTS.md" "$ROOT/AGENTS.md"
}

install_opencode() {
  echo "Installing OpenCode project instructions..."
  copy_file "$SOURCE_REPO/AGENTS.md" "$ROOT/.opencode/AGENTS.md"
}

install_crush() {
  echo "Installing Crush project instructions..."
  copy_file "$SOURCE_REPO/AGENTS.md" "$ROOT/.crush/AGENTS.md"
}

choice="${CHOICE:-all}"

cd "$ROOT"
install_base

case "$choice" in
  claude)
    install_claude
    ;;
  cursor)
    install_cursor
    ;;
  codex)
    install_codex
    ;;
  opencode)
    install_opencode
    ;;
  crush)
    install_crush
    ;;
  all)
    install_claude
    install_cursor
    install_codex
    install_opencode
    install_crush
    ;;
  *)
    echo "Unknown CHOICE: $choice" >&2
    echo "Use one of: claude, cursor, codex, opencode, crush, all" >&2
    exit 1
    ;;
esac

echo "CSG design system install complete."
