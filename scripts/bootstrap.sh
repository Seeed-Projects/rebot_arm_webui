#!/usr/bin/env bash
set -euo pipefail

REPO_URL="${REBOT_ARM_WEBUI_REPO_URL:-https://github.com/Seeed-Projects/rebot_arm_webui.git}"
INSTALL_DIR="${REBOT_ARM_WEBUI_DIR:-$HOME/robot-arm-webui}"

if [ -e "$INSTALL_DIR/.git" ]; then
  git -C "$INSTALL_DIR" fetch --tags
  git -C "$INSTALL_DIR" checkout v0.1
else
  git clone "$REPO_URL" "$INSTALL_DIR"
  git -C "$INSTALL_DIR" checkout v0.1
fi

"$INSTALL_DIR/scripts/install.sh"
