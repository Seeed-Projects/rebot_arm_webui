#!/usr/bin/env bash
set -euo pipefail

REPO_URL="${REBOT_ARM_WEBUI_REPO_URL:-https://github.com/Seeed-Projects/rebot_arm_webui.git}"
INSTALL_DIR="${REBOT_ARM_WEBUI_DIR:-$HOME/robot-arm-webui}"
REBOT_ARM_WEBUI_REF="${REBOT_ARM_WEBUI_REF:-v0.2.1}"
REBOT_ARM_WEBUI_SERVICE_NAME="${REBOT_ARM_WEBUI_SERVICE_NAME:-rebot-arm-webui}"
REBOT_ARM_WEBUI_PORT="${REBOT_ARM_WEBUI_PORT:-8000}"

if [ -e "$INSTALL_DIR/.git" ]; then
  git -C "$INSTALL_DIR" fetch --tags
  git -C "$INSTALL_DIR" checkout "$REBOT_ARM_WEBUI_REF"
else
  git clone "$REPO_URL" "$INSTALL_DIR"
  git -C "$INSTALL_DIR" checkout "$REBOT_ARM_WEBUI_REF"
fi

"$INSTALL_DIR/scripts/install.sh"

REBOT_ARM_WEBUI_SERVICE_NAME="$REBOT_ARM_WEBUI_SERVICE_NAME" \
REBOT_ARM_WEBUI_PORT="$REBOT_ARM_WEBUI_PORT" \
"$INSTALL_DIR/scripts/setup-systemd.sh" "$INSTALL_DIR"

echo "REBOT ARM WEBUI is running as systemd service: $REBOT_ARM_WEBUI_SERVICE_NAME"
echo "Check status with: systemctl status $REBOT_ARM_WEBUI_SERVICE_NAME"
echo "Open: http://127.0.0.1:${REBOT_ARM_WEBUI_PORT}/"
