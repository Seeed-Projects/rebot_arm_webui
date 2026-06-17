#!/usr/bin/env bash
set -euo pipefail

INSTALL_DIR="${1:-${REBOT_ARM_WEBUI_DIR:-$HOME/robot-arm-webui}}"
SERVICE_NAME="${REBOT_ARM_WEBUI_SERVICE_NAME:-rebot-arm-webui}"
SERVICE_USER="${REBOT_ARM_WEBUI_SERVICE_USER:-${SUDO_USER:-$(id -un)}}"
SERVICE_GROUP="${REBOT_ARM_WEBUI_SERVICE_GROUP:-$(id -gn "$SERVICE_USER")}"
SERVICE_PORT="${REBOT_ARM_WEBUI_PORT:-8000}"
SERVICE_HOME="${REBOT_ARM_WEBUI_SERVICE_HOME:-}"
SERVICE_PATH="/etc/systemd/system/${SERVICE_NAME}.service"

if [ -z "$SERVICE_HOME" ] && command -v getent >/dev/null 2>&1; then
  SERVICE_HOME="$(getent passwd "$SERVICE_USER" | cut -d: -f6)"
fi

if [ -z "$SERVICE_HOME" ] && [ "$SERVICE_USER" = "$(id -un)" ]; then
  SERVICE_HOME="$HOME"
fi

if [ -z "$SERVICE_HOME" ]; then
  echo "Failed to resolve home directory for user: $SERVICE_USER" >&2
  exit 1
fi

run_as_root() {
  if [ "$(id -u)" -eq 0 ]; then
    "$@"
    return
  fi

  if command -v sudo >/dev/null 2>&1; then
    sudo "$@"
    return
  fi

  echo "Root privileges are required to install the systemd service." >&2
  exit 1
}

mkdir -p "$INSTALL_DIR"
TMP_FILE="$(mktemp)"
trap 'rm -f "$TMP_FILE"' EXIT

cat >"$TMP_FILE" <<EOF
[Unit]
Description=REBOT ARM WEBUI
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=$SERVICE_USER
Group=$SERVICE_GROUP
WorkingDirectory=$INSTALL_DIR
Environment=HOME=$SERVICE_HOME
Environment=PORT=$SERVICE_PORT
Environment=PYTHONUNBUFFERED=1
ExecStart=/usr/bin/env bash $INSTALL_DIR/scripts/start.sh
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF

run_as_root install -m 0644 "$TMP_FILE" "$SERVICE_PATH"
run_as_root systemctl daemon-reload
run_as_root systemctl enable "$SERVICE_NAME"
run_as_root systemctl restart "$SERVICE_NAME"

echo "Installed systemd service at $SERVICE_PATH"
echo "Service user: $SERVICE_USER"
echo "Service port: $SERVICE_PORT"
