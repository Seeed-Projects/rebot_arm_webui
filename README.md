# REBOT ARM WEBUI

`REBOT ARM WEBUI` is an integrated control interface for the `reBot Arm B601`. The project combines the Web UI, URDF visualization, serial-device detection, arm connection flow, gripper control, diagnostics, and the local HTTP service in a single repository for straightforward deployment.

## V0.2 Highlights

- Smooth Chinese and English language switching
- Arm presence detection before connection, with guided connection prompts
- One-click disconnect after a successful connection
- Immediate arm enabling after the connection is established
- Real arm absolute joint-position sync
- Gripper open and close controls
- Diagnostics page with per-motor tabs, zero recalibration, and duplicate motor filtering by `feedback_id`
- Application Center with curated reBot Arm project links

## Requirements

- Linux device
- Python `3.10` to `3.12`
- Node.js `18+`
- `npm`
- `curl`
- Access to the robot arm serial interface, for example `/dev/ttyACM0`

## Repository Layout

```text
rebot-arm-webui/
|- app.js
|- index.html
|- kinematics_server.py
|- package.json
|- public/
|- scripts/
|  |- bootstrap.sh
|  |- install.sh
|  `- start.sh
`- vendor/
   `- reBotArm_control_py/
```

## Installation

```bash
git clone https://github.com/Seeed-Projects/rebot_arm_webui.git
cd rebot_arm_webui
./scripts/install.sh
```

Start the service after installation:

```bash
./scripts/start.sh
```

Default URL:

```text
http://127.0.0.1:8000/
```

## One-Line Install

Install the published `v0.2` version directly from GitHub:

```bash
curl -fsSL https://raw.githubusercontent.com/Seeed-Projects/rebot_arm_webui/v0.2/scripts/bootstrap.sh | bash
```

To install from a custom fork or mirror:

```bash
REBOT_ARM_WEBUI_REPO_URL=git@github.com:Seeed-Projects/rebot_arm_webui.git \
bash -c "$(curl -fsSL https://raw.githubusercontent.com/Seeed-Projects/rebot_arm_webui/v0.2/scripts/bootstrap.sh)"
```

## Development

Install dependencies:

```bash
./scripts/install.sh
```

Start the integrated service:

```bash
npm run start
```

Start on port `8001`:

```bash
PORT=8001 npm run start:8001
```

Build the frontend:

```bash
npm run build
```

## Device Permissions

If the serial device permissions are insufficient:

```bash
sudo chmod 666 /dev/ttyACM*
```

## Notes

- Python dependencies are managed by the repository-level `pyproject.toml`.
- The robot arm driver and configuration are consolidated under `vendor/reBotArm_control_py`.
- `kinematics_server.py` loads the in-repo vendor directory by default and remains compatible with older external paths.
