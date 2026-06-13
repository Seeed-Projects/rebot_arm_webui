# rebot-arm-webui

`rebot-arm-webui` 是面向 `reBot Arm B601` 的一体化控制界面。当前版本将前端页面、URDF 仿真、串口检测、机械臂连接、夹爪控制和本地 HTTP 服务放在同一个仓库里，便于直接发布和部署。

## V0.1

`V0.1` 包含以下能力：

- 中文 / English 丝滑切换
- 机械臂接入检测与连接引导
- 已连接状态下的断开连接按钮
- 连接成功后立即使能机械臂
- 读取真机绝对关节位置
- 夹爪打开 / 闭合控制
- 诊断页维护提示

## 环境要求

- Linux 设备
- Python `3.10` 到 `3.12`
- Node.js `18+`
- `npm`
- `curl`
- 可访问机械臂串口，例如 `/dev/ttyACM0`

## 仓库结构

```text
rebot-arm-webui/
├─ app.js
├─ index.html
├─ kinematics_server.py
├─ package.json
├─ public/
├─ scripts/
│  ├─ bootstrap.sh
│  ├─ install.sh
│  └─ start.sh
└─ vendor/
   └─ reBotArm_control_py/
```

## 本地安装

```bash
git clone https://github.com/Seeed-Projects/rebot_arm_webui.git
cd rebot_arm_webui
./scripts/install.sh
```

安装完成后启动：

```bash
./scripts/start.sh
```

默认访问地址：

```text
http://127.0.0.1:8000/
```

## 一键安装命令

发布到 GitHub 后，可直接使用：

```bash
curl -fsSL https://raw.githubusercontent.com/Seeed-Projects/rebot_arm_webui/v0.1/scripts/bootstrap.sh | bash
```

如果使用自定义仓库地址，也可以：

```bash
REBOT_ARM_WEBUI_REPO_URL=git@github.com:Seeed-Projects/rebot_arm_webui.git \
bash -c "$(curl -fsSL https://raw.githubusercontent.com/Seeed-Projects/rebot_arm_webui/v0.1/scripts/bootstrap.sh)"
```

## 开发命令

安装依赖：

```bash
./scripts/install.sh
```

启动一体化服务：

```bash
npm run start
```

切换到 `8001` 端口：

```bash
PORT=8001 npm run start:8001
```

前端构建：

```bash
npm run build
```

## 设备权限

如果串口权限不足，可执行：

```bash
sudo chmod 666 /dev/ttyACM*
```

## 说明

- Python 依赖由仓库根目录的 `pyproject.toml` 管理。
- 机械臂驱动和配置已收敛到 `vendor/reBotArm_control_py`。
- `kinematics_server.py` 默认优先加载仓库内的 vendor 目录，也兼容旧的外部路径。
