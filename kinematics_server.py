from __future__ import annotations

import json
import math
import mimetypes
import os
import random
import platform
import shutil
import subprocess
import sys
import threading
import time
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote, urlparse

import numpy as np
import pinocchio as pin
import yaml


ROOT = Path(__file__).resolve().parent
DIST = ROOT / "dist"
PUBLIC = ROOT / "public"


def _resolve_rebot_root() -> Path:
    env_repo = os.getenv("REBOTARM_REPO")
    candidates = []
    if env_repo:
        candidates.append(Path(env_repo).expanduser())
    candidates.extend(
        [
            ROOT / "vendor" / "reBotArm_control_py",
            ROOT.parent / "reBotArm_control_py",
            ROOT.parent / "rebot_arm_service" / "src" / "rebot_arm_service" / "vendor" / "reBotArm_control_py",
        ]
    )
    for candidate in candidates:
        candidate = candidate.resolve()
        if (candidate / "reBotArm_control_py" / "__init__.py").exists():
            return candidate
    return candidates[0].expanduser().resolve() if candidates else (ROOT.parent / "reBotArm_control_py").resolve()


REBOT_ROOT = _resolve_rebot_root()
ARM_CFG = REBOT_ROOT / "config" / "arm.yaml"
URDF_ROOT = PUBLIC / "assets" / "urdf" / "reBot-DevArm_fixend_description"
URDF_PATH = URDF_ROOT / "urdf" / "reBot-DevArm_fixend.urdf"
PACKAGE_ALIAS = "reBot-DevArm_description_fixend"
JOINT_NAME_MAP = {
    "joint1": "joint1",
    "joint2": "joint2",
    "joint3": "join3",
    "joint4": "joint4",
    "joint5": "joint5",
    "joint6": "joint6",
}

if REBOT_ROOT.exists():
    sys.path.insert(0, str(REBOT_ROOT))

try:
    from reBotArm_control_py.actuator.arm import RobotArm
except Exception as exc:
    RobotArm = None
    ROBOT_ARM_IMPORT_ERROR = str(exc)
else:
    ROBOT_ARM_IMPORT_ERROR = None

try:
    from motorbridge import Mode
except Exception as exc:
    Mode = None
    MOTORBRIDGE_IMPORT_ERROR = str(exc)
else:
    MOTORBRIDGE_IMPORT_ERROR = None


def _load_arm_channel() -> str:
    try:
        data = yaml.safe_load(ARM_CFG.read_text()) or {}
    except Exception:
        return "/dev/ttyACM0"
    return str(data.get("channel") or "/dev/ttyACM0")


ARM_CHANNEL = _load_arm_channel()
GRIPPER_CFG = REBOT_ROOT / "config" / "gripper.yaml"


def _env_float(name: str, default: float) -> float:
    raw = os.getenv(name)
    if raw is None:
        return default
    try:
        return float(raw)
    except ValueError:
        return default


GRIPPER_OPEN_POS_RAD = _env_float("GRIPPER_OPEN_POS_RAD", -5.5)
GRIPPER_CLOSE_POS_RAD = _env_float("GRIPPER_CLOSE_POS_RAD", 0.0)
GRIPPER_VLIM = _env_float("GRIPPER_VLIM", 3.0)
GRIPPER_SETTLE_DELAY_S = _env_float("GRIPPER_SETTLE_DELAY_S", 1.0)
GRIPPER_POSITION_TOLERANCE_RAD = _env_float("GRIPPER_POSITION_TOLERANCE_RAD", 0.18)
GRIPPER_MAX_WAIT_S = _env_float("GRIPPER_MAX_WAIT_S", 1.8)


def _scan_interfaces() -> tuple[bool, list[str], str]:
    system = platform.system().lower()
    if system == "linux":
        devices = sorted(str(path) for path in Path("/dev").glob("ttyACM*"))
        if devices:
            return True, devices, "检测到 Linux ttyACM 设备"
        return False, [], "未检测到 /dev/ttyACM* 设备"
    return False, [], f"当前系统 {platform.system()} 暂不执行真实机械臂自动扫描"


def _device_permission_status(devices: list[str]) -> tuple[bool, str | None]:
    blocked = [device for device in devices if not os.access(device, os.R_OK | os.W_OK)]
    if not blocked:
        return True, None
    joined = ", ".join(blocked)
    return False, f"串口权限不足: {joined}，请执行 sudo chmod 666 /dev/ttyACM* 后重试"


def _chmod_serial_devices(devices: list[str]) -> tuple[bool, str]:
    targets = [device for device in devices if Path(device).name.startswith("ttyACM")]
    if not targets:
        return False, "未检测到 /dev/ttyACM* 设备"

    for target in targets:
        try:
            os.chmod(target, 0o666)
        except PermissionError:
            break
        except OSError:
            break
    else:
        ok, _ = _device_permission_status(targets)
        if ok:
            return True, f"已配置串口权限: {', '.join(targets)}"

    attempts = []
    if shutil.which("sudo"):
        attempts.append(("sudo", ["sudo", "-n", "chmod", "666", *targets], 5))
    if shutil.which("pkexec"):
        attempts.append(("pkexec", ["pkexec", "chmod", "666", *targets], 20))

    last_detail = ""
    for name, command, timeout in attempts:
        try:
            completed = subprocess.run(command, capture_output=True, text=True, timeout=timeout, check=False)
        except subprocess.TimeoutExpired:
            last_detail = f"{name} 授权超时"
            continue

        ok, _ = _device_permission_status(targets)
        if completed.returncode == 0 and ok:
            return True, f"已配置串口权限: {', '.join(targets)}"

        detail = (completed.stderr or completed.stdout or "").strip()
        if detail:
            last_detail = detail

    manual = "请在终端执行: sudo chmod 666 /dev/ttyACM*"
    if last_detail:
        return False, f"一键配置需要系统授权，当前未完成。{manual}"
    return False, f"一键配置需要系统授权，当前未完成。{manual}"


def _read_binary_stl_bounds(path: Path) -> tuple[np.ndarray, np.ndarray]:
    raw = path.read_bytes()
    faces = int.from_bytes(raw[80:84], "little")
    vertices = []
    offset = 84
    for _ in range(faces):
        offset += 12
        for _ in range(3):
            vertices.append(np.frombuffer(raw[offset : offset + 12], dtype="<f4").astype(np.float64))
            offset += 12
        offset += 2
    arr = np.asarray(vertices, dtype=np.float64)
    return arr.min(axis=0), arr.max(axis=0)


def _compute_tcp_offset() -> np.ndarray:
    # TCP is the gripper fingertip center in the end_link mesh frame.
    # In this mesh the fingertip is the +X end; the -X end is the gripper body/motor side.
    mesh_min, mesh_max = _read_binary_stl_bounds(URDF_ROOT / "meshes" / "end_link.STL")
    return np.array(
        [
            mesh_max[0],
            0.5 * (mesh_min[1] + mesh_max[1]),
            0.5 * (mesh_min[2] + mesh_max[2]),
        ],
        dtype=np.float64,
    )


MODEL = pin.buildModelFromUrdf(str(URDF_PATH))
END_FRAME_ID = MODEL.getFrameId("end_link")
TCP_OFFSET = _compute_tcp_offset()


def _json_response(handler: SimpleHTTPRequestHandler, payload: dict, status: int = 200) -> None:
    body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
    handler.send_response(status)
    handler.send_header("Content-Type", "application/json; charset=utf-8")
    handler.send_header("Content-Length", str(len(body)))
    handler.end_headers()
    handler.wfile.write(body)


def _read_json_body(handler: SimpleHTTPRequestHandler) -> dict:
    size = int(handler.headers.get("Content-Length") or "0")
    if size <= 0:
        return {}
    return json.loads(handler.rfile.read(size).decode("utf-8"))


def _clamp_config(q: np.ndarray) -> np.ndarray:
    lo = np.asarray(MODEL.lowerPositionLimit, dtype=np.float64)
    hi = np.asarray(MODEL.upperPositionLimit, dtype=np.float64)
    lo = np.where(np.isfinite(lo), lo, -math.pi)
    hi = np.where(np.isfinite(hi), hi, math.pi)
    return np.minimum(np.maximum(q, lo), hi)


def _tcp_pose_from_q(q: np.ndarray) -> pin.SE3:
    data = MODEL.createData()
    pin.forwardKinematics(MODEL, data, q)
    pin.updateFramePlacements(MODEL, data)
    end_pose = data.oMf[END_FRAME_ID]
    return end_pose * pin.SE3(np.eye(3), TCP_OFFSET)


def _fk(q: np.ndarray) -> dict:
    tcp_pose = _tcp_pose_from_q(q)
    return {
        "ok": True,
        "joints_rad": q.tolist(),
        "end_effector": {
            "position_m": tcp_pose.translation.tolist(),
            "rpy_rad": pin.rpy.matrixToRpy(tcp_pose.rotation).tolist(),
        },
    }


def _ik_error(data: pin.Data, q: np.ndarray, target_tcp: pin.SE3) -> tuple[float, np.ndarray]:
    pin.forwardKinematics(MODEL, data, q)
    pin.updateFramePlacements(MODEL, data)
    tcp_pose = data.oMf[END_FRAME_ID] * pin.SE3(np.eye(3), TCP_OFFSET)
    err = pin.log6(tcp_pose.inverse() * target_tcp).vector
    return float(np.linalg.norm(err)), err


def _solve_ik(target_tcp: pin.SE3, seed: np.ndarray, max_iter: int = 700) -> tuple[np.ndarray, bool, float, int]:
    data = MODEL.createData()
    q = _clamp_config(seed.copy())
    prev_err, err = _ik_error(data, q, target_tcp)

    for iteration in range(max_iter):
        if prev_err < 1e-4:
            return q, True, prev_err, iteration

        pin.computeJointJacobians(MODEL, data, q)
        pin.updateFramePlacements(MODEL, data)
        end_pose = data.oMf[END_FRAME_ID]
        tcp_pose = end_pose * pin.SE3(np.eye(3), TCP_OFFSET)
        j_end = pin.getFrameJacobian(MODEL, data, END_FRAME_ID, pin.LOCAL)
        adjoint = pin.SE3(np.eye(3), TCP_OFFSET).inverse().action
        j_tcp = adjoint @ j_end

        lam = 1e-6 * max(1.0, prev_err * 10.0)
        jjt = j_tcp @ j_tcp.T
        jjt[np.arange(jjt.shape[0]), np.arange(jjt.shape[1])] += lam
        dq = 0.5 * j_tcp.T @ np.linalg.solve(jjt, err)

        alpha = 1.0
        improved = False
        for _ in range(5):
            q_new = _clamp_config(pin.integrate(MODEL, q, alpha * dq))
            new_err, new_vec = _ik_error(data, q_new, target_tcp)
            if new_err < prev_err:
                q, err, prev_err = q_new, new_vec, new_err
                improved = True
                break
            alpha *= 0.5
        if not improved and np.linalg.norm(dq) < 1e-8:
            break

    return q, False, prev_err, max_iter


def _solve_ik_with_retries(target_tcp: pin.SE3, seed: np.ndarray) -> tuple[np.ndarray, bool, float, int]:
    best_q, best_ok, best_error, best_iter = _solve_ik(target_tcp, seed)
    if best_ok:
        return best_q, best_ok, best_error, best_iter

    lo = np.asarray(MODEL.lowerPositionLimit, dtype=np.float64)
    hi = np.asarray(MODEL.upperPositionLimit, dtype=np.float64)
    for _ in range(8):
        q_seed = np.array(
            [
                random.uniform(
                    float(l) if np.isfinite(l) else -math.pi,
                    float(h) if np.isfinite(h) else math.pi,
                )
                for l, h in zip(lo, hi)
            ],
            dtype=np.float64,
        )
        q, ok, error, iterations = _solve_ik(target_tcp, q_seed)
        if error < best_error:
            best_q, best_ok, best_error, best_iter = q, ok, error, iterations
        if ok:
            break
    return best_q, best_ok, best_error, best_iter


class RealArmBridge:
    def __init__(self, scan_interval: float = 2.0) -> None:
        self._scan_interval = scan_interval
        self._lock = threading.RLock()
        self._arm = None
        self._gripper_motor = None
        self._gripper_controller = None
        self._gripper_vendor: str | None = None
        self._connected = False
        self._detected = False
        self._interface_present = False
        self._interfaces: list[str] = []
        self._active_channel = ARM_CHANNEL
        self._last_error = ROBOT_ARM_IMPORT_ERROR
        self._last_scan_at = 0.0
        self._motors: list[dict] = []
        self._permission_required = False
        self._permission_hint: str | None = None
        self._events: list[dict] = []
        self._monitoring = False
        self._zero_offset_raw: np.ndarray | None = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._scan_loop, name="real-arm-scan", daemon=True)
        self._thread.start()

    def _log_locked(self, step: str, message: str, **data) -> None:
        self._events.insert(
            0,
            {
                "time": time.strftime("%H:%M:%S"),
                "step": step,
                "message": message,
                "data": data,
            },
        )
        self._events = self._events[:120]

    def _soft_disconnect_locked(self) -> None:
        if self._arm is not None:
            try:
                self._stop_feedback_monitor_locked()
            except Exception as exc:
                self._last_error = str(exc)
            try:
                self._arm.disable(retries=5, poll_interval=0.05)
            except Exception as exc:
                self._last_error = str(exc)
        self._connected = False
        self._monitoring = False

    def _close_arm_locked(self) -> None:
        if self._arm is not None:
            try:
                self._arm.disconnect()
            except Exception as exc:
                self._last_error = str(exc)
        self._arm = None
        self._gripper_motor = None
        self._gripper_controller = None
        self._gripper_vendor = None
        self._connected = False
        self._monitoring = False

    def _ensure_arm_locked(self):
        if RobotArm is None:
            raise RuntimeError(f"RobotArm import failed: {ROBOT_ARM_IMPORT_ERROR}")
        if self._arm is None:
            self._arm = RobotArm(str(self._runtime_cfg_locked()))
        return self._arm

    def _read_stable_positions_locked(self, samples: int = 5, interval_s: float = 0.04) -> np.ndarray:
        arm = self._ensure_arm_locked()
        readings = []
        for _ in range(max(1, samples)):
            readings.append(np.asarray(arm.get_positions(request=True), dtype=np.float64))
            time.sleep(interval_s)
        return np.median(np.vstack(readings), axis=0)

    def _calibrate_zero_locked(self, reason: str) -> np.ndarray:
        zero = self._read_stable_positions_locked()
        self._zero_offset_raw = zero.copy()
        self._log_locked(
            "calibration",
            reason,
            zero_offset_raw=self._zero_offset_raw.tolist(),
        )
        return self._zero_offset_raw

    def _start_feedback_monitor_locked(self) -> None:
        arm = self._ensure_arm_locked()
        if arm.control_loop_active and self._monitoring:
            return
        if arm.control_loop_active:
            arm.stop_control_loop()

        def mit_monitor(ref, _dt):
            ref.mit(
                np.zeros(ref.num_joints),
                kp=np.zeros(ref.num_joints),
                kd=np.zeros(ref.num_joints),
                tau=np.zeros(ref.num_joints),
            )

        arm.start_control_loop(mit_monitor)
        self._monitoring = True
        self._log_locked("monitor", "已启动真机反馈监控循环")

    def _stop_feedback_monitor_locked(self) -> None:
        if self._arm is not None and self._arm.control_loop_active:
            self._arm.stop_control_loop()
        if self._monitoring:
            self._log_locked("monitor", "已停止真机反馈监控循环")
        self._monitoring = False

    def _set_active_channel_locked(self, channel: str) -> None:
        if channel != self._active_channel:
            self._close_arm_locked()
            self._active_channel = channel
            self._zero_offset_raw = None

    def _find_controller_for_vendor_locked(self, arm, vendor: str):
        ctrl_map = getattr(arm, "_ctrl_map", {})
        if isinstance(ctrl_map, dict):
            if vendor in ctrl_map:
                return ctrl_map[vendor]
            if ctrl_map:
                return next(iter(ctrl_map.values()))
        return None

    def _ensure_gripper_attached_locked(self) -> None:
        if self._gripper_motor is not None and self._gripper_controller is not None:
            return
        if self._arm is None:
            raise RuntimeError("真实机械臂未初始化，无法挂载夹爪")
        if not GRIPPER_CFG.is_file():
            raise RuntimeError(f"未找到夹爪配置文件: {GRIPPER_CFG}")

        data = yaml.safe_load(GRIPPER_CFG.read_text(encoding="utf-8")) or {}
        if not isinstance(data, dict):
            raise RuntimeError(f"夹爪配置格式无效: {GRIPPER_CFG}")
        gripper_list = data.get("gripper") or []
        if not gripper_list:
            raise RuntimeError(f"夹爪配置缺少 gripper 节点: {GRIPPER_CFG}")

        gripper = gripper_list[0] or {}
        vendor = str(gripper.get("vendor", "damiao")).lower()
        ctrl = self._find_controller_for_vendor_locked(self._arm, vendor)
        if ctrl is None:
            raise RuntimeError(f"未找到夹爪控制器: vendor={vendor!r}")

        motor_id = int(gripper.get("motor_id", 0x07))
        feedback_id = int(gripper.get("feedback_id", 0x17))
        model = str(gripper.get("model", "4310"))

        if vendor == "damiao":
            mot = ctrl.add_damiao_motor(motor_id, feedback_id, model)
        elif vendor == "myactuator":
            mot = ctrl.add_myactuator_motor(motor_id, feedback_id, model)
        elif vendor == "robstride":
            mot = ctrl.add_robstride_motor(motor_id, feedback_id, model)
        else:
            raise ValueError(f"Unsupported gripper vendor: {vendor}")

        self._gripper_motor = mot
        self._gripper_controller = ctrl
        self._gripper_vendor = vendor

    def _gripper_state_locked(self, request: bool = True) -> dict | None:
        if self._gripper_motor is None or self._gripper_controller is None:
            return None
        if request:
            try:
                self._gripper_motor.request_feedback()
            except Exception:
                pass
        try:
            self._gripper_controller.poll_feedback_once()
        except Exception:
            pass
        try:
            state = self._gripper_motor.get_state()
        except Exception:
            return None
        if state is None:
            return None
        return {
            "status_code": getattr(state, "status_code", None),
            "pos": float(getattr(state, "pos", 0.0)),
            "vel": float(getattr(state, "vel", 0.0)),
            "torq": float(getattr(state, "torq", 0.0)),
        }

    def _runtime_cfg_locked(self) -> Path:
        if self._active_channel == ARM_CHANNEL:
            return ARM_CFG
        data = yaml.safe_load(ARM_CFG.read_text()) or {}
        data["channel"] = self._active_channel
        runtime_cfg = ROOT / ".runtime_arm.yaml"
        runtime_cfg.write_text(yaml.safe_dump(data, allow_unicode=True), encoding="utf-8")
        return runtime_cfg

    def _scan_once_locked(self) -> None:
        self._interface_present, self._interfaces, interface_message = _scan_interfaces()
        self._set_active_channel_locked(self._interfaces[0] if self._interfaces else ARM_CHANNEL)
        if not self._interface_present:
            self._close_arm_locked()
            self._detected = False
            self._motors = []
            self._permission_required = False
            self._permission_hint = None
            self._last_error = interface_message
            self._last_scan_at = time.time()
            self._log_locked("scan", interface_message)
            return

        has_permission, permission_hint = _device_permission_status(self._interfaces)
        self._permission_required = not has_permission
        self._permission_hint = permission_hint
        if not has_permission:
            self._close_arm_locked()
            self._detected = False
            self._motors = []
            self._last_error = permission_hint
            self._last_scan_at = time.time()
            self._log_locked("permission", permission_hint or "串口权限不足", interfaces=self._interfaces)
            return

        arm = self._ensure_arm_locked()
        # Detection signal: actively request motor feedback and poll the Damiao serial bridge.
        # RobotArm.connect() is a no-op in the reference library; feedback is the real handshake.
        for _ in range(3):
            arm._request_and_poll()
            time.sleep(0.05)
        motors = []
        detected = False
        for joint in arm._joints:
            state = None
            try:
                state = arm._motor_map[joint.name].get_state()
            except Exception:
                state = None
            if state is not None:
                detected = True
                motors.append(
                    {
                        "name": joint.name,
                        "motor_id": joint.motor_id,
                        "feedback_id": joint.feedback_id,
                        "model": joint.model,
                        "status_code": getattr(state, "status_code", None),
                        "pos": float(getattr(state, "pos", 0.0)),
                        "vel": float(getattr(state, "vel", 0.0)),
                        "torq": float(getattr(state, "torq", 0.0)),
                    }
                )
        self._detected = detected
        self._motors = motors
        self._permission_required = False
        self._permission_hint = None
        self._last_error = None if detected else "未读取到电机反馈"
        self._last_scan_at = time.time()
        self._log_locked(
            "feedback",
            f"读取到 {len(motors)} 个电机反馈" if detected else "未读取到电机反馈",
            motors=motors,
        )

    def _scan_loop(self) -> None:
        while not self._stop.is_set():
            with self._lock:
                if not self._connected:
                    try:
                        self._scan_once_locked()
                    except Exception as exc:
                        self._detected = False
                        self._motors = []
                        if "Permission denied" in str(exc) or "permission" in str(exc).lower():
                            self._permission_required = True
                            self._permission_hint = "串口权限不足，请执行 sudo chmod 666 /dev/ttyACM* 后重试"
                        self._last_error = str(exc)
                        self._last_scan_at = time.time()
            self._stop.wait(self._scan_interval)

    def health(self) -> dict:
        with self._lock:
            return {
                "ok": True,
                "connected": self._connected,
                "arm_detected": self._detected,
                "interface_present": self._interface_present,
                "channel": self._active_channel,
                "interfaces": self._interfaces,
                "detected_motors": self._motors,
                "last_scan_at": self._last_scan_at,
                "scan_interval_s": self._scan_interval,
                "real_arm_available": RobotArm is not None,
                "gripper_available": GRIPPER_CFG.is_file() and Mode is not None,
                "gripper_attached": self._gripper_motor is not None and self._gripper_controller is not None,
                "permission_required": self._permission_required,
                "permission_hint": self._permission_hint,
                "error": self._last_error,
            }

    def logs(self) -> dict:
        with self._lock:
            return {"ok": True, "events": self._events}

    def connect(self, confirmed: bool = False) -> dict:
        with self._lock:
            try:
                self._scan_once_locked()
            except Exception as exc:
                self._detected = False
                self._motors = []
                if "Permission denied" in str(exc) or "permission" in str(exc).lower():
                    self._permission_required = True
                    self._permission_hint = "串口权限不足，请执行 sudo chmod 666 /dev/ttyACM* 后重试"
                self._last_error = str(exc)
                self._last_scan_at = time.time()
                return {**self.health(), "ok": False, "message": self._permission_hint or "无法连接真实机械臂"}
            if self._permission_required:
                return {**self.health(), "ok": False, "message": self._permission_hint or "串口权限不足"}
            if not self._detected:
                self._log_locked("connect", "未读取到电机反馈，无法连接真实机械臂")
                return {**self.health(), "ok": False, "message": "未读取到电机反馈，无法连接真实机械臂"}
            if not confirmed:
                self._log_locked("connect", "检测到电机反馈，等待用户确认连接", motors=len(self._motors))
                return {**self.health(), "ok": True, "requires_confirmation": True}
            arm = self._ensure_arm_locked()
            arm.connect()
            # Confirm one more time before marking the arm connected.
            self._scan_once_locked()
            if not self._detected:
                self._log_locked("connect", "连接前未读取到电机反馈")
                return {**self.health(), "ok": False, "message": "连接前未读取到电机反馈，请检查机械臂供电和通信线"}
            try:
                arm.enable(retries=5, poll_interval=0.05)
            except Exception as exc:
                self._last_error = str(exc)
                self._log_locked("connect", f"机械臂使能失败: {exc}")
                return {**self.health(), "ok": False, "message": f"机械臂使能失败: {exc}"}
            self._connected = True
            self._zero_offset_raw = None
            try:
                self._ensure_gripper_attached_locked()
            except Exception as exc:
                self._log_locked("gripper", f"夹爪挂载失败: {exc}")
            self._log_locked("connect", "真实机械臂已连接并完成使能", channel=self._active_channel)
            return {**self.health(), "ok": True, "message": "真实机械臂已连接并完成使能"}

    def configure_serial_permission(self) -> dict:
        with self._lock:
            interface_present, interfaces, interface_message = _scan_interfaces()
            self._interface_present = interface_present
            self._interfaces = interfaces
            self._set_active_channel_locked(interfaces[0] if interfaces else ARM_CHANNEL)
            if not interface_present:
                self._permission_required = False
                self._permission_hint = None
                self._last_error = interface_message
                self._last_scan_at = time.time()
                return {**self.health(), "ok": False, "message": interface_message}

            ok, message = _chmod_serial_devices(interfaces)
            self._last_scan_at = time.time()
            if not ok:
                self._permission_required = True
                self._permission_hint = "请手动执行 sudo chmod 666 /dev/ttyACM*"
                self._last_error = message
                return {**self.health(), "ok": False, "message": message}

            self._permission_required = False
            self._permission_hint = None
            self._last_error = None
            try:
                self._scan_once_locked()
            except Exception as exc:
                self._last_error = str(exc)
            return {**self.health(), "ok": True, "message": message}

    def disconnect(self) -> dict:
        with self._lock:
            self._soft_disconnect_locked()
            try:
                self._scan_once_locked()
            except Exception as exc:
                self._last_error = str(exc)
                self._detected = False
                self._motors = []
            self._log_locked("disconnect", "real arm disconnected")
            return self.health()

    def command_gripper(self, action: str) -> dict:
        with self._lock:
            action = str(action or "").lower()
            if action not in {"open", "close"}:
                return {**self.health(), "ok": False, "message": f"不支持的夹爪动作: {action}"}
            if not self._connected or self._arm is None:
                self._log_locked("gripper", "真实机械臂未连接，无法控制夹爪", action=action)
                return {**self.health(), "ok": False, "connected": False, "message": "真实机械臂未连接"}
            if Mode is None:
                message = f"夹爪驱动不可用: {MOTORBRIDGE_IMPORT_ERROR}"
                self._log_locked("gripper", message, action=action)
                return {**self.health(), "ok": False, "message": message}

            try:
                self._ensure_gripper_attached_locked()
                target_pos = GRIPPER_OPEN_POS_RAD if action == "open" else GRIPPER_CLOSE_POS_RAD
                if self._gripper_controller is None or self._gripper_motor is None:
                    raise RuntimeError("夹爪未挂载到当前控制器")
                self._gripper_controller.enable_all()
                self._gripper_motor.ensure_mode(Mode.POS_VEL, 1000)
                self._gripper_motor.send_pos_vel(float(target_pos), float(GRIPPER_VLIM))
                try:
                    self._gripper_controller.poll_feedback_once()
                except Exception:
                    pass
                time.sleep(min(0.2, GRIPPER_SETTLE_DELAY_S))
                gripper_state = self._wait_for_gripper_target_locked(target_pos)
            except Exception as exc:
                self._last_error = str(exc)
                message = f"夹爪控制失败: {exc}"
                self._log_locked("gripper", message, action=action)
                return {**self.health(), "ok": False, "message": message}

            message = "夹爪已打开" if action == "open" else "夹爪已闭合"
            self._log_locked(
                "gripper",
                message,
                action=action,
                target_pos_rad=target_pos,
                vlim=GRIPPER_VLIM,
                gripper_state=gripper_state,
            )
            result = {
                **self.health(),
                "ok": True,
                "action": action,
                "message": message,
                "target_pos_rad": float(target_pos),
                "vlim": float(GRIPPER_VLIM),
            }
            if gripper_state is not None:
                result["gripper_state"] = gripper_state
            return result

    def _wait_for_gripper_target_locked(self, target_pos: float) -> dict | None:
        deadline = time.time() + GRIPPER_MAX_WAIT_S
        last_state = None
        while time.time() < deadline:
            last_state = self._gripper_state_locked(request=True)
            if last_state is None:
                time.sleep(0.05)
                continue
            if abs(last_state["pos"] - float(target_pos)) <= GRIPPER_POSITION_TOLERANCE_RAD:
                return last_state
            if abs(last_state["vel"]) <= 0.06:
                return last_state
            time.sleep(0.05)
        return last_state

    def state(self) -> dict:
        with self._lock:
            payload = self.health()
            if not self._connected or self._arm is None:
                return payload
            try:
                pos = self._arm.get_positions(request=True)
                vel = self._arm.get_velocities(request=True)
                torq = self._arm.get_torques(request=True)
                payload.update(
                    {
                        "joints_rad": pos.tolist(),
                        "raw_joints_rad": pos.tolist(),
                        "velocities_rad_s": vel.tolist(),
                        "torques_nm": torq.tolist(),
                        "end_effector": _fk(pos)["end_effector"],
                    }
                )
                self._log_locked(
                    "state",
                    "已读取真机关节位置",
                    joints_rad=pos.tolist(),
                    raw_joints_rad=pos.tolist(),
                )
            except Exception as exc:
                payload.update({"ok": False, "error": str(exc)})
                self._log_locked("state", f"读取真机状态失败: {exc}")
            return payload

    def send_joints(self, q: np.ndarray, max_speed_rad_s: float = 0.22) -> dict:
        with self._lock:
            if not self._connected or self._arm is None:
                self._log_locked("move", "真实机械臂未连接，无法执行目标位置")
                return {"ok": False, "connected": False, "message": "真实机械臂未连接"}
            self._stop_feedback_monitor_locked()
            current = self._arm.get_positions(request=True)
            target = _clamp_config(np.asarray(q, dtype=np.float64).reshape(-1))
            max_speed = float(np.clip(max_speed_rad_s, 0.05, 0.35))
            delta = target - current
            duration = float(np.clip(np.max(np.abs(delta)) / max_speed if delta.size else 0.0, 1.5, 8.0))
            steps = max(20, int(duration / 0.05))
            slow_vlim = np.full(MODEL.nq, max_speed, dtype=np.float64)

            self._log_locked(
                "move",
                "准备执行目标位置",
                current_raw_joints_rad=current.tolist(),
                current_joints_rad=current.tolist(),
                target_joints_rad=target.tolist(),
                target_raw_joints_rad=target.tolist(),
                max_speed_rad_s=max_speed,
                duration_s=duration,
                steps=steps,
            )
            self._arm.enable(retries=5, poll_interval=0.05)
            self._arm.mode_pos_vel(vlim=slow_vlim, stabilize_delay=0.1)

            target_ref = {"q": current.copy()}

            def pos_vel_controller(ref, _dt):
                ref.pos_vel(target_ref["q"], vlim=slow_vlim)

            self._arm.start_control_loop(pos_vel_controller, rate=50.0)
            try:
                for index in range(1, steps + 1):
                    ratio = index / steps
                    blend = ratio * ratio * (3.0 - 2.0 * ratio)
                    target_ref["q"] = current + delta * blend
                    time.sleep(duration / steps)
                target_ref["q"] = target.copy()
                time.sleep(0.5)
            finally:
                if self._arm.control_loop_active:
                    self._arm.stop_control_loop()
                self._monitoring = False

            result = self.state()
            result.update(
                {
                    "ok": True,
                    "message": "目标位置已低速执行",
                    "command": {
                        "type": "joint_pos_vel_trajectory",
                        "target_joints_rad": target.tolist(),
                        "target_raw_joints_rad": target.tolist(),
                        "max_speed_rad_s": max_speed,
                        "duration_s": duration,
                        "steps": steps,
                    },
                }
            )
            self._log_locked("move", "目标位置执行完成", command=result["command"])
            return result


REAL_ARM = RealArmBridge(scan_interval=2.0)


class Handler(SimpleHTTPRequestHandler):
    def log_message(self, format: str, *args) -> None:
        print(f"[webui] {self.address_string()} {format % args}")

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        path = parsed.path

        if path == "/healthz":
            _json_response(self, REAL_ARM.health())
            return
        if path == "/state":
            _json_response(self, REAL_ARM.state())
            return
        if path == "/logs":
            _json_response(self, REAL_ARM.logs())
            return
        if path == "/sim/config":
            _json_response(
                self,
                {
                    "ok": True,
                    "urdf_url": "/assets/urdf/reBot-DevArm_fixend_description/urdf/reBot-DevArm_fixend.urdf",
                    "package_alias": PACKAGE_ALIAS,
                    "package_url": "/assets/urdf/reBot-DevArm_fixend_description",
                    "joint_name_map": JOINT_NAME_MAP,
                },
            )
            return
        if path == "/assets/urdf/model.urdf":
            self._send_file(URDF_PATH, "application/xml")
            return
        if path.startswith("/assets/urdf/reBot-DevArm_fixend_description/"):
            asset = unquote(path.split("/assets/urdf/reBot-DevArm_fixend_description/", 1)[1]).lstrip("/")
            self._send_file((URDF_ROOT / asset).resolve())
            return
        if path.startswith(f"/assets/urdf/package/{PACKAGE_ALIAS}/"):
            asset = unquote(path.split(f"/assets/urdf/package/{PACKAGE_ALIAS}/", 1)[1]).lstrip("/")
            self._send_file((URDF_ROOT / asset).resolve())
            return

        if path in {"/", "/webui"}:
            self._send_file(DIST / "index.html", "text/html")
            return

        candidate = (DIST / path.lstrip("/")).resolve()
        if str(candidate).startswith(str(DIST.resolve())) and candidate.is_file():
            self._send_file(candidate)
            return

        self.send_error(404, "not found")

    def do_POST(self) -> None:
        try:
            payload = _read_json_body(self)
            path = urlparse(self.path).path

            if path == "/sim/fk":
                q = np.asarray(payload.get("joints_rad", []), dtype=np.float64).reshape(-1)
                if q.shape != (MODEL.nq,):
                    raise ValueError(f"expected {MODEL.nq} joints, got {q.shape[0]}")
                _json_response(self, _fk(q))
                return

            if path == "/sim/ik":
                pos = np.array([payload["x"], payload["y"], payload["z"]], dtype=np.float64)
                rot = pin.rpy.rpyToMatrix(
                    float(payload.get("roll", 0.0)),
                    float(payload.get("pitch", 0.0)),
                    float(payload.get("yaw", 0.0)),
                )
                target_tcp = pin.SE3(rot, pos)
                seed = np.asarray(payload.get("seed_joints_rad") or np.zeros(MODEL.nq), dtype=np.float64).reshape(-1)
                if seed.shape != (MODEL.nq,):
                    seed = np.zeros(MODEL.nq, dtype=np.float64)
                q, ok, error, iterations = _solve_ik_with_retries(target_tcp, seed)
                result = _fk(q)
                result.update({"ok": ok, "iterations": iterations, "error": error})
                _json_response(self, result)
                return

            if path == "/connect":
                _json_response(self, REAL_ARM.connect(confirmed=bool(payload.get("confirm"))))
                return

            if path == "/disconnect":
                _json_response(self, REAL_ARM.disconnect())
                return

            if path == "/permissions/serial":
                _json_response(self, REAL_ARM.configure_serial_permission())
                return

            if path == "/gripper":
                _json_response(self, REAL_ARM.command_gripper(str(payload.get("action") or "")))
                return

            if path == "/move/joints":
                q = np.asarray(payload.get("joints_rad", []), dtype=np.float64).reshape(-1)
                if q.shape != (MODEL.nq,):
                    raise ValueError(f"expected {MODEL.nq} joints, got {q.shape[0]}")
                if REAL_ARM.health().get("connected"):
                    speed = float(payload.get("max_speed_rad_s", 0.22))
                    _json_response(self, REAL_ARM.send_joints(q, max_speed_rad_s=speed))
                else:
                    result = _fk(q)
                    result.update({"connected": False, "message": "simulation-only joint target"})
                    _json_response(self, result)
                return

            if path in {"/move/pose", "/home"}:
                _json_response(self, {"ok": True, "connected": REAL_ARM.health().get("connected", False), "message": "endpoint accepted"})
                return

            self.send_error(404, "not found")
        except Exception as exc:
            _json_response(self, {"ok": False, "detail": str(exc)}, status=400)

    def _send_file(self, path: Path, content_type: str | None = None) -> None:
        if not path.is_file():
            self.send_error(404, "file not found")
            return
        body = path.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", content_type or mimetypes.guess_type(str(path))[0] or "application/octet-stream")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def _server_host_port() -> tuple[str, int]:
    host = os.getenv("HOST") or "0.0.0.0"
    port_raw = os.getenv("PORT") or "8000"
    try:
        port = int(port_raw)
    except ValueError as exc:
        raise RuntimeError(f"invalid PORT: {port_raw}") from exc
    return host, port


if __name__ == "__main__":
    host, port = _server_host_port()
    try:
        server = ThreadingHTTPServer((host, port), Handler)
    except OSError as exc:
        if exc.errno == 98:
            print(
                f"port {port} is already in use. Stop the existing service or run PORT={port + 1} npm run start.",
                file=sys.stderr,
            )
        raise
    print(f"rebot arm webui backend: http://localhost:{port}/")
    server.serve_forever()
