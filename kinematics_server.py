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
from urllib.parse import parse_qs, unquote, urlparse

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
RS_CFG = REBOT_ROOT / "config" / "rebotarm_rs.yaml"
PUBLIC_URDF_ROOT = PUBLIC / "assets" / "urdf"


def _make_arm_profiles() -> dict[str, dict]:
    dm_urdf_root = PUBLIC_URDF_ROOT / "reBot-DevArm_fixend_description"
    rs_urdf_root = PUBLIC_URDF_ROOT / "reBot-DevArm-rs_asm-v3"
    return {
        "dm": {
            "arm_type": "dm",
            "label": "Damiao",
            "driver": "dm",
            "cfg_path": ARM_CFG,
            "channel_kind": "serial",
            "default_channel": "/dev/ttyACM0",
            "urdf_root": dm_urdf_root,
            "urdf_path": dm_urdf_root / "urdf" / "reBot-DevArm_fixend.urdf",
            "urdf_url": "/assets/urdf/reBot-DevArm_fixend_description/urdf/reBot-DevArm_fixend.urdf",
            "package_alias": "reBot-DevArm_description_fixend",
            "package_url": "/assets/urdf/reBot-DevArm_fixend_description",
            "end_effector_frame": "end_link",
            "tcp_mesh": dm_urdf_root / "meshes" / "end_link.STL",
            "tcp_axis": "max_x",
            "controlled_joints": 6,
            "joint_name_map": {
                "joint1": "joint1",
                "joint2": "joint2",
                "joint3": "join3",
                "joint4": "joint4",
                "joint5": "joint5",
                "joint6": "joint6",
            },
        },
        "rs": {
            "arm_type": "rs",
            "label": "RS",
            "driver": "rs",
            "cfg_path": RS_CFG,
            "channel_kind": "can",
            "default_channel": "can1",
            "urdf_root": rs_urdf_root,
            "urdf_path": rs_urdf_root / "urdf" / "00-arm-rs_asm-v3.urdf",
            "urdf_url": "/assets/urdf/reBot-DevArm-rs_asm-v3/urdf/00-arm-rs_asm-v3.urdf",
            "package_alias": "reBot-DevArm-rs_asm-v3",
            "package_url": "/assets/urdf/reBot-DevArm-rs_asm-v3",
            "end_effector_frame": "gripper_end",
            "tcp_mesh": rs_urdf_root / "meshes" / "gripper_end.STL",
            "tcp_axis": "min_x",
            "controlled_joints": 6,
            "joint_name_map": {
                "joint1": "joint1",
                "joint2": "joint2",
                "joint3": "joint3",
                "joint4": "joint4",
                "joint5": "joint5",
                "joint6": "joint6",
            },
        },
    }


ARM_PROFILES = _make_arm_profiles()
DEFAULT_ARM_TYPE = "rs" if os.getenv("REBOT_ARM_TYPE", "").lower() == "rs" else "dm"


def _normalize_arm_type(value: str | None) -> str:
    normalized = str(value or DEFAULT_ARM_TYPE).strip().lower()
    return normalized if normalized in ARM_PROFILES else DEFAULT_ARM_TYPE


def _profile_for_type(arm_type: str | None = None) -> dict:
    return ARM_PROFILES[_normalize_arm_type(arm_type)]

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
    from reBotArm_control_py.actuator.rebotarm import RebotArm
except Exception as exc:
    RebotArm = None
    REBOT_ARM_IMPORT_ERROR = str(exc)
else:
    REBOT_ARM_IMPORT_ERROR = None

try:
    from motorbridge import Mode
except Exception as exc:
    Mode = None
    MOTORBRIDGE_IMPORT_ERROR = str(exc)
else:
    MOTORBRIDGE_IMPORT_ERROR = None


def _load_profile_channel(profile: dict) -> str:
    try:
        data = yaml.safe_load(Path(profile["cfg_path"]).read_text()) or {}
    except Exception:
        return str(profile["default_channel"])
    return str(data.get("channel") or profile["default_channel"])


ARM_CHANNELS = {key: _load_profile_channel(profile) for key, profile in ARM_PROFILES.items()}
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


def _scan_interfaces(profile: dict | None = None) -> tuple[bool, list[str], str]:
    profile = profile or _profile_for_type()
    system = platform.system().lower()
    if system == "linux" and profile["channel_kind"] == "serial":
        devices = sorted(str(path) for path in Path("/dev").glob("ttyACM*"))
        if devices:
            return True, devices, "检测到 Linux ttyACM 设备"
        return False, [], "未检测到 /dev/ttyACM* 设备"
    if system == "linux" and profile["channel_kind"] == "can":
        configured = str(profile.get("channel") or profile["default_channel"])
        devices = sorted(path.name for path in Path("/sys/class/net").glob("can*"))
        if configured in devices:
            return True, [configured], f"检测到 CAN 接口 {configured}"
        if devices:
            return True, devices, f"检测到 CAN 接口: {', '.join(devices)}"
        return False, [], "未检测到 can* SocketCAN 接口"
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


def _compute_tcp_offset(profile: dict) -> np.ndarray:
    mesh_min, mesh_max = _read_binary_stl_bounds(Path(profile["tcp_mesh"]))
    x = mesh_max[0] if profile.get("tcp_axis") == "max_x" else mesh_min[0]
    return np.array(
        [
            x,
            0.5 * (mesh_min[1] + mesh_max[1]),
            0.5 * (mesh_min[2] + mesh_max[2]),
        ],
        dtype=np.float64,
    )


KINEMATICS_CACHE: dict[str, dict] = {}


def _kinematics(arm_type: str | None = None) -> dict:
    normalized = _normalize_arm_type(arm_type)
    if normalized not in KINEMATICS_CACHE:
        profile = _profile_for_type(normalized)
        model = pin.buildModelFromUrdf(str(profile["urdf_path"]))
        KINEMATICS_CACHE[normalized] = {
            "profile": profile,
            "model": model,
            "end_frame_id": model.getFrameId(profile["end_effector_frame"]),
            "tcp_offset": _compute_tcp_offset(profile),
            "controlled_joints": int(profile["controlled_joints"]),
        }
    return KINEMATICS_CACHE[normalized]


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


def _to_model_q(q: np.ndarray, kin: dict) -> np.ndarray:
    model = kin["model"]
    controlled = kin["controlled_joints"]
    values = np.asarray(q, dtype=np.float64).reshape(-1)
    if values.shape == (model.nq,):
        return values
    if values.shape != (controlled,):
        raise ValueError(f"expected {controlled} joints, got {values.shape[0]}")
    full_q = np.zeros(model.nq, dtype=np.float64)
    full_q[:controlled] = values
    return full_q


def _controlled_q(q: np.ndarray, kin: dict) -> np.ndarray:
    return np.asarray(q, dtype=np.float64).reshape(-1)[: kin["controlled_joints"]]


def _clamp_config(q: np.ndarray, kin: dict) -> np.ndarray:
    model = kin["model"]
    lo = np.asarray(model.lowerPositionLimit, dtype=np.float64)
    hi = np.asarray(model.upperPositionLimit, dtype=np.float64)
    lo = np.where(np.isfinite(lo), lo, -math.pi)
    hi = np.where(np.isfinite(hi), hi, math.pi)
    return np.minimum(np.maximum(q, lo), hi)


def _tcp_pose_from_q(q: np.ndarray, kin: dict) -> pin.SE3:
    model = kin["model"]
    data = model.createData()
    full_q = _to_model_q(q, kin)
    pin.forwardKinematics(model, data, full_q)
    pin.updateFramePlacements(model, data)
    end_pose = data.oMf[kin["end_frame_id"]]
    return end_pose * pin.SE3(np.eye(3), kin["tcp_offset"])


def _fk(q: np.ndarray, arm_type: str | None = None) -> dict:
    kin = _kinematics(arm_type)
    full_q = _to_model_q(q, kin)
    tcp_pose = _tcp_pose_from_q(full_q, kin)
    return {
        "ok": True,
        "arm_type": kin["profile"]["arm_type"],
        "joints_rad": _controlled_q(full_q, kin).tolist(),
        "end_effector": {
            "position_m": tcp_pose.translation.tolist(),
            "rpy_rad": pin.rpy.matrixToRpy(tcp_pose.rotation).tolist(),
        },
    }


def _ik_error(kin: dict, data: pin.Data, q: np.ndarray, target_tcp: pin.SE3) -> tuple[float, np.ndarray]:
    model = kin["model"]
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    tcp_pose = data.oMf[kin["end_frame_id"]] * pin.SE3(np.eye(3), kin["tcp_offset"])
    err = pin.log6(tcp_pose.inverse() * target_tcp).vector
    return float(np.linalg.norm(err)), err


def _solve_ik(kin: dict, target_tcp: pin.SE3, seed: np.ndarray, max_iter: int = 700) -> tuple[np.ndarray, bool, float, int]:
    model = kin["model"]
    data = model.createData()
    q = _clamp_config(_to_model_q(seed, kin).copy(), kin)
    prev_err, err = _ik_error(kin, data, q, target_tcp)

    for iteration in range(max_iter):
        if prev_err < 1e-4:
            return q, True, prev_err, iteration

        pin.computeJointJacobians(model, data, q)
        pin.updateFramePlacements(model, data)
        j_end = pin.getFrameJacobian(model, data, kin["end_frame_id"], pin.LOCAL)
        adjoint = pin.SE3(np.eye(3), kin["tcp_offset"]).inverse().action
        j_tcp = adjoint @ j_end

        lam = 1e-6 * max(1.0, prev_err * 10.0)
        jjt = j_tcp @ j_tcp.T
        jjt[np.arange(jjt.shape[0]), np.arange(jjt.shape[1])] += lam
        dq = 0.5 * j_tcp.T @ np.linalg.solve(jjt, err)

        alpha = 1.0
        improved = False
        for _ in range(5):
            q_new = _clamp_config(pin.integrate(model, q, alpha * dq), kin)
            new_err, new_vec = _ik_error(kin, data, q_new, target_tcp)
            if new_err < prev_err:
                q, err, prev_err = q_new, new_vec, new_err
                improved = True
                break
            alpha *= 0.5
        if not improved and np.linalg.norm(dq) < 1e-8:
            break

    return q, False, prev_err, max_iter


def _solve_ik_with_retries(target_tcp: pin.SE3, seed: np.ndarray, arm_type: str | None = None) -> tuple[np.ndarray, bool, float, int]:
    kin = _kinematics(arm_type)
    model = kin["model"]
    best_q, best_ok, best_error, best_iter = _solve_ik(kin, target_tcp, seed)
    if best_ok:
        return _controlled_q(best_q, kin), best_ok, best_error, best_iter

    lo = np.asarray(model.lowerPositionLimit, dtype=np.float64)
    hi = np.asarray(model.upperPositionLimit, dtype=np.float64)
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
        q, ok, error, iterations = _solve_ik(kin, target_tcp, q_seed)
        if error < best_error:
            best_q, best_ok, best_error, best_iter = q, ok, error, iterations
        if ok:
            break
    return _controlled_q(best_q, kin), best_ok, best_error, best_iter


class RealArmBridge:
    def __init__(self, scan_interval: float = 2.0) -> None:
        self._scan_interval = scan_interval
        self._lock = threading.RLock()
        self._active_arm_type = DEFAULT_ARM_TYPE
        self._active_profile = _profile_for_type(self._active_arm_type)
        self._arm = None
        self._gripper_motor = None
        self._gripper_controller = None
        self._gripper_vendor: str | None = None
        self._connected = False
        self._detected = False
        self._interface_present = False
        self._interfaces: list[str] = []
        self._active_channel = ARM_CHANNELS[self._active_arm_type]
        self._last_error = self._driver_import_error_locked()
        self._last_scan_at = 0.0
        self._motors: list[dict] = []
        self._permission_required = False
        self._permission_hint: str | None = None
        self._events: list[dict] = []
        self._monitoring = False
        self._enabled = False
        self._zero_offset_raw: np.ndarray | None = None
        self._zero_last_set_at = 0.0
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._scan_loop, name="real-arm-scan", daemon=True)
        self._thread.start()

    def _driver_locked(self) -> str:
        return str(self._active_profile["driver"])

    def _is_rs_locked(self) -> bool:
        return self._driver_locked() == "rs"

    def _driver_import_error_locked(self) -> str | None:
        if self._driver_locked() == "rs":
            return REBOT_ARM_IMPORT_ERROR
        return ROBOT_ARM_IMPORT_ERROR

    def _driver_available_locked(self) -> bool:
        if self._driver_locked() == "rs":
            return RebotArm is not None
        return RobotArm is not None

    def _set_arm_type_locked(self, arm_type: str | None) -> None:
        normalized = _normalize_arm_type(arm_type)
        if normalized == self._active_arm_type:
            return
        self._close_arm_locked()
        self._active_arm_type = normalized
        self._active_profile = _profile_for_type(normalized)
        self._active_channel = ARM_CHANNELS[normalized]
        self._last_error = self._driver_import_error_locked()
        self._detected = False
        self._interface_present = False
        self._interfaces = []
        self._motors = []
        self._permission_required = False
        self._permission_hint = None
        self._zero_offset_raw = None
        self._zero_last_set_at = 0.0
        self._log_locked("profile", f"已切换机械臂类型: {normalized}")

    def set_arm_type(self, arm_type: str | None) -> dict:
        with self._lock:
            self._set_arm_type_locked(arm_type)
            return self.health()

    def _arm_group_locked(self, arm=None):
        arm = arm or self._ensure_arm_locked()
        if self._is_rs_locked():
            return arm.arm
        return arm

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
                self._disable_arm_locked()
            except Exception as exc:
                self._last_error = str(exc)
        self._connected = False
        self._enabled = False
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
        self._enabled = False
        self._monitoring = False

    def _ensure_arm_locked(self):
        if not self._driver_available_locked():
            if self._is_rs_locked():
                raise RuntimeError(f"RebotArm import failed: {REBOT_ARM_IMPORT_ERROR}")
            raise RuntimeError(f"RobotArm import failed: {ROBOT_ARM_IMPORT_ERROR}")
        if self._arm is None:
            if self._is_rs_locked():
                self._arm = RebotArm(str(self._runtime_cfg_locked()))
                self._arm.connect()
            else:
                self._arm = RobotArm(str(self._runtime_cfg_locked()))
        return self._arm

    def _enable_arm_locked(self, retries: int = 5, poll_interval: float = 0.05, mode: str = "pos_vel", vlim=None) -> None:
        arm = self._ensure_arm_locked()
        if self._is_rs_locked():
            group = self._arm_group_locked(arm)
            if mode == "mit":
                group.mode_mit()
            else:
                group.mode_pos_vel(vlim=vlim)
            arm.enable_all()
            return
        if mode == "pos_vel":
            arm.mode_pos_vel(vlim=vlim, stabilize_delay=0.1)
        elif mode == "mit":
            arm.mode_mit(stabilize_delay=0.1)
        arm.enable(retries=retries, poll_interval=poll_interval)

    def _disable_arm_locked(self, retries: int = 5, poll_interval: float = 0.05) -> None:
        arm = self._ensure_arm_locked()
        if self._is_rs_locked():
            arm.disable_all()
        else:
            arm.disable(retries=retries, poll_interval=poll_interval)

    def _mode_pos_vel_locked(self, vlim=None, stabilize_delay: float = 0.1) -> None:
        group = self._arm_group_locked()
        if self._is_rs_locked():
            group.mode_pos_vel(vlim=vlim)
        else:
            group.mode_pos_vel(vlim=vlim, stabilize_delay=stabilize_delay)

    def _send_pos_vel_locked(self, q: np.ndarray, vlim=None) -> None:
        group = self._arm_group_locked()
        if self._is_rs_locked():
            group.send_pos_vel(q, vlim=vlim)
        else:
            group.pos_vel(q, vlim=vlim)

    def _get_positions_locked(self, request: bool = True) -> np.ndarray:
        group = self._arm_group_locked()
        if self._is_rs_locked():
            return np.asarray(group.get_positions(request_feedback=request), dtype=np.float64)
        return np.asarray(group.get_positions(request=request), dtype=np.float64)

    def _get_velocities_locked(self, request: bool = True) -> np.ndarray:
        group = self._arm_group_locked()
        if self._is_rs_locked():
            return np.asarray(group.get_velocities(request_feedback=request), dtype=np.float64)
        return np.asarray(group.get_velocities(request=request), dtype=np.float64)

    def _get_torques_locked(self, request: bool = True) -> np.ndarray:
        if self._is_rs_locked():
            pos, vel, torq = self._ensure_arm_locked().get_state(request_feedback=request)
            return np.asarray(torq[: self._active_profile["controlled_joints"]], dtype=np.float64)
        return np.asarray(self._arm_group_locked().get_torques(request=request), dtype=np.float64)

    def _read_stable_positions_locked(self, samples: int = 5, interval_s: float = 0.04) -> np.ndarray:
        self._ensure_arm_locked()
        readings = []
        for _ in range(max(1, samples)):
            readings.append(self._get_positions_locked(request=True))
            time.sleep(interval_s)
        return np.median(np.vstack(readings), axis=0)

    def _calibrate_zero_locked(self, reason: str) -> np.ndarray:
        zero = self._read_stable_positions_locked()
        self._zero_offset_raw = zero.copy()
        self._zero_last_set_at = time.time()
        self._log_locked(
            "calibration",
            reason,
            zero_reference_before_rad=self._zero_offset_raw.tolist(),
        )
        return self._zero_offset_raw

    def _start_feedback_monitor_locked(self) -> None:
        arm = self._ensure_arm_locked()
        if arm.control_loop_active and self._monitoring:
            return
        if arm.control_loop_active:
            arm.stop_control_loop()

        def mit_monitor(ref, _dt):
            group = ref.arm if self._is_rs_locked() else ref
            n = int(group.num_joints)
            if self._is_rs_locked():
                group.send_mit(
                    np.zeros(n),
                    kp=np.zeros(n),
                    kd=np.zeros(n),
                    tau=np.zeros(n),
                )
            else:
                group.mit(
                    np.zeros(n),
                    kp=np.zeros(n),
                    kd=np.zeros(n),
                    tau=np.zeros(n),
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
            self._zero_last_set_at = 0.0

    def _find_controller_for_vendor_locked(self, arm, vendor: str):
        ctrl_map = getattr(arm, "_ctrl_map", {})
        if isinstance(ctrl_map, dict):
            if vendor in ctrl_map:
                return ctrl_map[vendor]
            if ctrl_map:
                return next(iter(ctrl_map.values()))
        return None

    def _gripper_config_locked(self) -> dict | None:
        if self._is_rs_locked():
            try:
                data = yaml.safe_load(Path(self._active_profile["cfg_path"]).read_text(encoding="utf-8")) or {}
                group_joints = ((data.get("groups") or {}).get("gripper") or {}).get("joints") or []
                if not group_joints:
                    return None
                joint_name = str(group_joints[0])
                joint = next((item for item in data.get("joints", []) if str(item.get("name")) == joint_name), None)
                if not joint:
                    return None
                return {
                    "name": joint_name,
                    "vendor": str(joint.get("vendor", "robstride")).lower(),
                    "motor_id": int(joint.get("motor_id", 0x07)),
                    "feedback_id": int(joint.get("feedback_id", 0xFD)),
                    "model": str(joint.get("model", "rs-00")),
                }
            except Exception:
                return None
        if not GRIPPER_CFG.is_file():
            return None
        data = yaml.safe_load(GRIPPER_CFG.read_text(encoding="utf-8")) or {}
        if not isinstance(data, dict):
            raise RuntimeError(f"夹爪配置格式无效: {GRIPPER_CFG}")
        gripper_list = data.get("gripper") or []
        if not gripper_list:
            raise RuntimeError(f"夹爪配置缺少 gripper 节点: {GRIPPER_CFG}")
        gripper = gripper_list[0] or {}
        return {
            "name": str(gripper.get("name") or "gripper"),
            "vendor": str(gripper.get("vendor", "damiao")).lower(),
            "motor_id": int(gripper.get("motor_id", 0x07)),
            "feedback_id": int(gripper.get("feedback_id", 0x17)),
            "model": str(gripper.get("model", "4310")),
        }

    def _ensure_gripper_attached_locked(self) -> None:
        if self._gripper_motor is not None and self._gripper_controller is not None:
            return
        if self._arm is None:
            raise RuntimeError("真实机械臂未初始化，无法挂载夹爪")
        cfg = self._gripper_config_locked()
        if cfg is None:
            raise RuntimeError(f"未找到夹爪配置文件: {GRIPPER_CFG}")

        vendor = cfg["vendor"]
        ctrl = self._find_controller_for_vendor_locked(self._arm, vendor)
        if ctrl is None:
            raise RuntimeError(f"未找到夹爪控制器: vendor={vendor!r}")

        motor_id = cfg["motor_id"]
        feedback_id = cfg["feedback_id"]
        model = cfg["model"]

        if self._is_rs_locked():
            mot = getattr(self._arm, "_motor_map", {}).get(cfg["name"])
            if mot is None:
                raise RuntimeError(f"未找到 RS 夹爪电机: {cfg['name']}")
        elif vendor == "damiao":
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
        if self._is_rs_locked() and self._arm is not None and getattr(self._arm, "has_gripper", False):
            try:
                cfg = self._gripper_config_locked()
                if cfg is None:
                    return None
                group = self._arm.gripper
                if request:
                    group.get_positions(request_feedback=True)
                mot = getattr(self._arm, "_motor_map", {}).get(cfg["name"])
                state = mot.get_state() if mot is not None else None
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

    def _set_gripper_zero_locked(self, poll_max: int = 200, poll_interval: float = 0.05) -> dict:
        self._ensure_gripper_attached_locked()
        if self._gripper_motor is None or self._gripper_controller is None:
            raise RuntimeError("夹爪未挂载到当前控制器")

        before = self._gripper_state_locked(request=True)
        try:
            self._gripper_controller.disable_all()
        except Exception as exc:
            raise RuntimeError(f"夹爪失能失败: {exc}") from exc

        ready = False
        for _ in range(max(1, poll_max)):
            state = self._gripper_state_locked(request=True)
            if state is not None and state.get("status_code") == 0:
                ready = True
                break
            time.sleep(poll_interval)
        if not ready:
            raise RuntimeError("夹爪等待失能就绪超时")

        try:
            self._gripper_motor.set_zero_position()
        except Exception as exc:
            raise RuntimeError(f"夹爪写入 0 点失败: {exc}") from exc

        time.sleep(0.2)
        after = self._gripper_state_locked(request=True)
        self._log_locked(
            "calibration",
            "joint7 / 夹爪 0 点已重写",
            gripper_state_before=before,
            gripper_state_after=after,
        )
        return {
            "before": before,
            "after": after,
        }

    def _runtime_cfg_locked(self) -> Path:
        cfg_path = Path(self._active_profile["cfg_path"])
        if self._active_channel == ARM_CHANNELS[self._active_arm_type]:
            return cfg_path
        data = yaml.safe_load(cfg_path.read_text()) or {}
        data["channel"] = self._active_channel
        runtime_cfg = ROOT / f".runtime_{self._active_arm_type}.yaml"
        runtime_cfg.write_text(yaml.safe_dump(data, allow_unicode=True), encoding="utf-8")
        return runtime_cfg

    def _scan_once_locked(self) -> None:
        profile = {**self._active_profile, "channel": self._active_channel}
        self._interface_present, self._interfaces, interface_message = _scan_interfaces(profile)
        self._set_active_channel_locked(self._interfaces[0] if self._interfaces else ARM_CHANNELS[self._active_arm_type])
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

        has_permission, permission_hint = (
            _device_permission_status(self._interfaces)
            if self._active_profile["channel_kind"] == "serial"
            else (True, None)
        )
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
        motors = self._collect_motor_feedback_locked(arm)
        detected = bool(motors)
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

    def _collect_motor_feedback_locked(self, arm=None, polls: int = 3, interval_s: float = 0.05) -> list[dict]:
        arm = arm or self._ensure_arm_locked()
        # Feedback is the real handshake for both the DM serial bridge and RS CAN.
        for _ in range(max(1, polls)):
            if self._is_rs_locked():
                arm.get_state(request_feedback=True)
            else:
                arm._request_and_poll()
            time.sleep(interval_s)

        motors = []
        joint_cfgs = getattr(arm, "_all_joints", None) if self._is_rs_locked() else getattr(arm, "_joints", None)
        joint_cfgs = joint_cfgs or []
        for joint in joint_cfgs:
            state = None
            try:
                state = arm._motor_map[joint.name].get_state()
            except Exception:
                state = None
            if state is None:
                continue
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
                    "vendor": getattr(joint, "vendor", self._active_profile["driver"]),
                    "role": "gripper_motor" if str(joint.name) == "gripper" else "arm_joint",
                }
            )
        if self._is_rs_locked():
            return motors
        try:
            gripper_cfg = self._gripper_config_locked()
            if gripper_cfg is not None:
                self._ensure_gripper_attached_locked()
                gripper_state = self._gripper_state_locked(request=True)
                if gripper_state is not None:
                    motor_id = int(gripper_cfg["motor_id"])
                    feedback_id = int(gripper_cfg["feedback_id"])
                    exists = any(
                        int(motor.get("motor_id", -1)) == motor_id
                        and int(motor.get("feedback_id", -1)) == feedback_id
                        for motor in motors
                    )
                    if not exists:
                        motors.append(
                            {
                                "name": f"joint{motor_id}",
                                "motor_id": motor_id,
                                "feedback_id": feedback_id,
                                "model": gripper_cfg["model"],
                                "status_code": gripper_state.get("status_code"),
                                "pos": float(gripper_state.get("pos", 0.0)),
                                "vel": float(gripper_state.get("vel", 0.0)),
                                "torq": float(gripper_state.get("torq", 0.0)),
                                "alias": gripper_cfg["name"],
                                "role": "gripper_motor",
                                "vendor": gripper_cfg["vendor"],
                            }
                        )
        except Exception:
            pass
        return motors

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
            gripper_cfg = None
            gripper_state = None
            try:
                gripper_cfg = self._gripper_config_locked()
            except Exception:
                gripper_cfg = None
            try:
                if self._gripper_motor is not None and self._gripper_controller is not None:
                    gripper_state = self._gripper_state_locked(request=False)
            except Exception:
                gripper_state = None
            return {
                "ok": True,
                "arm_type": self._active_arm_type,
                "arm_label": self._active_profile["label"],
                "arm_profiles": [
                    {"type": key, "label": profile["label"], "channel_kind": profile["channel_kind"]}
                    for key, profile in ARM_PROFILES.items()
                ],
                "connected": self._connected,
                "enabled": self._enabled,
                "arm_detected": self._detected,
                "interface_present": self._interface_present,
                "channel": self._active_channel,
                "interfaces": self._interfaces,
                "detected_motors": self._motors,
                "last_scan_at": self._last_scan_at,
                "scan_interval_s": self._scan_interval,
                "real_arm_available": self._driver_available_locked(),
                "real_arm_driver": self._driver_locked(),
                "gripper_available": (self._is_rs_locked() and gripper_cfg is not None) or (GRIPPER_CFG.is_file() and Mode is not None),
                "gripper_attached": self._gripper_motor is not None and self._gripper_controller is not None,
                "gripper_name": gripper_cfg["name"] if gripper_cfg else None,
                "gripper_vendor": gripper_cfg["vendor"] if gripper_cfg else None,
                "gripper_model": gripper_cfg["model"] if gripper_cfg else None,
                "gripper_motor_id": gripper_cfg["motor_id"] if gripper_cfg else None,
                "gripper_feedback_id": gripper_cfg["feedback_id"] if gripper_cfg else None,
                "gripper_state": gripper_state,
                "zero_calibrated": self._zero_offset_raw is not None,
                "zero_last_set_at": self._zero_last_set_at or None,
                "permission_required": self._permission_required,
                "permission_hint": self._permission_hint,
                "error": self._last_error,
            }

    def logs(self) -> dict:
        with self._lock:
            return {"ok": True, "events": self._events}

    def connect(self, confirmed: bool = False, arm_type: str | None = None) -> dict:
        with self._lock:
            self._set_arm_type_locked(arm_type)
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
            
            # RS 电机必须在使能后才能返回反馈，所以只要接口存在就可以连接
            # DM 电机可以在未使能时返回反馈，需要检查反馈
            if not self._is_rs_locked() and not self._detected:
                self._log_locked("connect", "未读取到电机反馈，无法连接真实机械臂")
                return {**self.health(), "ok": False, "message": "未读取到电机反馈，无法连接真实机械臂"}
            
            if not confirmed:
                self._log_locked("connect", "检测到电机反馈，等待用户确认连接", motors=len(self._motors))
                return {**self.health(), "ok": True, "requires_confirmation": True}
            arm = self._ensure_arm_locked()
            arm.connect()
            
            # RS 电机使能后再读取反馈确认
            self._enable_arm_locked(retries=5, poll_interval=0.05, mode="pos_vel")
            self._enabled = True
            
            # 使能后再次扫描确认电机响应
            self._scan_once_locked()
            if not self._is_rs_locked() and not self._detected:
                self._log_locked("connect", "连接前未读取到电机反馈")
                return {**self.health(), "ok": False, "message": "连接前未读取到电机反馈，请检查机械臂供电和通信线"}
            self._connected = True
            try:
                self._ensure_gripper_attached_locked()
            except Exception as exc:
                self._log_locked("gripper", f"夹爪挂载失败: {exc}")
            self._log_locked(
                "connect",
                "真实机械臂已连接并完成使能",
                arm_type=self._active_arm_type,
                channel=self._active_channel,
                mode_before_enable=True,
            )
            return {**self.health(), "ok": True, "message": "真实机械臂已连接并完成使能"}

    def configure_serial_permission(self) -> dict:
        with self._lock:
            if self._active_profile["channel_kind"] != "serial":
                interface_present, interfaces, interface_message = _scan_interfaces({**self._active_profile, "channel": self._active_channel})
                self._interface_present = interface_present
                self._interfaces = interfaces
                self._last_scan_at = time.time()
                self._permission_required = False
                self._permission_hint = None
                self._last_error = None if interface_present else interface_message
                return {**self.health(), "ok": interface_present, "message": interface_message}

            interface_present, interfaces, interface_message = _scan_interfaces(self._active_profile)
            self._interface_present = interface_present
            self._interfaces = interfaces
            self._set_active_channel_locked(interfaces[0] if interfaces else ARM_CHANNELS[self._active_arm_type])
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

    def debug_action(self, action: str) -> dict:
        with self._lock:
            action = str(action or "").lower()
            if action == "scan":
                return self._debug_scan_locked()
            if action == "feedback":
                return self._debug_feedback_locked()
            if action == "enable":
                return self._debug_enable_locked()
            if action == "disable":
                return self._debug_disable_locked()
            if action == "calibrate":
                return self._debug_calibrate_locked()
            return {**self.health(), "ok": False, "message": f"unsupported debug action: {action}"}

    def _debug_scan_locked(self) -> dict:
        try:
            if self._connected and self._arm is not None:
                self._motors = self._collect_motor_feedback_locked(self._arm)
                self._detected = bool(self._motors)
                self._last_scan_at = time.time()
                self._last_error = None if self._detected else "未读取到电机反馈"
            else:
                self._scan_once_locked()
        except Exception as exc:
            self._last_error = str(exc)
            self._log_locked("debug", f"重新扫描失败: {exc}")
            return {**self.health(), "ok": False, "message": f"重新扫描失败: {exc}"}
        self._last_error = None if self._detected or self._interface_present else self._last_error
        message = "已重新扫描接口和电机反馈"
        self._log_locked("debug", message, motors=self._motors, interfaces=self._interfaces)
        return {**self.health(), "ok": True, "message": message}

    def _debug_feedback_locked(self) -> dict:
        if self._permission_required:
            return {**self.health(), "ok": False, "message": self._permission_hint or "串口权限不足"}
        if not self._interface_present:
            missing = "未检测到 /dev/ttyACM* 设备" if self._active_profile["channel_kind"] == "serial" else "未检测到 can* SocketCAN 接口"
            return {**self.health(), "ok": False, "message": missing}
        try:
            arm = self._ensure_arm_locked()
            self._motors = self._collect_motor_feedback_locked(arm)
            self._detected = bool(self._motors)
            self._last_scan_at = time.time()
            self._last_error = None if self._detected else "未读取到电机反馈"
        except Exception as exc:
            self._last_error = str(exc)
            self._log_locked("debug", f"读取反馈失败: {exc}")
            return {**self.health(), "ok": False, "message": f"读取反馈失败: {exc}"}
        self._last_error = None if self._motors else self._last_error
        message = f"已读取到 {len(self._motors)} 路电机反馈" if self._motors else "未读取到电机反馈"
        self._log_locked("debug", message, motors=self._motors)
        return {**self.health(), "ok": bool(self._motors), "message": message}

    def _debug_enable_locked(self) -> dict:
        if self._permission_required:
            return {**self.health(), "ok": False, "message": self._permission_hint or "串口权限不足"}
        try:
            if not self._interface_present:
                self._scan_once_locked()
            arm = self._ensure_arm_locked()
            
            # RS 电机必须在使能后才能返回反馈，所以跳过预检查直接使能
            # DM 电机可以在未使能时返回反馈，需要先检查反馈
            if not self._is_rs_locked():
                self._motors = self._collect_motor_feedback_locked(arm)
                self._detected = bool(self._motors)
                if not self._detected:
                    return {**self.health(), "ok": False, "message": "未读取到电机反馈，无法执行使能"}
            
            self._enable_arm_locked(retries=5, poll_interval=0.05, mode="pos_vel")
            self._enabled = True
            
            # 使能后再次读取反馈以确认
            self._motors = self._collect_motor_feedback_locked(arm)
            self._detected = bool(self._motors)
            
            try:
                self._ensure_gripper_attached_locked()
            except Exception as exc:
                self._log_locked("gripper", f"夹爪挂载失败: {exc}")
        except Exception as exc:
            self._enabled = False
            self._last_error = str(exc)
            self._log_locked("debug", f"机械臂使能失败: {exc}")
            return {**self.health(), "ok": False, "message": f"机械臂使能失败: {exc}"}
        self._last_error = None
        message = "机械臂已使能"
        self._log_locked("debug", message, connected=self._connected, motors=len(self._motors))
        return {**self.health(), "ok": True, "message": message}

    def _debug_disable_locked(self) -> dict:
        try:
            if self._arm is None:
                return {**self.health(), "ok": False, "message": "机械臂控制器未初始化"}
            self._stop_feedback_monitor_locked()
            self._disable_arm_locked(retries=5, poll_interval=0.05)
            self._enabled = False
        except Exception as exc:
            self._last_error = str(exc)
            self._log_locked("debug", f"机械臂失能失败: {exc}")
            return {**self.health(), "ok": False, "message": f"机械臂失能失败: {exc}"}
        self._last_error = None
        message = "机械臂已失能"
        self._log_locked("debug", message, connected=self._connected)
        return {**self.health(), "ok": True, "message": message}

    def _debug_calibrate_locked(self) -> dict:
        if self._permission_required:
            return {**self.health(), "ok": False, "message": self._permission_hint or "串口权限不足"}
        if not self._connected or self._arm is None:
            return {**self.health(), "ok": False, "message": "请先连接机械臂后再执行零点校准"}

        zero_before = None
        zero_after = None
        gripper_zero_before = None
        gripper_zero_after = None
        arm_zero_written = False
        gripper_zero_written = False
        try:
            self._stop_feedback_monitor_locked()
            zero_before = self._calibrate_zero_locked("准备重写机械臂 0 点")
            if self._is_rs_locked():
                self._arm.set_zero(poll_max=200, poll_interval=0.05)
            else:
                self._arm.set_zero(poll_max=200, poll_interval=0.05, set_zero_delay=0.12)
            arm_zero_written = True
            if not self._is_rs_locked() and GRIPPER_CFG.is_file():
                gripper_zero = self._set_gripper_zero_locked(poll_max=200, poll_interval=0.05)
                gripper_zero_before = gripper_zero.get("before")
                gripper_zero_after = gripper_zero.get("after")
                gripper_zero_written = True
            self._enabled = False
            time.sleep(0.35)
            self._enable_arm_locked(retries=5, poll_interval=0.05, mode="pos_vel")
            self._enabled = True
            zero_after = self._read_stable_positions_locked(samples=5, interval_s=0.03)
            if gripper_zero_written:
                gripper_zero_after = self._gripper_state_locked(request=True)
            self._motors = self._collect_motor_feedback_locked(self._arm)
            self._detected = bool(self._motors)
            self._last_scan_at = time.time()
            self._last_error = None
        except Exception as exc:
            self._last_error = str(exc)
            message = f"机械臂零点校准失败: {exc}"
            if arm_zero_written and not gripper_zero_written:
                message = f"机械臂 1-6 轴 0 点已重写，但 joint7 / 夹爪校准失败: {exc}"
            elif arm_zero_written and gripper_zero_written:
                message = f"机械臂与夹爪 0 点已重写，但重新使能失败: {exc}"
            self._log_locked(
                "calibration",
                message,
                zero_reference_before_rad=zero_before.tolist() if zero_before is not None else None,
                zero_reference_after_rad=zero_after.tolist() if zero_after is not None else None,
                gripper_state_before=gripper_zero_before,
                gripper_state_after=gripper_zero_after,
            )
            return {**self.health(), "ok": False, "message": message}

        message = "机械臂与夹爪 0 点已重写并重新使能" if gripper_zero_written else "机械臂 0 点已重写并重新使能"
        self._log_locked(
            "calibration",
            message,
            zero_reference_before_rad=zero_before.tolist() if zero_before is not None else None,
            zero_reference_after_rad=zero_after.tolist() if zero_after is not None else None,
            gripper_state_before=gripper_zero_before,
            gripper_state_after=gripper_zero_after,
            gripper_zero_written=gripper_zero_written,
            connected=self._connected,
        )
        result = self.state()
        result.update(
            {
                "ok": True,
                "message": message,
                "zero_reference_before_rad": zero_before.tolist() if zero_before is not None else None,
                "zero_reference_after_rad": zero_after.tolist() if zero_after is not None else None,
                "gripper_state_before": gripper_zero_before,
                "gripper_state_after": gripper_zero_after,
                "gripper_zero_written": gripper_zero_written,
            }
        )
        return result

    def command_gripper(self, action: str) -> dict:
        with self._lock:
            action = str(action or "").lower()
            if action not in {"open", "close"}:
                return {**self.health(), "ok": False, "message": f"不支持的夹爪动作: {action}"}
            if not self._connected or self._arm is None:
                self._log_locked("gripper", "真实机械臂未连接，无法控制夹爪", action=action)
                return {**self.health(), "ok": False, "connected": False, "message": "真实机械臂未连接"}
            if Mode is None and not self._is_rs_locked():
                message = f"夹爪驱动不可用: {MOTORBRIDGE_IMPORT_ERROR}"
                self._log_locked("gripper", message, action=action)
                return {**self.health(), "ok": False, "message": message}

            try:
                target_pos = GRIPPER_OPEN_POS_RAD if action == "open" else GRIPPER_CLOSE_POS_RAD
                if self._is_rs_locked():
                    if not self._arm.has_gripper:
                        raise RuntimeError("当前 RS 配置未启用夹爪电机")
                    self._arm.gripper.mode_pos_vel(vlim=np.array([GRIPPER_VLIM], dtype=np.float64))
                    self._arm.gripper.enable()
                    self._arm.gripper.send_pos_vel(np.array([target_pos], dtype=np.float64), vlim=np.array([GRIPPER_VLIM], dtype=np.float64))
                    gripper_state = self._gripper_state_locked(request=True)
                else:
                    self._ensure_gripper_attached_locked()
                    if self._gripper_controller is None or self._gripper_motor is None:
                        raise RuntimeError("夹爪未挂载到当前控制器")
                    self._gripper_controller.enable_all()
                    self._gripper_motor.ensure_mode(Mode.POS_VEL, 1000)
                    self._gripper_motor.send_pos_vel(float(target_pos), float(GRIPPER_VLIM))
                    try:
                        self._gripper_controller.poll_feedback_once()
                    except Exception:
                        pass
                    gripper_state = None
                time.sleep(min(0.2, GRIPPER_SETTLE_DELAY_S))
                gripper_state = gripper_state or self._wait_for_gripper_target_locked(target_pos)
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
                pos = self._get_positions_locked(request=True)
                vel = self._get_velocities_locked(request=True)
                torq = self._get_torques_locked(request=True)
                payload.update(
                    {
                        "joints_rad": pos.tolist(),
                        "raw_joints_rad": pos.tolist(),
                        "velocities_rad_s": vel.tolist(),
                        "torques_nm": torq.tolist(),
                        "end_effector": _fk(pos, self._active_arm_type)["end_effector"],
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
            current = self._get_positions_locked(request=True)
            kin = _kinematics(self._active_arm_type)
            target_full = _clamp_config(_to_model_q(np.asarray(q, dtype=np.float64).reshape(-1), kin), kin)
            target = _controlled_q(target_full, kin)
            max_speed = float(np.clip(max_speed_rad_s, 0.05, 0.35))
            delta = target - current
            duration = float(np.clip(np.max(np.abs(delta)) / max_speed if delta.size else 0.0, 1.5, 8.0))
            steps = max(20, int(duration / 0.05))
            slow_vlim = np.full(kin["controlled_joints"], max_speed, dtype=np.float64)

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
            self._enable_arm_locked(retries=5, poll_interval=0.05, mode="pos_vel", vlim=slow_vlim)
            self._enabled = True
            self._mode_pos_vel_locked(vlim=slow_vlim, stabilize_delay=0.1)

            target_ref = {"q": current.copy()}

            def pos_vel_controller(ref, _dt):
                if self._is_rs_locked():
                    ref.arm.send_pos_vel(target_ref["q"], vlim=slow_vlim)
                else:
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


def _arm_type_from_query(parsed) -> str:
    values = parse_qs(parsed.query).get("arm_type") or parse_qs(parsed.query).get("type") or []
    return _normalize_arm_type(values[0] if values else None)


def _sim_config_payload(arm_type: str | None = None) -> dict:
    profile = _profile_for_type(arm_type)
    kin = _kinematics(profile["arm_type"])
    return {
        "ok": True,
        "arm_type": profile["arm_type"],
        "arm_label": profile["label"],
        "arm_profiles": [
            {"type": key, "label": item["label"], "channel_kind": item["channel_kind"]}
            for key, item in ARM_PROFILES.items()
        ],
        "urdf_url": profile["urdf_url"],
        "package_alias": profile["package_alias"],
        "package_url": profile["package_url"],
        "joint_name_map": profile["joint_name_map"],
        "end_effector_link": profile["end_effector_frame"],
        "tcp_axis": profile["tcp_axis"],
        "controlled_joints": kin["controlled_joints"],
        "model_joints": kin["model"].nq,
    }


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
            _json_response(self, _sim_config_payload(_arm_type_from_query(parsed)))
            return
        if path == "/profile":
            _json_response(self, {**REAL_ARM.health(), **_sim_config_payload(REAL_ARM.health().get("arm_type"))})
            return
        if path == "/assets/urdf/model.urdf":
            self._send_file(Path(_profile_for_type(REAL_ARM.health().get("arm_type"))["urdf_path"]), "application/xml")
            return
        if path.startswith("/assets/urdf/reBot-DevArm_fixend_description/"):
            profile = ARM_PROFILES["dm"]
            asset = unquote(path.split("/assets/urdf/reBot-DevArm_fixend_description/", 1)[1]).lstrip("/")
            self._send_file((Path(profile["urdf_root"]) / asset).resolve())
            return
        if path.startswith("/assets/urdf/reBot-DevArm-rs_asm-v3/"):
            profile = ARM_PROFILES["rs"]
            asset = unquote(path.split("/assets/urdf/reBot-DevArm-rs_asm-v3/", 1)[1]).lstrip("/")
            self._send_file((Path(profile["urdf_root"]) / asset).resolve())
            return
        for profile in ARM_PROFILES.values():
            prefix = f"/assets/urdf/package/{profile['package_alias']}/"
            if path.startswith(prefix):
                asset = unquote(path.split(prefix, 1)[1]).lstrip("/")
                self._send_file((Path(profile["urdf_root"]) / asset).resolve())
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
            arm_type = _normalize_arm_type(payload.get("arm_type") or payload.get("type") or REAL_ARM.health().get("arm_type"))

            if path == "/sim/fk":
                q = np.asarray(payload.get("joints_rad", []), dtype=np.float64).reshape(-1)
                kin = _kinematics(arm_type)
                if q.shape[0] not in {kin["controlled_joints"], kin["model"].nq}:
                    raise ValueError(f"expected {kin['controlled_joints']} joints, got {q.shape[0]}")
                _json_response(self, _fk(q, arm_type))
                return

            if path == "/sim/ik":
                kin = _kinematics(arm_type)
                pos = np.array([payload["x"], payload["y"], payload["z"]], dtype=np.float64)
                rot = pin.rpy.rpyToMatrix(
                    float(payload.get("roll", 0.0)),
                    float(payload.get("pitch", 0.0)),
                    float(payload.get("yaw", 0.0)),
                )
                target_tcp = pin.SE3(rot, pos)
                seed = np.asarray(payload.get("seed_joints_rad") or np.zeros(kin["controlled_joints"]), dtype=np.float64).reshape(-1)
                if seed.shape[0] not in {kin["controlled_joints"], kin["model"].nq}:
                    seed = np.zeros(kin["controlled_joints"], dtype=np.float64)
                q, ok, error, iterations = _solve_ik_with_retries(target_tcp, seed, arm_type)
                result = _fk(q, arm_type)
                result.update({"ok": ok, "iterations": iterations, "error": error})
                _json_response(self, result)
                return

            if path == "/profile":
                _json_response(self, REAL_ARM.set_arm_type(arm_type))
                return

            if path == "/connect":
                _json_response(self, REAL_ARM.connect(confirmed=bool(payload.get("confirm")), arm_type=arm_type))
                return

            if path == "/disconnect":
                _json_response(self, REAL_ARM.disconnect())
                return

            if path == "/permissions/serial":
                _json_response(self, REAL_ARM.configure_serial_permission())
                return

            if path == "/debug/arm":
                _json_response(self, REAL_ARM.debug_action(str(payload.get("action") or "")))
                return

            if path == "/gripper":
                _json_response(self, REAL_ARM.command_gripper(str(payload.get("action") or "")))
                return

            if path == "/move/joints":
                q = np.asarray(payload.get("joints_rad", []), dtype=np.float64).reshape(-1)
                kin = _kinematics(arm_type)
                if q.shape[0] not in {kin["controlled_joints"], kin["model"].nq}:
                    raise ValueError(f"expected {kin['controlled_joints']} joints, got {q.shape[0]}")
                if REAL_ARM.health().get("connected"):
                    speed = float(payload.get("max_speed_rad_s", 0.22))
                    _json_response(self, REAL_ARM.send_joints(q, max_speed_rad_s=speed))
                else:
                    result = _fk(q, arm_type)
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


def _find_available_port(start_port: int, max_attempts: int = 100) -> int:
    """Find an available port starting from start_port."""
    import socket
    for port in range(start_port, start_port + max_attempts):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.bind(("0.0.0.0", port))
                return port
        except OSError:
            continue
    raise RuntimeError(f"Could not find available port after {max_attempts} attempts")


def _server_host_port() -> tuple[str, int]:
    host = os.getenv("HOST") or "0.0.0.0"
    port_raw = os.getenv("PORT") or ""
    if port_raw:
        try:
            port = int(port_raw)
        except ValueError as exc:
            raise RuntimeError(f"invalid PORT: {port_raw}") from exc
        return host, port
    # No PORT specified, auto-find available port starting from 8000
    port = _find_available_port(8000)
    return host, port


if __name__ == "__main__":
    host, port = _server_host_port()
    try:
        server = ThreadingHTTPServer((host, port), Handler)
    except OSError as exc:
        if exc.errno == 98:
            print(
                f"port {port} is already in use. Stop the existing service or run PORT=<port> npm run start.",
                file=sys.stderr,
            )
        raise
    print(f"rebot arm webui backend: http://localhost:{port}/")
    server.serve_forever()
