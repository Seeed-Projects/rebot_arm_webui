import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { STLLoader } from "three/examples/jsm/loaders/STLLoader.js";
import URDFLoader from "urdf-loader";

const fallbackSimConfig = {
  armType: "dm",
  armLabel: "Damiao",
  urdfUrl: "./assets/urdf/reBot-DevArm_fixend_description/urdf/reBot-DevArm_fixend.urdf",
  packageMap: {
    "reBot-DevArm_description_fixend": "./assets/urdf/reBot-DevArm_fixend_description/",
  },
  endEffectorLink: "end_link",
  tcpAxis: "max_x",
  jointNameMap: {
    joint1: "joint1",
    joint2: "joint2",
    joint3: "join3",
    joint4: "joint4",
    joint5: "joint5",
    joint6: "joint6",
  },
};

const fallbackSimConfigs = {
  dm: fallbackSimConfig,
  rs: {
    armType: "rs",
    armLabel: "RS",
    urdfUrl: "./assets/urdf/reBot-DevArm-rs_asm-v3/urdf/00-arm-rs_asm-v3.urdf",
    packageMap: {
      "reBot-DevArm-rs_asm-v3": "./assets/urdf/reBot-DevArm-rs_asm-v3/",
    },
    endEffectorLink: "gripper_end",
    tcpAxis: "min_x",
    jointNameMap: {
      joint1: "joint1",
      joint2: "joint2",
      joint3: "joint3",
      joint4: "joint4",
      joint5: "joint5",
      joint6: "joint6",
    },
  },
};

const scenePalette = {
  viewportBackground: new THREE.Color(0xf1f5f5),
  hemisphereSky: 0xffffff,
  hemisphereGround: 0xcce3e7,
  fillLight: 0x99c7cf,
  gridMajor: 0x99c7cf,
  gridMinor: 0xe5e5e5,
  floor: 0xfafafa,
  tcpMarker: 0x8fc31f,
  robotMesh: 0xa3a3a3,
};

// DM 版本关节限位
const dmJoints = [
  { name: "joint1", urdfName: "joint1", motorId: "0x01", feedbackId: "0x11", model: "4340P", min: -160, max: 160, angle: 0 },
  { name: "joint2", urdfName: "joint2", motorId: "0x02", feedbackId: "0x12", model: "4340P", min: -180, max: 0, angle: 0 },
  { name: "joint3", urdfName: "join3", motorId: "0x03", feedbackId: "0x13", model: "4340P", min: -180, max: 0, angle: 0 },
  { name: "joint4", urdfName: "joint4", motorId: "0x04", feedbackId: "0x14", model: "4310", min: -180, max: 180, angle: 0 },
  { name: "joint5", urdfName: "joint5", motorId: "0x05", feedbackId: "0x15", model: "4310", min: -180, max: 180, angle: 0 },
  { name: "joint6", urdfName: "joint6", motorId: "0x06", feedbackId: "0x16", model: "4310", min: -180, max: 180, angle: 0 },
];

// RS 版本关节限位 (弧度转角度: joint2/joint3 是 0~180度，其他关节限位根据 URDF)
const rsJoints = [
  { name: "joint1", urdfName: "joint1", motorId: "0x01", feedbackId: "0xFD", model: "rs-06", min: -160, max: 160, angle: 0, mitKp: 50,  mitKd: 3  },
  { name: "joint2", urdfName: "joint2", motorId: "0x02", feedbackId: "0xFD", model: "rs-06", min: 0,   max: 180, angle: 0, mitKp: 150, mitKd: 10 },
  { name: "joint3", urdfName: "joint3", motorId: "0x03", feedbackId: "0xFD", model: "rs-06", min: 0,   max: 180, angle: 0, mitKp: 150, mitKd: 10 },
  { name: "joint4", urdfName: "joint4", motorId: "0x04", feedbackId: "0xFD", model: "rs-00", min: -90,  max: 90,  angle: 0, mitKp: 50,  mitKd: 5  },
  { name: "joint5", urdfName: "joint5", motorId: "0x05", feedbackId: "0xFD", model: "rs-00", min: -90,  max: 90,  angle: 0, mitKp: 50,  mitKd: 4  },
  { name: "joint6", urdfName: "joint6", motorId: "0x06", feedbackId: "0xFD", model: "rs-00", min: -180, max: 180, angle: 0, mitKp: 50,  mitKd: 4  },
];

// 动态关节配置，根据 arm type 选择
let joints = [...dmJoints];

const state = {
  connected: false,
  enabled: false,
  gripperBusy: false,
  estop: false,
  backendOnline: false,
  armDetected: false,
  interfacePresent: false,
  detectedMotors: [],
  permissionRequired: false,
  permissionHint: "",
  sudoPasswordRequired: false,
  armPromptDismissed: false,
  armModalMode: "connect",
  apiMode: "mock",
  armType: "dm",
  controlMode: "pos_vel",
  mitKp: null,
  mitKd: null,
  armProfiles: [
    { type: "dm", label: "Damiao", channel_kind: "serial" },
    { type: "rs", label: "RS", channel_kind: "can" },
  ],
  simConfig: fallbackSimConfig,
  mode: "positionControl",
  robot: null,
  tcpLocalOffset: null,
  lastPose: {
    position_m: [0.31, 0.02, 0.24],
    rpy_rad: [0, 0.3141592653589793, 0],
  },
  fkSeq: 0,
  ikSeq: 0,
  suppressCartesianSolve: false,
  activeView: "control",
  animationFrameId: null,
  lastBackgroundHealthAt: 0,
  lastBackgroundStateAt: 0,
  lastBackendLogAt: 0,
  locale: "zh-CN",
  solverStatus: { message: { key: "solver.ready" }, warn: false },
  overlay: { message: { key: "overlay.loadingUrdf" }, persistent: true },
  lastHealth: null,
  lastRobotState: null,
  diagnosticsBusyAction: "",
  diagnosticsBusyLabel: "",
  diagnosticsStatusMessage: { key: "diagnostics.statusIdle" },
  selectedMotorName: "",
};


const SUPPORTED_LOCALES = ["zh-CN", "en-US"];
const LANGUAGE_STORAGE_KEY = "rebot-arm-webui.locale";
const translations = {
  "zh-CN": {
    brand: { title: "rebot arm webui", subtitle: "DevArm Console" },
    aria: {
      mainNav: "主导航",
      language: "语言",
      armType: "机械臂类型",
      viewMode: "视图模式",
      armViewport: "机械臂 3D URDF 仿真视图",
    },
    language: { zhCN: "中文", enUS: "English" },
    nav: { control: "控制台", apps: "应用中心", diagnostics: "诊断" },
    service: {
      label: "后台服务",
      sameOrigin: "same-origin REST API",
      mock: "Mock 模式",
      backendOnline: "后端在线",
      armDetected: "检测到机械臂",
      commDetected: "检测到通信设备",
      realConnected: "真实机械臂已连接",
    },
    view: {
      controlEyebrow: "rebot arm webui",
      controlTitle: "实时控制台",
      appsEyebrow: "应用中心",
      appsTitle: "应用中心",
      diagnosticsEyebrow: "诊断",
      diagnosticsTitle: "诊断",
    },
    topbar: { language: "语言", armType: "机械臂" },
    connection: { connected: "已连接", disconnected: "未连接" },
    button: {
      connectArm: "连接机械臂",
      refreshStatus: "刷新状态",
      disconnectArm: "断开连接",
      refreshScan: "刷新并扫描",
      fetchFeedback: "读取反馈",
      enableDrive: "使能机械臂",
      disableDrive: "失能机械臂",
      solveIk: "计算逆解",
      home: "初始位置",
      readRealPose: "获取真机位置",
      reading: "读取中...",
      sendTarget: "发送目标位置",
      gripperOpen: "打开夹爪",
      gripperClose: "闭合夹爪",
      notNow: "暂不连接",
      gotIt: "知道了",
      configureSerial: "配置串口权限",
      configuring: "配置中...",
      calibrateZero: "机械臂校准",
    },
    panel: {
      armPoseTitle: "机械臂姿态",
      armPoseSubtitle: "6 轴关节角度与末端趋势",
      manualControl: "手动控制",
      jointControl: "关节控制",
      ik: "运动学逆解",
      gripper: "夹爪控制",
      gripperNote: "夹爪控制仅在连接真实机械臂后激活；仿真环境中无法显示夹爪开合效果。",
    },
    camera: { side: "侧视", top: "俯视" },
    maintenance: {
      title: "页面正在维护",
      apps: "应用中心功能后续开发。",
      diagnostics: "诊断窗口暂时关闭，当前页面正在维护。",
    },
    appCenter: {
      title: "机械臂项目",
      subtitle: "精选 Seeed reBot Arm 项目，点击卡片可直接跳转到对应页面。",
      openProject: "查看项目",
      graspnet: {
        tag: "视觉抓取",
        title: "GraspNet Visual Grasping",
        desc: "基于 GraspNet 的视觉抓取方案，适合快速验证识别、抓取与机械臂联动流程。",
      },
      voice: {
        tag: "语音控制",
        title: "Voice Control reBot Arm",
        desc: "通过语音指令驱动机械臂动作，适合展示自然交互与执行链路整合能力。",
      },
      openclaw: {
        tag: "端侧智能体",
        title: "NemoClaw on Jetson Thor",
        desc: "结合 NVIDIA Jetson Thor 与 NemoClaw，构建更强的端侧机械臂控制体验。",
      },
      lerobot: {
        tag: "具身学习",
        title: "LeRobot for reBot Arm",
        desc: "围绕 LeRobot 训练与部署流程，展示机械臂数据采集、学习和复现能力。",
      },
    },
    diagnostics: {
      subtitle: "点击调试、实时状态与后端事件",
      summaryTitle: "状态摘要",
      actionsTitle: "点击调试",
      actionNote: "参考 MotorBridge 的使能与状态读取流程，适合快速验证串口、反馈和使能状态。",
      feedbackTitle: "电机调试",
      feedbackSubtitle: "为每个已扫描电机提供独立选项卡，点击后查看当前电机的完整信息。",
      motorTabsTitle: "电机列表",
      motorDetailTitle: "电机详情",
      statusIdle: "等待调试操作",
      actionRunning: "正在执行 {{action}}...",
      actionDone: "{{action}} 已完成",
      serviceCard: "后台服务",
      connectionCard: "连接状态",
      serialCard: "串口接口",
      feedbackCard: "电机反馈",
      enabledCard: "驱动使能",
      zeroCard: "零点",
      backendOnline: "在线",
      backendOffline: "离线",
      armConnected: "已连接",
      armDisconnected: "未连接",
      serialReady: "已检测",
      serialMissing: "未检测",
      feedbackReady: "{{count}} 路反馈",
      feedbackMissing: "无反馈",
      enabled: "已使能",
      disabled: "未使能",
      zeroCalibrated: "已重写",
      zeroPending: "未重写",
      zeroLastSet: "最后校准",
      permissionRequired: "需要串口权限",
      channel: "通道",
      lastScan: "最近扫描",
      calibrateConfirm: "当前姿态将被写入为机械臂 0 点，机械臂会短暂失能并重新使能。是否继续？",
      noMotors: "暂无电机反馈，请先点击“刷新并扫描”或“读取反馈”。",
      noMotorSelected: "请先选择一个电机选项卡。",
      clientLogs: "前端日志",
      backendLogs: "后端日志",
      noBackendLogs: "暂无后台日志",
      name: "名称",
      model: "型号",
      code: "状态码",
      position: "位置",
      velocity: "速度",
      torque: "扭矩",
      motorId: "电机 ID",
      feedbackId: "反馈 ID",
    },
    modal: {
      detectedTitle: "检测到真实机械臂",
      noArmTitle: "未检测到机械臂",
      detectedPrompt: "后台扫描到电机反馈，是否连接真实机械臂？",
      connectGuide: "检测到机械臂，请确认连接。",
      interfaceGuide: "已检测到机械臂通信接口，请确认机械臂已上电后再连接。",
      noArmPrompt: "未检测到机械臂，请先接入机械臂，再重新点击连接机械臂。",
      detectedAsk: "已读取到 {{count}} 个电机反馈，是否连接真实机械臂？",
      detectedWaiting: "已检测到 /dev/ttyACM*，正在等待机械臂反馈。请确认机械臂供电和通信线。",
      permissionHintDefault: "串口权限不足，请执行 sudo chmod 666 /dev/ttyACM* 后重试",
      sudoNeeded: "配置 CAN 接口需要 sudo 权限，请在下方输入系统密码后再次点击连接。",
      sudoWrongPassword: "sudo 密码错误，请重新输入。",
      sudoPasswordLabel: "sudo 密码（用于配置 CAN 接口）:",
      sudoPasswordPlaceholder: "请输入系统密码",
      sudoPasswordHint: "密码仅在本次会话内存中使用，不会保存或上传。",
      motorFallback: "motor",
    },
    overlay: {
      loadingUrdf: "正在加载 URDF 模型...",
      viewerInitFailed: "3D 视图初始化失败，控制功能仍可使用: {{error}}",
      loadingModel: "正在加载模型 {{loaded}}/{{total}}: {{file}}",
      urdfLoaded: "URDF 模型加载完成",
      urdfLoadFailed: "URDF 加载失败: {{error}}",
    },
    solver: {
      ready: "FK/IK 就绪",
      fkUpdated: "FK: Pinocchio 正解已更新 ({{source}})",
      fkFallback: "FK: 后端不可用，使用前端近似正解",
      fkMock: "FK: Mock 近似正解",
      ikSuccess: "IK: Pinocchio 逆解成功，误差 {{error}}",
      ikFallback: "IK: 目标不可达，已生成最近位姿，位置误差约 {{error}} m",
      readPoseSynced: "已同步真机位置到 J1-J6 和仿真模型",
      readPoseMissing: "未读取到真机关节位置",
      permissionConfigured: "串口权限配置完成，正在重新检测机械臂",
      connectSuccess: "真实机械臂连接成功",
      backendUnavailable: "后端服务不可用，请先启动机械臂服务后重试。",
    },
    source: {
      manual: "手动",
      modelLoad: "模型加载",
      init: "初始化",
      home: "初始位置",
    },
    mode: {
      positionControl: "位置控制",
      posVel: "位置-速度模式",
      mit: "MIT 力矩模式",
      estopLocked: "急停锁定",
    },
    panel: {
      controlMode: "控制模式",
      jointControl: "关节控制",
      ik: "运动学逆解",
      mitParams: "MIT 参数",
      mitKp: "Kp (刚度)",
      mitKd: "Kd (阻尼)",
      mitTau: "tau (前馈力矩)",
    },
  },
  "en-US": {
    brand: { title: "rebot arm webui", subtitle: "DevArm Console" },
    aria: {
      mainNav: "Main navigation",
      language: "Language",
      armType: "Arm type",
      viewMode: "View mode",
      armViewport: "Robot arm 3D URDF simulator",
    },
    language: { zhCN: "Chinese", enUS: "English" },
    nav: { control: "Control", apps: "Apps", diagnostics: "Diagnostics" },
    service: {
      label: "Backend Service",
      sameOrigin: "same-origin REST API",
      mock: "Mock Mode",
      backendOnline: "Backend Online",
      armDetected: "Arm Detected",
      commDetected: "Communication Device Detected",
      realConnected: "Real Arm Connected",
    },
    view: {
      controlEyebrow: "rebot arm webui",
      controlTitle: "Live Control",
      appsEyebrow: "App Center",
      appsTitle: "App Center",
      diagnosticsEyebrow: "Diagnostics",
      diagnosticsTitle: "Diagnostics",
    },
    topbar: { language: "Language", armType: "Arm" },
    connection: { connected: "Connected", disconnected: "Disconnected" },
    button: {
      connectArm: "Connect Arm",
      refreshStatus: "Refresh Status",
      disconnectArm: "Disconnect",
      refreshScan: "Refresh & Scan",
      fetchFeedback: "Fetch Feedback",
      enableDrive: "Enable Arm",
      disableDrive: "Disable Arm",
      solveIk: "Solve IK",
      home: "Home",
      readRealPose: "Read Real Pose",
      reading: "Reading...",
      sendTarget: "Send Target Pose",
      gripperOpen: "Open Gripper",
      gripperClose: "Close Gripper",
      notNow: "Not Now",
      gotIt: "Got It",
      configureSerial: "Grant Serial Access",
      configuring: "Granting...",
      calibrateZero: "Calibrate Zero",
    },
    panel: {
      armPoseTitle: "Robot Pose",
      armPoseSubtitle: "6-axis joints and end-effector trend",
      manualControl: "Manual Control",
      jointControl: "Joint Control",
      ik: "Inverse Kinematics",
      gripper: "Gripper Control",
      gripperNote: "Gripper control is enabled only when a real arm is connected; the simulator does not render open/close motion.",
    },
    camera: { side: "Side", top: "Top" },
    maintenance: {
      title: "Page Under Maintenance",
      apps: "App Center is under active development.",
      diagnostics: "Diagnostics is temporarily unavailable while this page is under maintenance.",
    },
    appCenter: {
      title: "Robot Arm Projects",
      subtitle: "A curated set of Seeed reBot Arm projects. Click any card to open its project page.",
      openProject: "Open Project",
      graspnet: {
        tag: "Visual Grasping",
        title: "GraspNet Visual Grasping",
        desc: "A GraspNet-based visual grasping workflow for quickly validating perception, grasping, and arm coordination.",
      },
      voice: {
        tag: "Voice Control",
        title: "Voice Control reBot Arm",
        desc: "Control the robot arm with voice commands to demonstrate natural interaction and execution flow integration.",
      },
      openclaw: {
        tag: "Edge Agent",
        title: "NemoClaw on Jetson Thor",
        desc: "Combine NVIDIA Jetson Thor with NemoClaw for a stronger on-device robot arm control experience.",
      },
      lerobot: {
        tag: "Embodied Learning",
        title: "LeRobot for reBot Arm",
        desc: "Focus on LeRobot training and deployment workflows for data collection, learning, and reproduction on the arm.",
      },
    },
    diagnostics: {
      subtitle: "Click-to-debug tools, live status, and backend events",
      summaryTitle: "Status Summary",
      actionsTitle: "Click Debug",
      actionNote: "Aligned with the MotorBridge enable and status workflow for fast checks of serial access, feedback, and drive enable.",
      feedbackTitle: "Motor Debug",
      feedbackSubtitle: "Each scanned motor gets its own tab so you can inspect the full live payload.",
      motorTabsTitle: "Motor Tabs",
      motorDetailTitle: "Motor Details",
      statusIdle: "Waiting for a debug action",
      actionRunning: "Running {{action}}...",
      actionDone: "{{action}} completed",
      serviceCard: "Backend",
      connectionCard: "Connection",
      serialCard: "Serial",
      feedbackCard: "Feedback",
      enabledCard: "Drive Enable",
      zeroCard: "Zero Point",
      backendOnline: "Online",
      backendOffline: "Offline",
      armConnected: "Connected",
      armDisconnected: "Disconnected",
      serialReady: "Detected",
      serialMissing: "Missing",
      feedbackReady: "{{count}} motors",
      feedbackMissing: "No feedback",
      enabled: "Enabled",
      disabled: "Disabled",
      zeroCalibrated: "Rewritten",
      zeroPending: "Not Set",
      zeroLastSet: "Last Calibration",
      permissionRequired: "Serial permission required",
      channel: "Channel",
      lastScan: "Last Scan",
      calibrateConfirm: "The current pose will be written as the arm zero point. The arm will be briefly disabled and then re-enabled. Continue?",
      noMotors: "No motor feedback yet. Try Refresh & Scan or Fetch Feedback.",
      noMotorSelected: "Select a motor tab first.",
      clientLogs: "Client Logs",
      backendLogs: "Backend Logs",
      noBackendLogs: "No backend logs yet",
      name: "Name",
      model: "Model",
      code: "Status",
      position: "Position",
      velocity: "Velocity",
      torque: "Torque",
      motorId: "Motor ID",
      feedbackId: "Feedback ID",
    },
    modal: {
      detectedTitle: "Real Arm Detected",
      noArmTitle: "No Arm Detected",
      detectedPrompt: "The backend found motor feedback. Connect the real arm?",
      connectGuide: "Arm detected. Confirm to connect the real arm.",
      interfaceGuide: "Detected the arm communication interface. Power on the arm, then confirm to connect.",
      noArmPrompt: "No robot arm was detected. Connect the arm first, then click Connect Arm again.",
      detectedAsk: "Received feedback from {{count}} motors. Connect the real arm?",
      detectedWaiting: "Detected /dev/ttyACM* and waiting for motor feedback. Check arm power and communication cable.",
      permissionHintDefault: "Serial permission is missing. Run sudo chmod 666 /dev/ttyACM* and try again.",
      sudoNeeded: "sudo privileges are required to configure CAN interfaces. Enter your system password below and click Connect again.",
      sudoWrongPassword: "Incorrect sudo password. Please try again.",
      sudoPasswordLabel: "sudo password (used to configure CAN interfaces):",
      sudoPasswordPlaceholder: "Enter system password",
      sudoPasswordHint: "The password is only used in memory for this session and is never stored or uploaded.",
      motorFallback: "motor",
    },
    overlay: {
      loadingUrdf: "Loading URDF model...",
      viewerInitFailed: "3D viewer initialization failed. Control functions remain available: {{error}}",
      loadingModel: "Loading model {{loaded}}/{{total}}: {{file}}",
      urdfLoaded: "URDF model loaded",
      urdfLoadFailed: "URDF load failed: {{error}}",
    },
    solver: {
      ready: "FK/IK ready",
      fkUpdated: "FK: Pinocchio forward kinematics updated ({{source}})",
      fkFallback: "FK: Backend unavailable, using frontend approximation",
      fkMock: "FK: Mock approximation",
      ikSuccess: "IK: Pinocchio solved, error {{error}}",
      ikFallback: "IK: Target unreachable. Generated the nearest pose with about {{error}} m position error",
      readPoseSynced: "Synced real arm pose to J1-J6 and the simulator",
      readPoseMissing: "Real arm joint pose unavailable",
      permissionConfigured: "Serial access granted. Detecting the arm again.",
      connectSuccess: "Real arm connected successfully",
      backendUnavailable: "Backend service unavailable. Start the arm service and try again.",
    },
    source: {
      manual: "Manual",
      modelLoad: "Model Load",
      init: "Initialization",
      home: "Home",
    },
    mode: {
      positionControl: "Position Control",
      posVel: "Position-Velocity Mode",
      mit: "MIT Torque Mode",
      estopLocked: "E-stop Locked",
    },
    panel: {
      controlMode: "Control Mode",
      jointControl: "Joint Control",
      ik: "Inverse Kinematics",
      mitParams: "MIT Parameters",
      mitKp: "Kp (Stiffness)",
      mitKd: "Kd (Damping)",
      mitTau: "tau (Feedforward Torque)",
    },
  },
};

function lookupTranslation(locale, key) {
  return key.split(".").reduce((value, segment) => (value && value[segment] !== undefined ? value[segment] : undefined), translations[locale]);
}

function formatTemplate(template, vars = {}) {
  return String(template).replace(/{{\s*(\w+)\s*}}/g, (_, key) => String(vars[key] ?? ""));
}

function normalizeLocale(locale) {
  if (SUPPORTED_LOCALES.includes(locale)) return locale;
  return String(locale || "").toLowerCase().startsWith("en") ? "en-US" : "zh-CN";
}

function getInitialLocale() {
  try {
    const saved = window.localStorage.getItem(LANGUAGE_STORAGE_KEY);
    if (saved) return normalizeLocale(saved);
  } catch {
  }
  return normalizeLocale(window.navigator.language || document.documentElement.lang || "zh-CN");
}

function t(key, vars = {}, locale = state.locale) {
  const template = lookupTranslation(locale, key) ?? lookupTranslation("zh-CN", key) ?? key;
  return formatTemplate(template, vars);
}

function resolveText(message) {
  if (message && typeof message === "object" && "key" in message) {
    return t(message.key, message.vars ?? {});
  }
  return String(message ?? "");
}

function translateSource(source) {
  const key = { manual: "source.manual", modelLoad: "source.modelLoad", init: "source.init", home: "source.home" }[source];
  return key ? t(key) : source;
}

state.locale = getInitialLocale();

window.__rebotArmDebug = {
  get robotLoaded() {
    return Boolean(state.robot);
  },
  get jointNames() {
    return state.robot ? Object.keys(state.robot.joints) : [];
  },
  get linkNames() {
    return state.robot ? Object.keys(state.robot.links) : [];
  },
  get sceneBounds() {
    if (!state.robot) return null;
    const box = new THREE.Box3().setFromObject(state.robot);
    return {
      min: box.min.toArray(),
      max: box.max.toArray(),
      center: box.getCenter(new THREE.Vector3()).toArray(),
      size: box.getSize(new THREE.Vector3()).toArray(),
      camera: camera.position.toArray(),
      target: controls.target.toArray(),
    };
  },
};

const els = {
  viewport: document.querySelector("#armViewport"),
  viewportOverlay: document.querySelector("#viewportOverlay"),
  pageEyebrow: document.querySelector("#pageEyebrow"),
  pageTitle: document.querySelector("#pageTitle"),
  systemActions: document.querySelector(".system-actions"),
  languageSelect: document.querySelector("#languageSelect"),
  armTypeSelect: document.querySelector("#armTypeSelect"),
  connectionStatus: document.querySelector("#connectionStatus"),
  sidebarModeBadge: document.querySelector("#sidebarModeBadge"),
  jointList: document.querySelector("#jointList"),
  controlModeSelector: document.querySelector("#controlModeSelector"),
  mitParamsPanel: document.querySelector("#mitParamsPanel"),
  mitParamsGrid: document.querySelector("#mitParamsGrid"),
  backendLogList: document.querySelector("#backendLogList"),
  diagnosticsStatus: document.querySelector("#diagnosticsStatus"),
  diagnosticsSummary: document.querySelector("#diagnosticsSummary"),
  diagnosticsMotorTabs: document.querySelector("#diagnosticsMotorTabs"),
  diagnosticsMotorDetail: document.querySelector("#diagnosticsMotorDetail"),
  refreshScanButton: document.querySelector("#refreshScanButton"),
  fetchFeedbackButton: document.querySelector("#fetchFeedbackButton"),
  enableDriveButton: document.querySelector("#enableDriveButton"),
  disableDriveButton: document.querySelector("#disableDriveButton"),
  calibrateZeroButton: document.querySelector("#calibrateZeroButton"),
  serviceState: document.querySelector("#serviceState"),
  serviceEndpoint: document.querySelector("#serviceEndpoint"),
  urdfPathText: document.querySelector("#urdfPathText"),
  enableButton: document.querySelector("#enableButton"),
  solveIkButton: document.querySelector("#solveIkButton"),
  readRealPoseButton: document.querySelector("#readRealPoseButton"),
  modeText: document.querySelector("#modeText"),
  tcpText: document.querySelector("#tcpText"),
  rpyText: document.querySelector("#rpyText"),
  loadText: document.querySelector("#loadText"),
  voltageText: document.querySelector("#voltageText"),
  tempText: document.querySelector("#tempText"),
  torqueText: document.querySelector("#torqueText"),
  errorText: document.querySelector("#errorText"),
  cartesianForm: document.querySelector("#cartesianForm"),
  solverStatus: document.querySelector("#solverStatus"),
  gripperOpenButton: document.querySelector("#gripperOpenButton"),
  gripperCloseButton: document.querySelector("#gripperCloseButton"),
  armDetectModal: document.querySelector("#armDetectModal"),
  armDetectTitle: document.querySelector("#armDetectTitle"),
  armDetectText: document.querySelector("#armDetectText"),
  detectedMotors: document.querySelector("#detectedMotors"),
  armDetectCancel: document.querySelector("#armDetectCancel"),
  serialPermissionButton: document.querySelector("#serialPermissionButton"),
  armDetectConfirm: document.querySelector("#armDetectConfirm"),
  sudoPasswordGroup: document.querySelector("#sudoPasswordGroup"),
  sudoPasswordInput: document.querySelector("#sudoPasswordInput"),
};

function setBadgeVariant(element, variant) {
  if (!element) return;
  element.classList.remove("badge-success", "badge-warning", "badge-error", "badge-info", "badge-inactive");
  element.classList.add(`badge-${variant}`);
}

function setButtonContent(button, icon, label) {
  if (!button) return;
  button.innerHTML = `<iconify-icon icon="${icon}" aria-hidden="true"></iconify-icon><span>${label}</span>`;
}

function setConnectionIndicator({ connected, label }) {
  if (!els.connectionStatus) return;
  els.connectionStatus.dataset.connected = String(connected);
  setBadgeVariant(els.connectionStatus, connected ? "success" : "inactive");
  els.connectionStatus.innerHTML = `
    <span class="status-dot" aria-hidden="true"></span>
    <iconify-icon icon="${connected ? "mingcute:check-circle-line" : "mingcute:close-circle-line"}" aria-hidden="true"></iconify-icon>
    <strong>${label}</strong>
  `;
}

function renderArmTypeOptions() {
  if (!els.armTypeSelect) return;
  const current = state.armType;
  els.armTypeSelect.innerHTML = state.armProfiles
    .map((profile) => `<option value="${escapeHtml(profile.type)}">${escapeHtml(profile.label || profile.type.toUpperCase())}</option>`)
    .join("");
  els.armTypeSelect.value = current;
}

function setSidebarModeBadge() {
  if (!els.sidebarModeBadge) return;

  if (!state.backendOnline) {
    setBadgeVariant(els.sidebarModeBadge, "inactive");
    els.sidebarModeBadge.innerHTML = `<iconify-icon icon="mingcute:cloud-off-line" aria-hidden="true"></iconify-icon><span>Mock</span>`;
    return;
  }

  if (state.connected) {
    setBadgeVariant(els.sidebarModeBadge, "success");
    els.sidebarModeBadge.innerHTML = `<iconify-icon icon="mingcute:check-circle-line" aria-hidden="true"></iconify-icon><span>Live</span>`;
    return;
  }

  if (state.interfacePresent) {
    setBadgeVariant(els.sidebarModeBadge, "warning");
    els.sidebarModeBadge.innerHTML = `<iconify-icon icon="mingcute:alert-line" aria-hidden="true"></iconify-icon><span>Detect</span>`;
    return;
  }

  setBadgeVariant(els.sidebarModeBadge, "info");
  els.sidebarModeBadge.innerHTML = `<iconify-icon icon="mingcute:ai-line" aria-hidden="true"></iconify-icon><span>REST API</span>`;
}

const viewMeta = {
  control: { eyebrowKey: "view.controlEyebrow", titleKey: "view.controlTitle" },
  apps: { eyebrowKey: "view.appsEyebrow", titleKey: "view.appsTitle" },
  diagnostics: { eyebrowKey: "view.diagnosticsEyebrow", titleKey: "view.diagnosticsTitle" },
};

function applyStaticTranslations() {
  document.documentElement.lang = state.locale;
  document.title = t("brand.title");
  document.querySelectorAll("[data-i18n]").forEach((node) => {
    node.textContent = t(node.dataset.i18n);
  });
  document.querySelectorAll("[data-i18n-aria-label]").forEach((node) => {
    node.setAttribute("aria-label", t(node.dataset.i18nAriaLabel));
  });
  document.querySelectorAll("[data-i18n-placeholder]").forEach((node) => {
    node.setAttribute("placeholder", t(node.dataset.i18nPlaceholder));
  });
  els.languageSelect.value = state.locale;
  renderArmTypeOptions();
}

function animateLanguageSwitch() {
  const shell = document.querySelector(".app-shell");
  if (!shell) return;
  shell.classList.add("is-language-switching");
  window.clearTimeout(animateLanguageSwitch.timerId);
  animateLanguageSwitch.timerId = window.setTimeout(() => {
    shell.classList.remove("is-language-switching");
  }, 180);
}

function updateViewHeader() {
  const meta = viewMeta[state.activeView] ?? viewMeta.control;
  els.pageEyebrow.textContent = "REBOT ARM WEBUI";
  els.pageTitle.textContent = t(meta.titleKey);
}

function applyTranslations({ animate = false } = {}) {
  if (animate) animateLanguageSwitch();
  applyStaticTranslations();
  updateViewHeader();
  renderSolverStatus();
  setOverlay(state.overlay.message, state.overlay.persistent);
  updateConnectionUi();
  if (!els.armDetectModal.hidden) showArmDetectModal();
}

function setLanguage(locale) {
  const nextLocale = normalizeLocale(locale);
  if (nextLocale === state.locale) {
    applyTranslations();
    return;
  }
  state.locale = nextLocale;
  try {
    window.localStorage.setItem(LANGUAGE_STORAGE_KEY, nextLocale);
  } catch {
  }
  applyTranslations({ animate: true });
}

const scene = new THREE.Scene();
scene.background = scenePalette.viewportBackground;

const camera = new THREE.PerspectiveCamera(45, 1, 0.01, 100);
camera.up.set(0, 0, 1);
camera.position.set(0.62, -0.78, 0.48);

let renderer = null;
let controls = {
  target: new THREE.Vector3(0, 0, 0.2),
  update() {},
};

try {
  renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  els.viewport.append(renderer.domElement);

  controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(0, 0, 0.2);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;
  controls.minDistance = 0.25;
  controls.maxDistance = 2.8;
  controls.screenSpacePanning = false;
} catch (error) {
  setOverlay({ key: "overlay.viewerInitFailed", vars: { error: error.message } }, true);
}

const ambient = new THREE.HemisphereLight(scenePalette.hemisphereSky, scenePalette.hemisphereGround, 2.2);
scene.add(ambient);

const keyLight = new THREE.DirectionalLight(0xffffff, 2.4);
keyLight.position.set(1.2, -1.4, 1.8);
keyLight.castShadow = true;
keyLight.shadow.mapSize.set(2048, 2048);
scene.add(keyLight);

const fillLight = new THREE.DirectionalLight(scenePalette.fillLight, 0.9);
fillLight.position.set(-1.2, 1.1, 0.8);
scene.add(fillLight);

const grid = new THREE.GridHelper(1.2, 24, scenePalette.gridMajor, scenePalette.gridMinor);
grid.rotation.x = Math.PI / 2;
scene.add(grid);

const floor = new THREE.Mesh(
  new THREE.CircleGeometry(0.72, 96),
  new THREE.MeshStandardMaterial({ color: scenePalette.floor, roughness: 0.74, metalness: 0.05 }),
);
floor.receiveShadow = true;
floor.position.z = -0.004;
scene.add(floor);

const tcpMarker = new THREE.Mesh(
  new THREE.SphereGeometry(0.014, 24, 16),
  new THREE.MeshStandardMaterial({ color: scenePalette.tcpMarker, roughness: 0.42, metalness: 0.08 }),
);
scene.add(tcpMarker);

const tempPosition = new THREE.Vector3();
const tempQuaternion = new THREE.Quaternion();
const tempScale = new THREE.Vector3();
const tempEuler = new THREE.Euler();
const tempBox = new THREE.Box3();
const tempCorner = new THREE.Vector3();
const tempTcpWorld = new THREE.Vector3();

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

function degToRad(value) {
  return (Number(value) * Math.PI) / 180;
}

function formatAngle(value) {
  return `${Number(value).toFixed(1)}°`;
}

function radToDeg(value) {
  return (Number(value) * 180) / Math.PI;
}

function formatNumber(value, digits = 3) {
  return Number(value).toFixed(digits);
}

function getJointRadians() {
  return joints.map((joint) => degToRad(joint.angle));
}

function setJointRadians(jointsRad) {
  if (!Array.isArray(jointsRad) || jointsRad.length < joints.length) return;
  joints.forEach((joint, index) => {
    joint.angle = clamp(radToDeg(jointsRad[index]), joint.min, joint.max);
  });
  syncJointInputs();
  updateRobotJoints();
  updateDerivedState();
}

function readCartesianInputs() {
  const values = {};
  els.cartesianForm.querySelectorAll("input").forEach((input) => {
    values[input.dataset.axis] = Number(input.value);
  });
  return {
    position_m: [values.x, values.y, values.z],
    rpy_rad: [degToRad(values.roll), degToRad(values.pitch), degToRad(values.yaw)],
  };
}

function writeCartesianInputs(pose, { markApprox = false } = {}) {
  if (!pose?.position_m || !pose?.rpy_rad) return;

  const [x, y, z] = pose.position_m;
  const [roll, pitch, yaw] = pose.rpy_rad;
  const values = { x, y, z, roll: radToDeg(roll), pitch: radToDeg(pitch), yaw: radToDeg(yaw) };

  state.suppressCartesianSolve = true;
  els.cartesianForm.querySelectorAll("input").forEach((input) => {
    const axis = input.dataset.axis;
    const digits = axis === "x" || axis === "y" || axis === "z" ? 3 : 1;
    input.value = formatNumber(values[axis], digits);
    input.classList.toggle("approx", markApprox);
  });
  queueMicrotask(() => {
    state.suppressCartesianSolve = false;
  });
}

function writePoseSummary(pose) {
  if (!pose?.position_m || !pose?.rpy_rad) return;
  if (els.tcpText) els.tcpText.textContent = `${pose.position_m.map((value) => Number(value).toFixed(2)).join(", ")} m`;
  if (els.rpyText) els.rpyText.textContent = `${pose.rpy_rad.map((value) => Number(radToDeg(value)).toFixed(1)).join(", ")}°`;
}

function renderSolverStatus() {
  els.solverStatus.textContent = resolveText(state.solverStatus.message);
  els.solverStatus.classList.toggle("warn", state.solverStatus.warn);
}

function setSolverStatus(message, warn = false) {
  state.solverStatus = { message, warn };
  renderSolverStatus();
}

async function apiJson(path, options = {}) {
  const controller = new AbortController();
  const timeout = window.setTimeout(() => controller.abort(), options.timeoutMs ?? 4500);
  let requestPath = path;
  const includeArmType = options.includeArmType !== false;
  let body = options.body;
  if (includeArmType) {
    if (options.method && options.method !== "GET") {
      body = { ...(body ?? {}), arm_type: state.armType };
    } else {
      const separator = requestPath.includes("?") ? "&" : "?";
      requestPath = `${requestPath}${separator}arm_type=${encodeURIComponent(state.armType)}`;
    }
  }

  try {
    const response = await fetch(requestPath, {
      method: options.method ?? "GET",
      headers: {
        "Content-Type": "application/json",
        ...(options.headers ?? {}),
      },
      body: body === undefined ? undefined : JSON.stringify(body),
      signal: controller.signal,
    });

    if (!response.ok) {
      const text = await response.text();
      throw new Error(text || `${response.status} ${response.statusText}`);
    }

    const contentType = response.headers.get("content-type") ?? "";
    if (!contentType.includes("json")) {
      throw new Error(`expected JSON response from ${requestPath}`);
    }
    return response.json();
  } finally {
    window.clearTimeout(timeout);
  }
}

function applyBackendStatus(online) {
  state.backendOnline = online;
  state.apiMode = online ? "backend" : "mock";
  els.serviceState.textContent = online
    ? state.connected
      ? t("service.realConnected")
      : state.armDetected
        ? t("service.armDetected")
        : t("service.backendOnline")
    : t("service.mock");
  els.serviceEndpoint.textContent = online ? window.location.origin : t("service.sameOrigin");
}

async function loadSimConfig() {
  try {
    const config = await apiJson("/sim/config", { timeoutMs: 1800 });
    const packageAlias = config.package_alias ?? "reBot-DevArm_description_fixend";
    const configArmType = config.arm_type ?? state.armType;
    const localFallback = fallbackSimConfigs[configArmType] ?? fallbackSimConfig;
    const packageUrl = (config.package_url ?? localFallback.packageMap[packageAlias] ?? `/assets/urdf/package/${packageAlias}`).replace(/\/+$/, "");
    state.armType = config.arm_type ?? state.armType;
    state.armProfiles = Array.isArray(config.arm_profiles) && config.arm_profiles.length ? config.arm_profiles : state.armProfiles;
    
    // 根据机械臂类型切换关节配置
    if (state.armType === "rs") {
      joints = rsJoints.map(j => ({ ...j }));
    } else {
      joints = dmJoints.map(j => ({ ...j }));
    }
    
    state.simConfig = {
      armType: config.arm_type ?? state.armType,
      armLabel: config.arm_label ?? state.armType.toUpperCase(),
      urdfUrl: config.urdf_url ?? localFallback.urdfUrl,
      packageMap: {
        [packageAlias]: packageUrl,
      },
      jointNameMap: config.joint_name_map ?? localFallback.jointNameMap,
      endEffectorLink: config.end_effector_link ?? localFallback.endEffectorLink,
      tcpAxis: config.tcp_axis ?? localFallback.tcpAxis,
    };
    joints.forEach((joint) => {
      joint.urdfName = state.simConfig.jointNameMap[joint.name] ?? joint.urdfName;
    });
    applyBackendStatus(true);
    if (els.urdfPathText) els.urdfPathText.textContent = state.simConfig.urdfUrl;
    renderArmTypeOptions();
    addLog("已读取后端仿真配置");
    return;
  } catch {
    const localFallback = fallbackSimConfigs[state.armType] ?? fallbackSimConfig;
    state.simConfig = { ...localFallback, armType: state.armType };
    
    // 根据机械臂类型切换关节配置
    if (state.armType === "rs") {
      joints = rsJoints.map(j => ({ ...j }));
    } else {
      joints = dmJoints.map(j => ({ ...j }));
    }
    
    joints.forEach((joint) => {
      joint.urdfName = state.simConfig.jointNameMap[joint.name] ?? joint.name;
    });
    applyBackendStatus(false);
    if (els.urdfPathText) els.urdfPathText.textContent = state.simConfig.urdfUrl;
    renderArmTypeOptions();
    addLog("未检测到后端，使用本地 URDF 资源");
  }
}

function addLog(message) {
  if (!els.logList) return;
  const item = document.createElement("div");
  item.className = "log-item";
  item.innerHTML = `<strong>${message}</strong><time>${new Date().toLocaleTimeString("zh-CN", { hour12: false })}</time>`;
  els.logList.prepend(item);

  while (els.logList.children.length > 80) {
    els.logList.lastElementChild.remove();
  }
}

function escapeHtml(value) {
  return String(value ?? "")
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;")
    .replaceAll("'", "&#39;");
}

function formatClock(value) {
  if (!value) return "--";
  if (typeof value === "number") {
    return new Date(value * 1000).toLocaleTimeString(state.locale, { hour12: false });
  }
  return String(value);
}

function formatMetric(value, digits = 3, suffix = "") {
  if (typeof value !== "number" || Number.isNaN(value)) return "--";
  return `${value.toFixed(digits)}${suffix}`;
}

function renderBackendLogs(events = []) {
  if (!els.backendLogList) return;
  if (!events.length) {
    els.backendLogList.innerHTML = `<div class="log-item"><strong>${t("diagnostics.noBackendLogs")}</strong><time>--</time></div>`;
    return;
  }
  els.backendLogList.innerHTML = events
    .map((event) => {
      const detail = event.data && Object.keys(event.data).length
        ? `<pre>${JSON.stringify(event.data, null, 2)}</pre>`
        : "";
      return `
        <div class="log-item">
          <strong>${event.step || "backend"}: ${event.message || ""}</strong>
          <time>${event.time || "--"}</time>
          ${detail}
        </div>
      `;
    })
    .join("");
}

async function refreshBackendLogs() {
  if (!els.backendLogList) return;
  try {
    const result = await apiJson("/logs", { timeoutMs: 1800 });
    renderBackendLogs(Array.isArray(result.events) ? result.events : []);
  } catch {
    renderBackendLogs([]);
  }
}

function getDiagnosticsMotors() {
  const motors = Array.isArray(state.detectedMotors) ? [...state.detectedMotors] : [];
  const seenFeedbackIds = new Set(
    motors
      .map((motor) => Number(motor?.feedback_id))
      .filter((feedbackId) => Number.isFinite(feedbackId))
  );
  const gripperState = state.lastHealth?.gripper_state;
  const gripperFeedbackId = Number(state.lastHealth?.gripper_feedback_id);
  if (gripperState && (!Number.isFinite(gripperFeedbackId) || !seenFeedbackIds.has(gripperFeedbackId))) {
    motors.push({
      name: String(state.lastHealth?.gripper_name || "gripper"),
      model: state.lastHealth?.gripper_model || "--",
      motor_id: state.lastHealth?.gripper_motor_id,
      feedback_id: state.lastHealth?.gripper_feedback_id,
      status_code: gripperState.status_code,
      pos: gripperState.pos,
      vel: gripperState.vel,
      torq: gripperState.torq,
      vendor: state.lastHealth?.gripper_vendor,
      linked_joint: state.lastHealth?.gripper_motor_id ? `joint${Number(state.lastHealth.gripper_motor_id)}` : "",
      role: "gripper",
    });
  }
  return motors;
}

function renderDiagnostics() {
  if (els.diagnosticsStatus) {
    const busyLabel = state.diagnosticsBusyAction
      ? t("diagnostics.actionRunning", { action: state.diagnosticsBusyLabel || state.diagnosticsBusyAction })
      : resolveText(state.diagnosticsStatusMessage);
    els.diagnosticsStatus.textContent = busyLabel;
    els.diagnosticsStatus.classList.toggle("warn", Boolean(state.permissionRequired || state.lastHealth?.error));
  }

  if (els.diagnosticsSummary) {
    const cards = [
      {
        tone: state.backendOnline ? "success" : "inactive",
        label: t("diagnostics.serviceCard"),
        value: state.backendOnline ? t("diagnostics.backendOnline") : t("diagnostics.backendOffline"),
        meta: state.permissionRequired ? t("diagnostics.permissionRequired") : window.location.origin,
      },
      {
        tone: state.connected ? "success" : "inactive",
        label: t("diagnostics.connectionCard"),
        value: state.connected ? t("diagnostics.armConnected") : t("diagnostics.armDisconnected"),
        meta: t("diagnostics.enabledCard") + " · " + (state.enabled ? t("diagnostics.enabled") : t("diagnostics.disabled")),
      },
      {
        tone: state.interfacePresent ? "info" : "inactive",
        label: t("diagnostics.serialCard"),
        value: state.interfacePresent ? t("diagnostics.serialReady") : t("diagnostics.serialMissing"),
        meta: `${t("diagnostics.channel")}: ${state.lastHealth?.channel ?? "--"}`,
      },
      {
        tone: state.detectedMotors.length ? "success" : state.interfacePresent ? "warning" : "inactive",
        label: t("diagnostics.feedbackCard"),
        value: state.detectedMotors.length ? t("diagnostics.feedbackReady", { count: state.detectedMotors.length }) : t("diagnostics.feedbackMissing"),
        meta: `${t("diagnostics.lastScan")}: ${formatClock(state.lastHealth?.last_scan_at)}`,
      },
      {
        tone: state.lastHealth?.zero_calibrated ? "success" : "info",
        label: t("diagnostics.zeroCard"),
        value: state.lastHealth?.zero_calibrated ? t("diagnostics.zeroCalibrated") : t("diagnostics.zeroPending"),
        meta: `${t("diagnostics.zeroLastSet")}: ${formatClock(state.lastHealth?.zero_last_set_at)}`,
      },
    ];

    els.diagnosticsSummary.innerHTML = cards.map((card) => `
      <article class="diagnostics-card diagnostics-card-${card.tone}">
        <span class="diagnostics-card-label">${escapeHtml(card.label)}</span>
        <strong>${escapeHtml(card.value)}</strong>
        <span class="diagnostics-card-meta">${escapeHtml(card.meta)}</span>
      </article>
    `).join("");
  }

  const diagnosticsMotors = getDiagnosticsMotors();
  const motorNames = diagnosticsMotors.map((motor) => String(motor.name ?? ""));
  if (!motorNames.includes(state.selectedMotorName)) {
    state.selectedMotorName = motorNames[0] ?? "";
  }

  if (els.diagnosticsMotorTabs) {
    if (!diagnosticsMotors.length) {
      els.diagnosticsMotorTabs.innerHTML = `<div class="diagnostics-empty">${escapeHtml(t("diagnostics.noMotors"))}</div>`;
    } else {
      els.diagnosticsMotorTabs.innerHTML = diagnosticsMotors.map((motor) => {
        const name = String(motor.name ?? "motor");
        const active = name === state.selectedMotorName;
        const subtitle = motor.role === "gripper"
          ? (motor.linked_joint ? `${motor.model ?? "--"} · ${motor.linked_joint}` : String(motor.model ?? "--"))
          : (motor.alias ? `${motor.model ?? "--"} · ${motor.alias}` : String(motor.model ?? "--"));
        return `
          <button class="motor-tab${active ? " active" : ""}" type="button" data-motor-tab="${escapeHtml(name)}">
            <strong>${escapeHtml(name)}</strong>
            <span>${escapeHtml(subtitle)}</span>
          </button>
        `;
      }).join("");
    }
  }

  if (els.diagnosticsMotorDetail) {
    const currentMotor = diagnosticsMotors.find((motor) => String(motor.name ?? "") === state.selectedMotorName);
    if (!currentMotor) {
      els.diagnosticsMotorDetail.innerHTML = `<div class="diagnostics-empty">${escapeHtml(t("diagnostics.noMotorSelected"))}</div>`;
    } else {
      const detailEntries = [
        [t("diagnostics.name"), currentMotor.name],
        [t("diagnostics.model"), currentMotor.model],
        [t("diagnostics.motorId"), `0x${Number(currentMotor.motor_id ?? 0).toString(16).padStart(2, "0")}`],
        [t("diagnostics.feedbackId"), `0x${Number(currentMotor.feedback_id ?? 0).toString(16).padStart(2, "0")}`],
        [t("diagnostics.code"), currentMotor.status_code],
        [t("diagnostics.position"), formatMetric(Number(currentMotor.pos), 6, " rad")],
        [t("diagnostics.velocity"), formatMetric(Number(currentMotor.vel), 6, " rad/s")],
        [t("diagnostics.torque"), formatMetric(Number(currentMotor.torq), 6, " Nm")],
      ];
      Object.entries(currentMotor).forEach(([key, value]) => {
        const knownKeys = new Set(["name", "model", "motor_id", "feedback_id", "status_code", "pos", "vel", "torq"]);
        if (knownKeys.has(key)) return;
        detailEntries.push([key, typeof value === "object" ? JSON.stringify(value) : value]);
      });
      els.diagnosticsMotorDetail.innerHTML = `
        <div class="motor-detail-card">
          <div class="motor-detail-head">
            <strong>${escapeHtml(String(currentMotor.name ?? "motor"))}</strong>
            <span class="badge ${Number(currentMotor.status_code ?? 0) > 0 ? "badge-success" : "badge-inactive"}">${escapeHtml(t("diagnostics.code"))}: ${escapeHtml(currentMotor.status_code ?? "--")}</span>
          </div>
          <dl class="motor-detail-grid">
            ${detailEntries.map(([label, value]) => `
              <div>
                <dt>${escapeHtml(label)}</dt>
                <dd>${escapeHtml(value ?? "--")}</dd>
              </div>
            `).join("")}
          </dl>
        </div>
      `;
    }
  }

  const actionButtons = [
    ["refreshScan", els.refreshScanButton, "button.refreshScan"],
    ["feedback", els.fetchFeedbackButton, "button.fetchFeedback"],
    ["enable", els.enableDriveButton, "button.enableDrive"],
    ["disable", els.disableDriveButton, "button.disableDrive"],
    ["calibrate", els.calibrateZeroButton, "button.calibrateZero"],
  ];
  actionButtons.forEach(([action, button, labelKey]) => {
    if (!button) return;
    const unavailable = !state.backendOnline && action !== "refreshScan";
    const redundantEnable = action === "enable" && state.enabled;
    const redundantDisable = action === "disable" && !state.enabled;
    const requiresConnection = action === "calibrate" && !state.connected;
    button.disabled = Boolean(state.diagnosticsBusyAction) || unavailable || redundantEnable || redundantDisable || requiresConnection;
    button.dataset.action = action;
    button.classList.toggle("is-busy", state.diagnosticsBusyAction === action);
    if (!button.dataset.ready) {
      button.dataset.ready = "true";
    }
    button.querySelector("span")?.replaceChildren(document.createTextNode(t(labelKey)));
  });
}

function setOverlay(message, persistent = false) {
  state.overlay = { message, persistent };
  els.viewportOverlay.textContent = resolveText(message);
  els.viewportOverlay.hidden = Boolean(renderer) && !persistent && state.robot;
}

function resizeRenderer() {
  if (!renderer) return;
  if (state.activeView !== "control") return;
  const { clientWidth, clientHeight } = els.viewport;
  if (!clientWidth || !clientHeight) return;

  camera.aspect = clientWidth / clientHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(clientWidth, clientHeight, false);
}

function normalizeRobot(robot) {
  robot.rotation.set(0, 0, 0);
  robot.traverse((child) => {
    if (child.isMesh) {
      child.castShadow = true;
      child.receiveShadow = true;
      child.material = new THREE.MeshStandardMaterial({
        color: child.material?.color ?? new THREE.Color(scenePalette.robotMesh),
        roughness: 0.58,
        metalness: 0.22,
      });
      child.geometry.computeVertexNormals();
    }
  });

  robot.updateMatrixWorld(true);
  const box = new THREE.Box3().setFromObject(robot);
  const center = new THREE.Vector3();
  box.getCenter(center);
  robot.position.set(-center.x, -center.y, -box.min.z + 0.018);
  robot.updateMatrixWorld(true);

  const fittedBox = new THREE.Box3().setFromObject(robot);
  const fittedCenter = new THREE.Vector3();
  const fittedSize = new THREE.Vector3();
  fittedBox.getCenter(fittedCenter);
  fittedBox.getSize(fittedSize);
  const radius = Math.max(fittedSize.x, fittedSize.y, fittedSize.z, 0.35);
  controls.target.set(fittedCenter.x, fittedCenter.y, fittedCenter.z + fittedSize.z * 0.12);
  camera.position.set(fittedCenter.x + radius * 0.92, fittedCenter.y - radius * 1.22, fittedCenter.z + radius * 0.74);
  camera.lookAt(controls.target);
  controls.update();
}

function updateRobotJoints() {
  if (!state.robot) {
    // 兜底：如果 joint 数组有数据但 3D 视图未加载，尝试重新加载 URDF 模型
    if (!updateRobotJoints.reloading && joints?.length && state.simConfig?.urdfUrl) {
      updateRobotJoints.reloading = true;
      loadRobotModel();
      setTimeout(() => { updateRobotJoints.reloading = false; }, 500);
    }
    // 即使没有 robot 也继续走完后面的逻辑，让关节值在重新加载后能立即应用
  }

  joints.forEach((joint) => {
    if (state.robot?.joints?.[joint.urdfName]) {
      state.robot.setJointValue(joint.urdfName, degToRad(joint.angle));
    }
  });

  if (state.robot) {
    state.robot.updateMatrixWorld(true);
    const endEffectorLink = state.simConfig.endEffectorLink ?? "end_link";
    const endLink = state.robot.links?.[endEffectorLink];
    if (endLink) {
      getTcpWorldPosition(endLink, tcpMarker.position);
    }

    const pose = getUrdfEndEffectorPose();
    if (pose) {
      state.lastPose = pose;
    }
  }
}

function computeTcpLocalOffset() {
  const endEffectorLink = state.simConfig.endEffectorLink ?? "end_link";
  if (!state.robot?.links?.[endEffectorLink]) return;
  const endLink = state.robot.links[endEffectorLink];
  state.robot.updateMatrixWorld(true);
  endLink.updateMatrixWorld(true);
  tempBox.setFromObject(endLink);

  if (tempBox.isEmpty()) {
    state.tcpLocalOffset = new THREE.Vector3(0, 0, 0);
    return;
  }

  const localMin = new THREE.Vector3(Infinity, Infinity, Infinity);
  const localMax = new THREE.Vector3(-Infinity, -Infinity, -Infinity);
  for (const x of [tempBox.min.x, tempBox.max.x]) {
    for (const y of [tempBox.min.y, tempBox.max.y]) {
      for (const z of [tempBox.min.z, tempBox.max.z]) {
        tempCorner.set(x, y, z);
        endLink.worldToLocal(tempCorner);
        localMin.min(tempCorner);
        localMax.max(tempCorner);
      }
    }
  }

  const tcpX = state.simConfig.tcpAxis === "min_x" ? localMin.x : localMax.x;
  state.tcpLocalOffset = new THREE.Vector3(
    tcpX,
    (localMin.y + localMax.y) * 0.5,
    (localMin.z + localMax.z) * 0.5,
  );
}

function getTcpWorldPosition(endLink, target) {
  const localOffset = state.tcpLocalOffset ?? new THREE.Vector3(0, 0, 0);
  target.copy(localOffset);
  endLink.localToWorld(target);
  return target;
}

function loadRobotModel() {
  if (state.robot) {
    scene.remove(state.robot);
    state.robot.traverse((child) => {
      if (child.isMesh) {
        child.geometry?.dispose?.();
        if (Array.isArray(child.material)) {
          child.material.forEach((material) => material.dispose?.());
        } else {
          child.material?.dispose?.();
        }
      }
    });
    state.robot = null;
  }
  state.tcpLocalOffset = null;
  setOverlay({ key: "overlay.loadingUrdf" }, true);

  const manager = new THREE.LoadingManager();
  const loader = new URDFLoader(manager);
  const stlLoader = new STLLoader(manager);

  loader.packages = state.simConfig.packageMap;
  loader.loadMeshCb = (path, meshManager, done) => {
    stlLoader.manager = meshManager;
    stlLoader.load(
      path,
      (geometry) => {
        const mesh = new THREE.Mesh(
          geometry,
          new THREE.MeshStandardMaterial({ color: scenePalette.robotMesh, roughness: 0.58, metalness: 0.2 }),
        );
        done(mesh);
      },
      undefined,
      (error) => done(null, error),
    );
  };

  manager.onProgress = (url, loaded, total) => {
    setOverlay({ key: "overlay.loadingModel", vars: { loaded, total, file: url.split("/").pop() } }, true);
  };

  manager.onLoad = () => {
    if (state.robot) {
      normalizeRobot(state.robot);
      computeTcpLocalOffset();
      updateRobotJoints();
      updateDerivedState();
      solveFkRealtime("modelLoad");
      setOverlay({ key: "overlay.urdfLoaded" });
      addLog("URDF 模型加载完成");
    }
  };

  loader.load(
    state.simConfig.urdfUrl,
    (robot) => {
      state.robot = robot;
      scene.add(robot);
    },
    undefined,
    (error) => {
      setOverlay({ key: "overlay.urdfLoadFailed", vars: { error: error.message } }, true);
      addLog("URDF 模型加载失败");
    },
  );
}

async function changeArmType(armType) {
  const nextType = String(armType || "dm").toLowerCase();
  if (nextType === state.armType && state.robot) return;
  state.armType = nextType;
  
  // 根据机械臂类型切换关节配置
  if (nextType === "rs") {
    joints = rsJoints.map(j => ({ ...j }));
  } else {
    joints = dmJoints.map(j => ({ ...j }));
  }
  
  renderArmTypeOptions();
  try {
    const profile = await apiJson("/profile", {
      method: "POST",
      body: { arm_type: nextType },
      timeoutMs: 3000,
    });
    applyRealtimePayload(profile);
  } catch {
    applyBackendStatus(false);
  }
  await loadSimConfig();
  loadRobotModel();
  renderJoints();  // 重新渲染关节控件以应用新的限位
  updateDerivedState();
  await solveFkRealtime("modelLoad");
}

function renderJoints() {
  els.jointList.innerHTML = "";

  joints.forEach((joint, index) => {
    const row = document.createElement("div");
    row.className = "joint-row";
    row.innerHTML = `
      <strong>J${index + 1}</strong>
      <input type="range" min="${joint.min}" max="${joint.max}" step="0.1" value="${joint.angle}" aria-label="${joint.name}" />
      <span class="joint-value">${formatAngle(joint.angle)}</span>
    `;

    const input = row.querySelector("input");
    const value = row.querySelector(".joint-value");
    input.addEventListener("input", () => {
      joint.angle = Number(input.value);
      value.textContent = formatAngle(joint.angle);
      updateRobotJoints();
      updateDerivedState();
      solveFkRealtime(`J${index + 1}`);
    });

    input.addEventListener("change", () => addLog(`${joint.name} 目标角度 ${formatAngle(joint.angle)}`));
    els.jointList.append(row);
  });
}

function syncJointInputs() {
  els.jointList.querySelectorAll(".joint-row").forEach((row, index) => {
    const input = row.querySelector("input");
    const value = row.querySelector(".joint-value");
    input.value = joints[index].angle;
    value.textContent = formatAngle(joints[index].angle);
  });
}

function updateConnectionUi() {
  els.serviceState.textContent = state.backendOnline
    ? state.connected ? t("service.realConnected") : state.interfacePresent ? t("service.commDetected") : t("service.backendOnline")
    : t("service.mock");
  els.serviceEndpoint.textContent = state.backendOnline ? window.location.origin : t("service.sameOrigin");
  setConnectionIndicator({
    connected: state.connected,
    label: state.connected ? t("connection.connected") : t("connection.disconnected"),
  });
  setSidebarModeBadge();
  setButtonContent(
    els.enableButton,
    state.connected ? "mingcute:link-unlink-line" : "mingcute:plug-line",
    state.connected ? t("button.disconnectArm") : t("button.connectArm"),
  );
  els.enableButton.disabled = state.connected ? false : state.estop;
  if (els.readRealPoseButton) {
    els.readRealPoseButton.disabled = !state.connected;
    if (!els.readRealPoseButton.dataset.loading) {
      setButtonContent(els.readRealPoseButton, "mingcute:radar-line", t("button.readRealPose"));
    }
  }
  if (els.modeText) els.modeText.textContent = state.estop ? t("mode.estopLocked") : t(`mode.${state.mode}`);
  if (els.gripperOpenButton) els.gripperOpenButton.disabled = !state.connected || state.gripperBusy;
  if (els.gripperCloseButton) els.gripperCloseButton.disabled = !state.connected || state.gripperBusy;
  setButtonContent(els.solveIkButton, "mingcute:ai-line", t("button.solveIk"));
  setButtonContent(document.querySelector("#homeButton"), "mingcute:home-3-line", t("button.home"));
  setButtonContent(document.querySelector("#sendTargetButton"), "mingcute:send-plane-line", t("button.sendTarget"));
  setButtonContent(els.gripperOpenButton, "mingcute:open-door-line", t("button.gripperOpen"));
  setButtonContent(els.gripperCloseButton, "mingcute:lock-line", t("button.gripperClose"));
  renderDiagnostics();
}

function applyArmDetection(payload) {
  const wasDetected = state.armDetected;
  const hadInterface = state.interfacePresent;
  state.armDetected = Boolean(payload?.arm_detected);
  state.interfacePresent = Boolean(payload?.interface_present);
  state.detectedMotors = Array.isArray(payload?.detected_motors) ? payload.detected_motors : [];
  state.permissionRequired = Boolean(payload?.permission_required);
  state.permissionHint = payload?.permission_hint || "";
  state.sudoPasswordRequired = Boolean(payload?.sudo_password_required);
  const interfaceDetected = !hadInterface && state.interfacePresent;
  const armDetected = !wasDetected && state.armDetected;

  if (!state.interfacePresent) {
    state.armPromptDismissed = false;
    if (state.armModalMode !== "missing") hideArmDetectModal();
  } else if (interfaceDetected || armDetected) {
    state.armPromptDismissed = false;
  }

}

function applyRealtimePayload(payload, { robotState = false } = {}) {
  if (!payload) return;
  state.lastHealth = payload;
  if (robotState) state.lastRobotState = payload;
  if (!robotState && !payload.connected) state.lastRobotState = null;
  applyArmDetection(payload);
  applyBackendStatus(true);
  
  const prevArmType = state.armType;
  if (payload.arm_type) state.armType = payload.arm_type;
  
  // 如果 arm_type 变了，更新关节限位
  if (state.armType !== prevArmType) {
    if (state.armType === "rs") {
      joints = rsJoints.map(j => ({ ...j }));
    } else {
      joints = dmJoints.map(j => ({ ...j }));
    }
    renderJoints();  // 重新渲染控件
  }
  
  if (Array.isArray(payload.arm_profiles) && payload.arm_profiles.length) state.armProfiles = payload.arm_profiles;
  renderArmTypeOptions();
  state.connected = Boolean(payload.connected);
  state.enabled = "enabled" in payload ? Boolean(payload.enabled) : state.connected;
  if (payload.control_mode) {
    state.controlMode = payload.control_mode;
    const radio = els.controlModeSelector?.querySelector(`input[value="${payload.control_mode}"]`);
    if (radio) radio.checked = true;
    if (els.mitParamsPanel) {
      els.mitParamsPanel.style.display = payload.control_mode === "mit" ? "block" : "none";
    }
    if (payload.control_mode === "mit") {
      state.mitKp = Array.isArray(payload.mit_kp) ? payload.mit_kp : state.mitKp;
      state.mitKd = Array.isArray(payload.mit_kd) ? payload.mit_kd : state.mitKd;
      if (els.mitParamsGrid?.children.length) {
        syncMitSliderFromState();
      }
    }
  }
  if (Array.isArray(payload.joints_rad)) {
    setJointRadians(payload.joints_rad);
  }
  if (payload.end_effector?.position_m) {
    state.lastPose = payload.end_effector;
    writeCartesianInputs(payload.end_effector);
    writePoseSummary(payload.end_effector);
  }
}

function showArmDetectModal(mode = state.armModalMode) {
  if (!els.armDetectModal || state.connected || state.armPromptDismissed) return;

  state.armModalMode = mode || "connect";
  const isMissing = state.armModalMode === "missing";
  const count = state.detectedMotors.length;

  els.armDetectTitle.textContent = isMissing ? t("modal.noArmTitle") : t("modal.detectedTitle");
  els.armDetectCancel.textContent = isMissing ? t("button.gotIt") : t("button.notNow");
  els.armDetectConfirm.hidden = isMissing;
  els.serialPermissionButton.hidden = true;
  els.armDetectConfirm.disabled = false;
  if (els.sudoPasswordGroup) els.sudoPasswordGroup.hidden = !state.sudoPasswordRequired;

  if (isMissing) {
    els.armDetectText.textContent = t("modal.noArmPrompt");
    els.detectedMotors.innerHTML = `
      <span class="detected-motor">
        <iconify-icon icon="mingcute:warning-line" aria-hidden="true"></iconify-icon>
        <span>${t("modal.noArmTitle")}</span>
      </span>
    `;
    els.armDetectModal.hidden = false;
    return;
  }

  if (!state.interfacePresent) return;

  els.armDetectConfirm.hidden = false;
  if (state.permissionRequired) {
    els.serialPermissionButton.hidden = false;
    els.armDetectConfirm.disabled = true;
    els.armDetectText.textContent = state.permissionHint || t("modal.permissionHintDefault");
    els.detectedMotors.innerHTML = `
      <span class="detected-motor">
        <iconify-icon icon="mingcute:terminal-box-line" aria-hidden="true"></iconify-icon>
        <span>sudo chmod 666 /dev/ttyACM*</span>
      </span>
    `;
  } else if (state.sudoPasswordRequired) {
    els.armDetectText.textContent = t("modal.sudoNeeded");
    els.detectedMotors.innerHTML = "";
  } else {
    els.armDetectText.textContent = count > 0
      ? t("modal.detectedAsk", { count })
      : state.armDetected
        ? t("modal.connectGuide")
        : t("modal.interfaceGuide");
    els.detectedMotors.innerHTML = state.detectedMotors
      .map((motor) => `
        <span class="detected-motor">
          <iconify-icon icon="mingcute:cpu-line" aria-hidden="true"></iconify-icon>
          <span>${motor.name ?? t("modal.motorFallback")} · 0x${Number(motor.motor_id ?? 0).toString(16).padStart(2, "0")}</span>
        </span>
      `)
      .join("");
  }
  els.armDetectModal.hidden = false;
}

function hideArmDetectModal() {
  if (els.armDetectModal) els.armDetectModal.hidden = true;
  if (els.serialPermissionButton) els.serialPermissionButton.hidden = true;
  if (els.sudoPasswordInput) els.sudoPasswordInput.value = "";
  if (els.armDetectConfirm) {
    els.armDetectConfirm.hidden = false;
    els.armDetectConfirm.disabled = false;
  }
  if (els.armDetectCancel) els.armDetectCancel.textContent = t("button.notNow");
  state.armModalMode = "connect";
}

async function configureSerialPermission() {
  els.serialPermissionButton.disabled = true;
  els.serialPermissionButton.textContent = t("button.configuring");
  try {
    const result = await apiJson("/permissions/serial", { method: "POST", body: {}, timeoutMs: 25000 });
    applyArmDetection(result);
    updateConnectionUi();
    if (result.ok) {
      setSolverStatus({ key: "solver.permissionConfigured" });
      addLog("串口权限配置完成");
      await refreshBackendHealth();
      showArmDetectModal();
    } else {
      const message = result.message || result.permission_hint || "串口权限配置失败";
      state.permissionHint = message;
      setSolverStatus(message, true);
      addLog(message);
      showArmDetectModal();
    }
  } catch (error) {
    const message = state.locale === "en-US"
      ? "Serial access setup failed. Run sudo chmod 666 /dev/ttyACM* manually."
      : "串口权限配置失败，请手动执行 sudo chmod 666 /dev/ttyACM*";
    setSolverStatus(message, true);
    addLog(message);
  } finally {
    els.serialPermissionButton.disabled = false;
    els.serialPermissionButton.textContent = t("button.configureSerial");
  }
}

function setActiveView(viewName) {
  state.activeView = viewMeta[viewName] ? viewName : "control";
  updateViewHeader();
  els.systemActions.hidden = false;

  if (state.activeView === "control") {
    resizeRenderer();
    startAnimation();
  } else {
    stopAnimation();
    if (state.activeView === "diagnostics") {
      renderDiagnostics();
      refreshBackendLogs();
      refreshBackendHealth({ showPrompt: false });
      if (state.connected) refreshRobotState();
    }
  }
}

function estimateTcp() {
  const radians = joints.map((joint) => degToRad(joint.angle));
  const reach = 0.19 + 0.09 * Math.cos(radians[1]) + 0.075 * Math.cos(radians[1] + radians[2]);
  const x = reach * Math.cos(radians[0]);
  const y = reach * Math.sin(radians[0]);
  const z = 0.15 + 0.11 * Math.sin(radians[1]) + 0.085 * Math.sin(radians[1] + radians[2]);
  return { x, y, z };
}

function getUrdfEndEffectorPose() {
  if (!state.robot) return null;
  const endEffectorLink = state.simConfig.endEffectorLink ?? "end_link";
  const endLink = state.robot.links?.[endEffectorLink];
  if (!endLink) return null;

  state.robot.updateMatrixWorld(true);
  endLink.updateMatrixWorld(true);
  endLink.matrixWorld.decompose(tempPosition, tempQuaternion, tempScale);
  getTcpWorldPosition(endLink, tempTcpWorld);
  tempPosition.copy(tempTcpWorld);
  state.robot.worldToLocal(tempPosition);
  tempEuler.setFromQuaternion(tempQuaternion, "XYZ");

  return {
    position_m: [tempPosition.x, tempPosition.y, tempPosition.z],
    rpy_rad: [tempEuler.x, tempEuler.y, tempEuler.z],
  };
}

function estimatePoseFromJoints() {
  const urdfPose = getUrdfEndEffectorPose();
  if (urdfPose) return urdfPose;

  const tcp = estimateTcp();
  return {
    position_m: [tcp.x, tcp.y, tcp.z],
    rpy_rad: [degToRad(joints[3].angle), degToRad(joints[4].angle), degToRad(joints[0].angle + joints[5].angle)],
  };
}

function nearestPlanarTarget(position) {
  const [x, y, z] = position;
  const baseOffset = 0.19;
  const zOffset = 0.15;
  const l1 = 0.11;
  const l2 = 0.085;
  const xy = Math.hypot(x, y);
  const radial = xy - baseOffset;
  const dz = z - zOffset;
  const maxReach = l1 + l2 - 0.001;
  const minReach = Math.abs(l1 - l2) + 0.001;
  const distance = Math.hypot(radial, dz);
  const clampedDistance = clamp(distance || minReach, minReach, maxReach);
  const scale = clampedDistance / (distance || clampedDistance);
  const nearestRadial = radial * scale;
  const nearestZ = zOffset + dz * scale;
  const nearestXy = Math.max(0.02, baseOffset + nearestRadial);
  const yaw = Math.atan2(y, x || 1e-9);
  return {
    position_m: [nearestXy * Math.cos(yaw), nearestXy * Math.sin(yaw), nearestZ],
    distanceError: Math.abs(distance - clampedDistance),
  };
}

function estimateIkFromPose(pose) {
  const nearest = nearestPlanarTarget(pose.position_m);
  const [x, y, z] = nearest.position_m;
  const baseOffset = 0.19;
  const zOffset = 0.15;
  const l1 = 0.11;
  const l2 = 0.085;
  const yaw = Math.atan2(y, x || 1e-9);
  const radial = Math.hypot(x, y) - baseOffset;
  const dz = z - zOffset;
  const d = clamp(Math.hypot(radial, dz), Math.abs(l1 - l2) + 0.001, l1 + l2 - 0.001);
  const elbowCos = clamp((d * d - l1 * l1 - l2 * l2) / (2 * l1 * l2), -1, 1);
  const elbow = -Math.acos(elbowCos);
  const shoulder = Math.atan2(dz, radial) - Math.atan2(l2 * Math.sin(elbow), l1 + l2 * Math.cos(elbow));
  const [roll, pitch, targetYaw] = pose.rpy_rad;

  const jointsRad = [
    yaw,
    clamp(shoulder, degToRad(joints[1].min), degToRad(joints[1].max)),
    clamp(elbow, degToRad(joints[2].min), degToRad(joints[2].max)),
    clamp(roll, degToRad(joints[3].min), degToRad(joints[3].max)),
    clamp(pitch, degToRad(joints[4].min), degToRad(joints[4].max)),
    clamp(targetYaw - yaw, degToRad(joints[5].min), degToRad(joints[5].max)),
  ];

  return {
    ok: nearest.distanceError < 0.002,
    joints_rad: jointsRad,
    end_effector: {
      position_m: nearest.position_m,
      rpy_rad: pose.rpy_rad,
    },
    error: nearest.distanceError,
  };
}

async function solveFkRealtime(source = "manual") {
  const seq = ++state.fkSeq;
  let pose = null;

  if (state.backendOnline) {
    try {
      const result = await apiJson("/sim/fk", {
        method: "POST",
        body: { joints_rad: getJointRadians() },
        timeoutMs: 1200,
      });
      if (seq !== state.fkSeq) return null;
      pose = result.end_effector;
      setSolverStatus({ key: "solver.fkUpdated", vars: { source: translateSource(source) } });
    } catch {
      pose = estimatePoseFromJoints();
      setSolverStatus({ key: "solver.fkFallback" }, true);
    }
  } else {
    pose = estimatePoseFromJoints();
    setSolverStatus({ key: "solver.fkMock" });
  }

  if (pose) {
    state.lastPose = pose;
    writeCartesianInputs(pose);
    writePoseSummary(pose);
  }
  return pose;
}

async function solveIkRealtime() {
  if (state.suppressCartesianSolve) return;
  const seq = ++state.ikSeq;
  const target = readCartesianInputs();

  let result = null;
  let usedApproximation = false;

  if (state.backendOnline) {
    try {
      result = await apiJson("/sim/ik", {
        method: "POST",
        body: {
          x: target.position_m[0],
          y: target.position_m[1],
          z: target.position_m[2],
          roll: target.rpy_rad[0],
          pitch: target.rpy_rad[1],
          yaw: target.rpy_rad[2],
          seed_joints_rad: getJointRadians(),
        },
        timeoutMs: 2200,
      });
      setSolverStatus({ key: "solver.ikSuccess", vars: { error: (result.error ?? 0).toExponential(2) } });
    } catch {
      result = estimateIkFromPose(target);
      usedApproximation = true;
    }
  } else {
    result = estimateIkFromPose(target);
    usedApproximation = !result.ok;
  }

  if (seq !== state.ikSeq || !result?.joints_rad) return;

  setJointRadians(result.joints_rad);

  const pose = result.end_effector ?? estimatePoseFromJoints();
  state.lastPose = pose;
  writeCartesianInputs(pose, { markApprox: usedApproximation || result.ok === false });
  writePoseSummary(pose);

  if (usedApproximation || result.ok === false) {
    const errorText = Number(result.error ?? 0).toFixed(4);
    setSolverStatus({ key: "solver.ikFallback", vars: { error: errorText } }, true);
  }
}

function updateDerivedState() {
  const pose = state.lastPose ?? estimatePoseFromJoints();
  const tcp = {
    x: Number(pose.position_m[0]),
    y: Number(pose.position_m[1]),
    z: Number(pose.position_m[2]),
  };
  const load = clamp(16 + Math.abs(joints[1].angle) * 0.32 + Math.abs(joints[2].angle) * 0.18, 8, 92);
  const torque = clamp(0.8 + load / 26, 0.8, 5.5);
  const temp = clamp(35 + load * 0.31, 35, 76);
  const error = clamp(0.25 + Math.abs(joints[5].angle) / 90, 0.25, 3.2);

  if (els.tcpText) els.tcpText.textContent = `${tcp.x.toFixed(2)}, ${tcp.y.toFixed(2)}, ${tcp.z.toFixed(2)} m`;
  if (els.rpyText) els.rpyText.textContent = `${pose.rpy_rad.map((value) => Number(radToDeg(value)).toFixed(1)).join(", ")}°`;
  if (els.loadText) els.loadText.textContent = `${load.toFixed(0)}%`;
  if (els.voltageText) els.voltageText.textContent = `${(24.0 + Math.sin(Date.now() / 1200) * 0.25).toFixed(1)} V`;
  if (els.tempText) els.tempText.textContent = `${temp.toFixed(1)} °C`;
  if (els.torqueText) els.torqueText.textContent = `${torque.toFixed(1)} Nm`;
  if (els.errorText) els.errorText.textContent = `${error.toFixed(1)}°`;
  renderDiagnostics();
}

async function refreshBackendHealth({ showPrompt = false } = {}) {
  try {
    const health = await apiJson("/healthz", { timeoutMs: 1800 });
    applyRealtimePayload(health);
    updateConnectionUi();
    if (showPrompt || (!state.connected && state.interfacePresent && !state.armPromptDismissed)) {
      showArmDetectModal("connect");
    }
    return health;
  } catch {
    applyBackendStatus(false);
    state.armDetected = false;
    state.interfacePresent = false;
    state.detectedMotors = [];
    state.permissionRequired = false;
    state.permissionHint = "";
    state.lastHealth = null;
    state.lastRobotState = null;
    hideArmDetectModal();
    updateConnectionUi();
    return null;
  }
}

async function refreshRobotState() {
  try {
    const robotState = await apiJson("/state", { timeoutMs: 2500 });
    applyRealtimePayload(robotState, { robotState: true });
    updateConnectionUi();
    return robotState;
  } catch (error) {
    await refreshBackendHealth({ showPrompt: false });
    return null;
  }
}

async function readRealArmPose() {
  if (!state.connected) return;
  els.readRealPoseButton.disabled = true;
  els.readRealPoseButton.dataset.loading = "true";
  setButtonContent(els.readRealPoseButton, "mingcute:loading-3-line", t("button.reading"));
  try {
    const robotState = await refreshRobotState();
    if (robotState?.joints_rad) {
      setSolverStatus({ key: "solver.readPoseSynced" });
      addLog(`已获取真机位置: ${robotState.joints_rad.map((value) => radToDeg(value).toFixed(1)).join(", ")}°`);
      await refreshBackendLogs();
    } else {
      setSolverStatus({ key: "solver.readPoseMissing" }, true);
      addLog("未读取到真机位置");
    }
  } finally {
    delete els.readRealPoseButton.dataset.loading;
    updateConnectionUi();
  }
}

async function runDiagnosticsAction(action) {
  if (action === "calibrate" && !window.confirm(t("diagnostics.calibrateConfirm"))) {
    return;
  }
  const labelMap = {
    refreshScan: t("button.refreshScan"),
    feedback: t("button.fetchFeedback"),
    enable: t("button.enableDrive"),
    disable: t("button.disableDrive"),
    calibrate: t("button.calibrateZero"),
  };
  const actionLabel = labelMap[action] ?? action;
  state.diagnosticsBusyAction = action;
  state.diagnosticsBusyLabel = actionLabel;
  state.diagnosticsStatusMessage = { key: "diagnostics.actionRunning", vars: { action: actionLabel } };
  renderDiagnostics();

  try {
    if (action === "refreshScan") {
      const health = await refreshBackendHealth({ showPrompt: false });
      if (!health) {
        throw new Error(state.locale === "en-US" ? "Backend unavailable" : "后端服务不可用");
      }
      const result = await apiJson("/debug/arm", {
        method: "POST",
        body: { action: "scan" },
        timeoutMs: 10000,
      });
      applyRealtimePayload(result, { robotState: Array.isArray(result.joints_rad) });
      if (result.connected) {
        await refreshRobotState();
      }
      await refreshBackendLogs();
      const message = result.message || t("diagnostics.actionDone", { action: actionLabel });
      state.diagnosticsStatusMessage = message;
      addLog(state.locale === "en-US" ? `${actionLabel} complete` : `${actionLabel}完成`);
      return;
    }

    const result = await apiJson("/debug/arm", {
      method: "POST",
      body: { action },
      timeoutMs: 10000,
    });
    applyRealtimePayload(result, { robotState: Array.isArray(result.joints_rad) });
    updateConnectionUi();
    await refreshBackendLogs();

    const message = result.message || t("diagnostics.actionDone", { action: actionLabel });
    state.diagnosticsStatusMessage = message;
    addLog(message);
  } catch (error) {
    let message = state.locale === "en-US" ? `Debug action failed: ${actionLabel}` : `调试操作失败: ${actionLabel}`;
    if (error instanceof Error && error.message) {
      try {
        const payload = JSON.parse(error.message);
        message = payload.message || payload.detail || message;
      } catch {
        message = error.message || message;
      }
    }
    state.diagnosticsStatusMessage = message;
    addLog(message);
  } finally {
    state.diagnosticsBusyAction = "";
    state.diagnosticsBusyLabel = "";
    renderDiagnostics();
  }
}


async function handleConnectButton() {
  if (state.connected) {
    await disconnectRobot();
    return;
  }

  state.armPromptDismissed = false;
  hideArmDetectModal();

  const health = await refreshBackendHealth({ showPrompt: false });
  if (!health) {
    setSolverStatus({ key: "solver.backendUnavailable" }, true);
    addLog(t("solver.backendUnavailable"));
    return;
  }

  if (health.permission_required || health.interface_present || health.arm_detected) {
    await connectRobot({ confirmed: false });
    return;
  }

  state.armModalMode = "missing";
  showArmDetectModal("missing");
  setSolverStatus({ key: "modal.noArmPrompt" }, true);
  addLog(t("modal.noArmPrompt"));
}

async function connectRobot({ confirmed = false, sudoPassword = null } = {}) {
  const body = { confirm: confirmed };
  if (sudoPassword) body.sudo_password = sudoPassword;
  try {
    const result = await apiJson("/connect", { method: "POST", body, timeoutMs: 8000 });
    applyRealtimePayload(result);
    if (result.requires_confirmation) {
      state.armPromptDismissed = false;
      state.armModalMode = "connect";
      showArmDetectModal("connect");
      addLog("检测到真实机械臂，等待用户确认连接");
      updateConnectionUi();
      return;
    }
    if (result.ok === false) {
      const message = result.message ?? result.permission_hint ?? "未检测到真实机械臂";
      state.armPromptDismissed = false;
      if (result.permission_required || result.interface_present || result.arm_detected) {
        state.armModalMode = "connect";
        showArmDetectModal("connect");
      } else {
        state.armModalMode = "missing";
        showArmDetectModal("missing");
      }
      setSolverStatus(message, true);
      addLog(message);
      updateConnectionUi();
      return;
    }
    hideArmDetectModal();
    state.armPromptDismissed = false;
    addLog("真实机械臂连接成功");
    setSolverStatus({ key: "solver.connectSuccess" });
    updateConnectionUi();
    await refreshBackendLogs();
  } catch (error) {
    addLog("连接真实机械臂失败");
    await refreshBackendHealth({ showPrompt: false });
    updateConnectionUi();
  }
}

async function disconnectRobot() {
  try {
    const result = await apiJson("/disconnect", { method: "POST", body: {} });
    applyRealtimePayload(result);
    state.armPromptDismissed = false;
    hideArmDetectModal();
    addLog("真实机械臂已断开");
  } catch {
    state.connected = false;
    state.enabled = false;
    state.lastHealth = null;
    addLog("断开机械臂失败");
  }
  updateConnectionUi();
  updateDerivedState();
}

async function sendJointTarget() {
  try {
    const result = await apiJson("/move/joints", {
      method: "POST",
      body: { joints_rad: getJointRadians(), max_speed_rad_s: 0.22 },
      timeoutMs: 30000,
    });
    if (result.ok === false) {
      const message = result.message || "目标位置发送失败";
      setSolverStatus(message, true);
      addLog(message);
      await refreshBackendLogs();
      return;
    }
    addLog(result.message || "目标位置已低速下发");
    if (result.command) {
      addLog(`运动命令: ${result.command.type}, ${result.command.duration_s.toFixed(2)}s, ${result.command.steps} 步`);
    }
    await refreshBackendLogs();
    await refreshBackendHealth();
  } catch (error) {
    const message = state.connected ? "目标位置发送失败" : "Mock: 目标位置已发送";
    setSolverStatus(message, state.connected);
    addLog(message);
  }
}

function readCartesianTarget() {
  const pose = readCartesianInputs();
  return {
    x: pose.position_m[0],
    y: pose.position_m[1],
    z: pose.position_m[2],
    roll: pose.rpy_rad[0],
    pitch: pose.rpy_rad[1],
    yaw: pose.rpy_rad[2],
    mode: "traj",
    duration: 1.5,
  };
}

async function sendPoseTarget() {
  try {
    await apiJson("/move/pose", {
      method: "POST",
      body: readCartesianTarget(),
      timeoutMs: 10000,
    });
    addLog("末端位姿目标已下发");
    await refreshBackendHealth();
  } catch {
    addLog("Mock: 末端位姿目标已发送");
  }
}

async function homeRobot() {
  joints.forEach((joint) => {
    joint.angle = 0;
  });
  syncJointInputs();
  updateRobotJoints();
  updateDerivedState();
  await solveFkRealtime("home");
  addLog("已切换到初始位置");
}

async function sendGripper(action) {
  if (!state.connected || state.gripperBusy) return;
  state.gripperBusy = true;
  updateConnectionUi();
  try {
    const result = await apiJson("/gripper", {
      method: "POST",
      body: { action },
      timeoutMs: 8000,
    });
    const message = result.message || (action === "open" ? "夹爪已打开" : "夹爪已闭合");
    setSolverStatus(message);
    addLog(message);
    await refreshBackendLogs();
  } catch (error) {
    const fallbackMessage = action === "open" ? "夹爪打开失败" : "夹爪闭合失败";
    let message = fallbackMessage;
    if (error instanceof Error && error.message) {
      try {
        const payload = JSON.parse(error.message);
        message = payload.message || payload.detail || fallbackMessage;
      } catch {
        message = error.message || fallbackMessage;
      }
    }
    setSolverStatus(message, true);
    addLog(message);
  } finally {
    state.gripperBusy = false;
    updateConnectionUi();
  }
}

function bindControls() {
  document.querySelectorAll(".nav-item").forEach((button) => {
    button.addEventListener("click", () => {
      document.querySelectorAll(".nav-item").forEach((item) => item.classList.remove("active"));
      document.querySelectorAll(".view").forEach((view) => view.classList.remove("active"));
      button.classList.add("active");
      document.querySelector(`#${button.dataset.view}View`).classList.add("active");
      setActiveView(button.dataset.view);
      resizeRenderer();
    });
  });

  document.querySelectorAll("[data-camera]").forEach((button) => {
    button.addEventListener("click", () => {
      document.querySelectorAll("[data-camera]").forEach((item) => item.classList.remove("active"));
      button.classList.add("active");

      const target = controls.target;
      if (button.dataset.camera === "top") {
        camera.position.set(target.x, target.y, target.z + 1.1);
      } else {
        camera.position.set(target.x + 0.48, target.y - 0.64, target.z + 0.38);
      }
      camera.lookAt(target);
      controls.update();
    });
  });

  els.solveIkButton.addEventListener("click", solveIkRealtime);
  els.armDetectCancel.addEventListener("click", () => {
    state.armPromptDismissed = true;
    hideArmDetectModal();
    addLog("已暂不连接真实机械臂");
  });
  els.armDetectConfirm.addEventListener("click", async () => {
    const sudoPassword = state.sudoPasswordRequired
      ? (els.sudoPasswordInput?.value || "").trim()
      : null;
    if (state.sudoPasswordRequired && !sudoPassword) {
      setSolverStatus(t("modal.sudoNeeded"), true);
      els.sudoPasswordInput?.focus();
      return;
    }
    if (state.sudoPasswordRequired && sudoPassword) {
      // 先调用 /can/prepare 验证密码并配置所有 CAN 接口
      // 配置成功后再继续走 /connect，避免密码错误时静默失败
      els.armDetectConfirm.disabled = true;
      try {
        const prepareResult = await apiJson("/can/prepare", {
          method: "POST",
          body: { sudo_password: sudoPassword },
          timeoutMs: 8000,
        });
        if (prepareResult.wrong_sudo_password) {
          setSolverStatus(t("modal.sudoWrongPassword") || "sudo 密码错误", true);
          addLog("sudo 密码错误，请重试");
          els.sudoPasswordInput.value = "";
          els.sudoPasswordInput.focus();
          els.armDetectConfirm.disabled = false;
          return;
        }
        if (!prepareResult.ok) {
          setSolverStatus(prepareResult.message || "CAN 接口配置失败", true);
          addLog(prepareResult.message || "CAN 接口配置失败");
          els.armDetectConfirm.disabled = false;
          return;
        }
        addLog(`CAN 接口已配置: bitrate=${prepareResult.bitrate}`);
      } catch (error) {
        setSolverStatus(t("modal.sudoNeeded"), true);
        addLog("CAN 接口准备失败: " + (error?.message || error));
        els.armDetectConfirm.disabled = false;
        return;
      }
    }
    hideArmDetectModal();
    connectRobot({ confirmed: true, sudoPassword });
  });
  els.serialPermissionButton.addEventListener("click", configureSerialPermission);

  els.languageSelect.addEventListener("change", () => {
    setLanguage(els.languageSelect.value);
  });
  els.armTypeSelect?.addEventListener("change", () => {
    changeArmType(els.armTypeSelect.value);
  });
  els.enableButton.addEventListener("click", handleConnectButton);

  document.querySelector("#homeButton").addEventListener("click", homeRobot);
  els.readRealPoseButton.addEventListener("click", readRealArmPose);

  document.querySelector("#sendTargetButton").addEventListener("click", () => {
    sendJointTarget();
  });
  els.gripperOpenButton.addEventListener("click", () => sendGripper("open"));
  els.gripperCloseButton.addEventListener("click", () => sendGripper("close"));
  els.refreshScanButton?.addEventListener("click", () => runDiagnosticsAction("refreshScan"));
  els.fetchFeedbackButton?.addEventListener("click", () => runDiagnosticsAction("feedback"));
  els.enableDriveButton?.addEventListener("click", () => runDiagnosticsAction("enable"));
  els.disableDriveButton?.addEventListener("click", () => runDiagnosticsAction("disable"));
  els.calibrateZeroButton?.addEventListener("click", () => runDiagnosticsAction("calibrate"));
  els.diagnosticsMotorTabs?.addEventListener("click", (event) => {
    const button = event.target.closest("[data-motor-tab]");
    if (!button) return;
    state.selectedMotorName = button.getAttribute("data-motor-tab") || "";
    renderDiagnostics();
  });

  window.addEventListener("resize", resizeRenderer);
}

function tickTelemetry() {
  if (state.activeView !== "control") {
    const now = Date.now();
    if (now - state.lastBackgroundHealthAt > 6000) {
      state.lastBackgroundHealthAt = now;
      refreshBackendHealth();
    }
    if (state.activeView === "diagnostics" && now - state.lastBackendLogAt > 2000) {
      state.lastBackendLogAt = now;
      refreshBackendLogs();
    }
    return;
  }

  if (state.backendOnline && state.connected) {
    refreshBackendHealth();
    return;
  }
  if (state.backendOnline && !state.connected) {
    refreshBackendHealth();
    return;
  }

  updateDerivedState();
}

function animate() {
  if (state.activeView !== "control") {
    state.animationFrameId = null;
    return;
  }
  if (!renderer) {
    state.animationFrameId = null;
    return;
  }
  controls.update();
  renderer.render(scene, camera);
  state.animationFrameId = requestAnimationFrame(animate);
}

function startAnimation() {
  if (state.animationFrameId !== null) return;
  state.animationFrameId = requestAnimationFrame(animate);
}

function stopAnimation() {
  if (state.animationFrameId === null) return;
  cancelAnimationFrame(state.animationFrameId);
  state.animationFrameId = null;
}

// ── 控制模式选择器 & MIT 参数 UI ───────────────────────────────────

function initControlModeUI() {
  if (!els.controlModeSelector) return;

  // 绑定 radio change 事件
  els.controlModeSelector.querySelectorAll('input[name="controlMode"]').forEach((radio) => {
    radio.addEventListener("change", async () => {
      const mode = radio.value;
      state.controlMode = mode;

      // 显示/隐藏 MIT 参数面板
      if (els.mitParamsPanel) {
        els.mitParamsPanel.style.display = mode === "mit" ? "block" : "none";
      }

      // 切换到 MIT 模式时，渲染 Kp/Kd 滑块
      if (mode === "mit") {
        renderMitParamsPanel();
      }

      // 发到后端（未连接时也发，后端会记录状态）
      if (state.backendOnline) {
        try {
          const body = { mode };
          if (mode === "mit") {
            body.kp = getMitKp();
            body.kd = getMitKd();
          }
          await apiJson("/control/mode", {
            method: "POST",
            body,
            timeoutMs: 5000,
          });
        } catch (e) {
          addLog("控制模式切换失败: " + (e?.message || e));
        }
      }

      // 更新发送按钮文字
      updateConnectionUi();
    });
  });
}

function renderMitParamsPanel() {
  if (!els.mitParamsGrid) return;
  const n = joints.length;
  const kp = state.mitKp ?? [];
  const kd = state.mitKd ?? [];

  els.mitParamsGrid.innerHTML = "";

  for (let i = 0; i < n; i++) {
    const joint = joints[i];
    const kpVal = kp[i] ?? joint.mitKp ?? 50;
    const kdVal = kd[i] ?? joint.mitKd ?? 5;

    const col = document.createElement("div");
    col.className = "mit-param-group";

    const title = document.createElement("div");
    title.className = "mit-param-label";
    title.textContent = joint.name;
    col.appendChild(title);

    const kpRow = document.createElement("div");
    kpRow.className = "mit-param-row";
    kpRow.innerHTML = `
      <label>${t("panel.mitKp")}
        <span id="mitKpVal${i}">${kpVal.toFixed(0)}</span>
      </label>
      <input type="range" id="mitKp${i}" min="0" max="200" step="1" value="${kpVal}" />
    `;
    const kpInput = kpRow.querySelector(`#mitKp${i}`);
    const kpValSpan = kpRow.querySelector(`#mitKpVal${i}`);
    kpInput.addEventListener("input", () => {
      const v = Number(kpInput.value);
      kpValSpan.textContent = v;
      state.mitKp = state.mitKp || getMitKp();
      state.mitKp[i] = v;
      syncMitParamsToBackend();
    });
    col.appendChild(kpRow);

    const kdRow = document.createElement("div");
    kdRow.className = "mit-param-row";
    kdRow.innerHTML = `
      <label>${t("panel.mitKd")}
        <span id="mitKdVal${i}">${kdVal.toFixed(1)}</span>
      </label>
      <input type="range" id="mitKd${i}" min="0" max="20" step="0.5" value="${kdVal}" />
    `;
    const kdInput = kdRow.querySelector(`#mitKd${i}`);
    const kdValSpan = kdRow.querySelector(`#mitKdVal${i}`);
    kdInput.addEventListener("input", () => {
      const v = Number(kdInput.value);
      kdValSpan.textContent = v.toFixed(1);
      state.mitKd = state.mitKd || getMitKd();
      state.mitKd[i] = v;
      syncMitParamsToBackend();
    });
    col.appendChild(kdRow);

    els.mitParamsGrid.appendChild(col);
  }
}

function syncMitSliderFromState() {
  const kp = state.mitKp ?? [];
  const kd = state.mitKd ?? [];
  for (let i = 0; i < joints.length; i++) {
    const kpInput = document.getElementById(`mitKp${i}`);
    const kpVal = document.getElementById(`mitKpVal${i}`);
    if (kpInput) {
      kpInput.value = kp[i] ?? 50;
      if (kpVal) kpVal.textContent = kpInput.value;
    }
    const kdInput = document.getElementById(`mitKd${i}`);
    const kdVal = document.getElementById(`mitKdVal${i}`);
    if (kdInput) {
      kdInput.value = kd[i] ?? 5;
      if (kdVal) kdVal.textContent = Number(kdInput.value).toFixed(1);
    }
  }
}

function getMitKp() {
  const n = joints.length;
  if (!state.mitKp || state.mitKp.length !== n) {
    return joints.map((j) => j.mitKp ?? 50);
  }
  return state.mitKp;
}

function getMitKd() {
  const n = joints.length;
  if (!state.mitKd || state.mitKd.length !== n) {
    return joints.map((j) => j.mitKd ?? 5);
  }
  return state.mitKd;
}

let _mitSyncDebounce = null;
async function syncMitParamsToBackend() {
  if (!state.backendOnline) return;
  clearTimeout(_mitSyncDebounce);
  _mitSyncDebounce = setTimeout(async () => {
    try {
      await apiJson("/control/mode", {
        method: "POST",
        body: { mode: "mit", kp: getMitKp(), kd: getMitKd() },
        timeoutMs: 3000,
      });
    } catch (_) {}
  }, 400);
}

// ── 初始化 ──────────────────────────────────────────────────────

async function init() {
  resizeRenderer();
  renderJoints();
  initControlModeUI();
  bindControls();
  applyTranslations();
  setActiveView("control");
  await refreshBackendHealth({ showPrompt: false });
  await loadSimConfig();
  updateConnectionUi();
  updateDerivedState();
  renderDiagnostics();
  solveFkRealtime("init");
  loadRobotModel();
  startAnimation();
  addLog("WebUI 初始化完成");
  setInterval(tickTelemetry, 1200);
}

init();
