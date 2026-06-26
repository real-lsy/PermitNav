import * as THREE from "./vendor/three.module.js";
import { OrbitControls } from "./vendor/jsm/controls/OrbitControls.js";

const ROSLIB = window.ROSLIB;
if (!ROSLIB) {
  throw new Error("ROSLIB 加载失败，请检查 CDN 网络访问，或改为本地提供 roslib。");
}

const wsInput = document.getElementById("ws-url");
const advancedSettings = document.getElementById("advanced-settings");
const connectBtn = document.getElementById("connect-btn");
const reconnectBtn = document.getElementById("reconnect-btn");
const connStatus = document.getElementById("conn-status");
const relocalizationStatus = document.getElementById("relocalization-status");
const robotTfStatus = document.getElementById("robot-tf-status");
const selectionStatus = document.getElementById("selection-status");
const mapStatus = document.getElementById("map-status");
const canvas = document.getElementById("viewport");
const navigationConfirmModal = document.getElementById("navigation-confirm-modal");
const navigationConfirmMessage = document.getElementById("navigation-confirm-message");
const navigationConfirmStartBtn = document.getElementById("navigation-confirm-start");
const navigationConfirmCancelBtn = document.getElementById("navigation-confirm-cancel");
const setCurrentPoseBtn = document.getElementById("set-current-pose-btn");
const setNavigateBtn = document.getElementById("set-navigate-btn");
const setStartBtn = document.getElementById("set-start-btn");
const setGoalBtn = document.getElementById("set-goal-btn");
const stopNavigationBtn = document.getElementById("stop-navigation-btn");
const resetViewBtn = document.getElementById("reset-view-btn");
const joystickPad = document.getElementById("motion-joystick");
const joystickKnob = document.getElementById("motion-joystick-knob");
const manualRotationSlider = document.getElementById("manual-rotation-slider");
const manualVelocityDisplay = document.getElementById("manual-velocity-display");
const toggleOccupied = document.getElementById("toggle-occupied");
const toggleTraversable = document.getElementById("toggle-traversable");
const togglePreblocked = document.getElementById("toggle-preblocked");
const toggleRisk = document.getElementById("toggle-risk");

function defaultRosbridgeUrl() {
  const protocol = window.location.protocol === "https:" ? "wss" : "ws";
  return `${protocol}://${window.location.hostname || "localhost"}:9090`;
}

wsInput.value = defaultRosbridgeUrl();

const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.setSize(canvas.clientWidth || window.innerWidth, canvas.clientHeight || window.innerHeight);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x091116);

const camera = new THREE.PerspectiveCamera(55, 1, 0.01, 500);
camera.up.set(0, 0, 1);
camera.position.set(8, -10, 7);

const controls = new OrbitControls(camera, canvas);
controls.enableDamping = true;
controls.target.set(0, 0, 0.8);

function makeAxisArrow(direction, color, length, shaftRadius, headLength, headRadius) {
  const group = new THREE.Group();
  const shaftLength = Math.max(0.01, length - headLength);
  const shaftMaterial = new THREE.MeshStandardMaterial({
    color,
    roughness: 0.35,
    metalness: 0.08,
  });
  const headMaterial = new THREE.MeshStandardMaterial({
    color,
    emissive: color,
    emissiveIntensity: 0.18,
    roughness: 0.25,
    metalness: 0.1,
  });

  const shaft = new THREE.Mesh(
    new THREE.CylinderGeometry(shaftRadius, shaftRadius, shaftLength, 16),
    shaftMaterial,
  );
  shaft.position.y = shaftLength * 0.5;

  const head = new THREE.Mesh(
    new THREE.ConeGeometry(headRadius, headLength, 20),
    headMaterial,
  );
  head.position.y = shaftLength + headLength * 0.5;

  group.add(shaft);
  group.add(head);

  const up = new THREE.Vector3(0, 1, 0);
  group.quaternion.setFromUnitVectors(up, direction.clone().normalize());
  return group;
}

function makeAxes(length = 2.0) {
  const group = new THREE.Group();
  const shaftRadius = 0.035;
  const headLength = 0.22;
  const headRadius = 0.09;

  group.add(makeAxisArrow(new THREE.Vector3(1, 0, 0), 0xff5f5f, length, shaftRadius, headLength, headRadius));
  group.add(makeAxisArrow(new THREE.Vector3(0, 1, 0), 0x58ef74, length, shaftRadius, headLength, headRadius));
  group.add(makeAxisArrow(new THREE.Vector3(0, 0, 1), 0x53b7d8, length, shaftRadius, headLength, headRadius));
  return group;
}

function makeBox(sizeX, sizeY, sizeZ, color) {
  const geometry = new THREE.BoxGeometry(sizeX, sizeY, sizeZ);
  const material = new THREE.MeshStandardMaterial({
    color,
    roughness: 0.42,
    metalness: 0.05,
  });
  return new THREE.Mesh(geometry, material);
}

function buildSimpleDogModel() {
  const group = new THREE.Group();

  const body = makeBox(0.50, 0.22, 0.16, 0xf2f2f2);
  body.position.set(0, 0, 0.33);
  group.add(body);

  const head = makeBox(0.12, 0.12, 0.12, 0x151515);
  head.position.set(0.31, 0, 0.35);
  group.add(head);

  const hipColor = 0xeb3131;
  const legColor = 0xf2f2f2;
  const hips = [
    [0.18, 0.13, 0.30],
    [0.18, -0.13, 0.30],
    [-0.18, 0.13, 0.30],
    [-0.18, -0.13, 0.30],
  ];

  hips.forEach(([x, y, z]) => {
    const hip = makeBox(0.06, 0.06, 0.06, hipColor);
    hip.position.set(x, y, z);
    group.add(hip);

    const upper = makeBox(0.04, 0.04, 0.22, legColor);
    upper.position.set(x, y, z - 0.14);
    group.add(upper);
  });

  group.visible = false;
  return group;
}

scene.add(new THREE.AmbientLight(0xffffff, 0.55));
const dirLight = new THREE.DirectionalLight(0xffffff, 1.1);
dirLight.position.set(10, -6, 14);
scene.add(dirLight);

const groundGrid = new THREE.GridHelper(30, 30, 0x34515c, 0x24353d);
groundGrid.rotation.x = Math.PI * 0.5;
scene.add(groundGrid);
scene.add(makeAxes(1.8));

let occupiedPointsObject = null;
let occupiedPickObject = null;
let preblockedPointsObject = null;
let traversablePointsObject = null;
let traversablePickObject = null;
let riskPointsObject = null;
let pathObject = null;
let trackingPointObject = null;
let startArrow = null;
let startCube = null;
let goalArrow = null;
let goalCube = null;
let robotObject = buildSimpleDogModel();
let voxelSize = 0.2;
let ros = null;
let startTopic = null;
let goalTopic = null;
let goalPoseTopic = null;
let initialPoseTopic = null;
let startNavigationTopic = null;
let stopNavigationTopic = null;
let cmdVelTopic = null;
let tfTopic = null;
let tfStaticTopic = null;
let reconnectTimer = null;
let activePointerId = null;
let navigationDrag = null;
let placementMode = null;
let pendingNavigationGoal = null;
let pendingNavigationPath = null;
let navigationConfirmTimer = null;
let joystickActivePointerId = null;
let joystickLastPublishMs = 0;
let joystickRepeatTimer = null;
let joystickCurrentLinearX = 0;
let joystickCurrentLinearY = 0;
let joystickCurrentAngularZ = 0;
let mapHasBeenAutoFramed = false;
let latestOccupiedBounds = null;
const tfState = new Map();
const joystickMaxLinearX = 0.42;
const joystickMaxLinearY = 0.42;
const joystickMaxAngularZ = 0.45;
const joystickDeadband = 0.12;
const joystickMinCommandSpeed = 0.06;
const joystickPublishIntervalMs = 80;
const robotCenterOffsetFrame = "odin1_base_link";
const robotCenterOffset = { x: -0.18, y: 0.0, z: 0.0 };
const robotDisplayOffset = { x: 0.0, y: 0.0, z: -0.3 };

scene.add(robotObject);

const raycaster = new THREE.Raycaster();
const pointer = new THREE.Vector2();
function setConnectionStatus(text) {
  connStatus.textContent = text;
}

function setSelectionStatus(text) {
  selectionStatus.textContent = text;
}

function setRobotTfStatus(text) {
  if (robotTfStatus) {
    robotTfStatus.textContent = text;
  }
}

function setRelocalizationStatus(localized) {
  if (!relocalizationStatus) {
    return;
  }
  relocalizationStatus.textContent = localized ? "机器人定位成功" : "机器人尚未定位";
  relocalizationStatus.classList.toggle("ok", localized);
  relocalizationStatus.classList.toggle("fail", !localized);
}

function setMapStatus(text) {
  mapStatus.textContent = text;
}

function updateRendererSize() {
  const width = canvas.clientWidth || window.innerWidth;
  const height = canvas.clientHeight || Math.max(window.innerHeight - 260, 320);
  renderer.setSize(width, height, false);
  camera.aspect = width / height;
  camera.updateProjectionMatrix();
}

function disposeObject(object) {
  if (!object) {
    return;
  }
  if (object.children && object.children.length > 0) {
    object.children.forEach((child) => disposeObject(child));
  }
  if (object.geometry) {
    object.geometry.dispose();
  }
  if (Array.isArray(object.material)) {
    object.material.forEach((material) => material.dispose());
  } else if (object.material) {
    object.material.dispose();
  }
}

function computePointBounds(points) {
  if (!points || points.length === 0) {
    return null;
  }
  const min = new THREE.Vector3(Infinity, Infinity, Infinity);
  const max = new THREE.Vector3(-Infinity, -Infinity, -Infinity);
  for (const point of points) {
    min.x = Math.min(min.x, point.x);
    min.y = Math.min(min.y, point.y);
    min.z = Math.min(min.z, point.z);
    max.x = Math.max(max.x, point.x);
    max.y = Math.max(max.y, point.y);
    max.z = Math.max(max.z, point.z);
  }
  if (!Number.isFinite(min.x) || !Number.isFinite(max.x)) {
    return null;
  }
  return { min, max };
}

function frameCameraToBounds(bounds) {
  if (!bounds) {
    return;
  }
  const center = bounds.min.clone().add(bounds.max).multiplyScalar(0.5);
  const size = bounds.max.clone().sub(bounds.min);
  const maxDim = Math.max(size.x, size.y, size.z, voxelSize * 8.0, 1.0);
  const fovRad = THREE.MathUtils.degToRad(camera.fov);
  const distance = Math.max(maxDim * 1.35, (maxDim * 0.5) / Math.tan(fovRad * 0.5));
  const direction = new THREE.Vector3(1.0, -1.25, 0.8).normalize();
  const position = center.clone().add(direction.multiplyScalar(distance));

  camera.position.copy(position);
  controls.target.copy(center);
  controls.minDistance = Math.max(voxelSize * 2.0, distance * 0.02);
  controls.maxDistance = Math.max(distance * 8.0, maxDim * 8.0);
  camera.near = Math.max(0.01, distance / 10000.0);
  camera.far = Math.max(1000.0, distance * 10.0, maxDim * 20.0);
  camera.updateProjectionMatrix();
  controls.update();
}

function resetMapView() {
  if (!latestOccupiedBounds) {
    setMapStatus("尚未收到可用于重置视角的占据栅格。");
    return;
  }
  frameCameraToBounds(latestOccupiedBounds);
}

function makeVoxelGroup(marker, color, opacity = 1.0, visualScale = 1.0) {
  const sizeX = Math.max(0.02, Number(marker.scale.x || voxelSize)) * visualScale;
  const sizeY = Math.max(0.02, Number(marker.scale.y || voxelSize)) * visualScale;
  const sizeZ = Math.max(0.02, Number(marker.scale.z || voxelSize)) * visualScale;
  const geometry = new THREE.BoxGeometry(sizeX, sizeY, sizeZ);
  const fillMaterial = new THREE.MeshStandardMaterial({
    color,
    transparent: opacity < 0.999,
    opacity,
    roughness: 0.42,
    metalness: 0.04,
  });
  const edgeMaterial = new THREE.MeshBasicMaterial({
    color: 0x7a7a7a,
    wireframe: true,
    transparent: true,
    opacity: Math.min(1.0, opacity + 0.15),
  });
  const fillMesh = new THREE.InstancedMesh(geometry, fillMaterial, marker.points.length);
  const edgeMesh = new THREE.InstancedMesh(geometry, edgeMaterial, marker.points.length);
  fillMesh.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
  edgeMesh.instanceMatrix.setUsage(THREE.DynamicDrawUsage);

  const matrix = new THREE.Matrix4();
  for (let i = 0; i < marker.points.length; i += 1) {
    const point = marker.points[i];
    matrix.makeTranslation(point.x, point.y, point.z);
    fillMesh.setMatrixAt(i, matrix);
    edgeMesh.setMatrixAt(i, matrix);
  }
  fillMesh.instanceMatrix.needsUpdate = true;
  edgeMesh.instanceMatrix.needsUpdate = true;

  const group = new THREE.Group();
  group.add(fillMesh);
  group.add(edgeMesh);
  return { group, pickMesh: fillMesh };
}

function setOccupiedPoints(marker) {
  if (occupiedPointsObject) {
    scene.remove(occupiedPointsObject);
    disposeObject(occupiedPointsObject);
  }
  occupiedPickObject = null;

  voxelSize = Math.max(0.02, Number(marker.scale.x || 0.2));

  let sumX = 0;
  let sumY = 0;
  let sumZ = 0;

  for (const point of marker.points) {
    sumX += point.x;
    sumY += point.y;
    sumZ += point.z;
  }

  const occupiedGroup = makeVoxelGroup(marker, 0xf6c85d, 1.0, 1.0);
  occupiedPointsObject = occupiedGroup.group;
  occupiedPickObject = occupiedGroup.pickMesh;
  scene.add(occupiedPointsObject);
  occupiedPointsObject.visible = toggleOccupied.checked;

  if (marker.points.length > 0) {
    const cx = sumX / marker.points.length;
    const cy = sumY / marker.points.length;
    const cz = sumZ / marker.points.length;
    controls.target.set(cx, cy, cz);
    latestOccupiedBounds = computePointBounds(marker.points);
    if (!mapHasBeenAutoFramed) {
      frameCameraToBounds(latestOccupiedBounds);
      mapHasBeenAutoFramed = true;
    }
  }

  setMapStatus(`${marker.points.length} 个占据栅格，分辨率 ${voxelSize.toFixed(2)} 米`);
}

function setPreblockedPoints(marker) {
  if (preblockedPointsObject) {
    scene.remove(preblockedPointsObject);
    disposeObject(preblockedPointsObject);
    preblockedPointsObject = null;
  }

  if (!marker.points || marker.points.length === 0) {
    return;
  }

  preblockedPointsObject = makeVoxelGroup(marker, 0x4d83ff, 0.92, 1.0).group;
  scene.add(preblockedPointsObject);
  preblockedPointsObject.visible = togglePreblocked.checked;
}

function setTraversablePoints(marker) {
  if (traversablePointsObject) {
    scene.remove(traversablePointsObject);
    disposeObject(traversablePointsObject);
    traversablePointsObject = null;
  }
  traversablePickObject = null;

  if (!marker.points || marker.points.length === 0) {
    return;
  }

  const traversableGroup = makeVoxelGroup(marker, 0x58ef74, 0.22, 1.0);
  traversablePointsObject = traversableGroup.group;
  traversablePickObject = traversableGroup.pickMesh;
  scene.add(traversablePointsObject);
  traversablePointsObject.visible = toggleTraversable.checked;
}

function parsePointCloud2(msg) {
  if (!msg || !msg.data || !msg.fields) {
    return [];
  }

  const byteArray = Array.isArray(msg.data)
    ? new Uint8Array(msg.data)
    : Uint8Array.from(atob(msg.data), (ch) => ch.charCodeAt(0));
  const view = new DataView(byteArray.buffer, byteArray.byteOffset, byteArray.byteLength);
  const fieldMap = new Map(msg.fields.map((field) => [field.name, field]));
  const xField = fieldMap.get("x");
  const yField = fieldMap.get("y");
  const zField = fieldMap.get("z");
  const intensityField = fieldMap.get("intensity");
  if (!xField || !yField || !zField) {
    return [];
  }

  const littleEndian = !msg.is_bigendian;
  const points = [];
  const pointStep = msg.point_step;
  const total = pointStep > 0 ? Math.floor(byteArray.byteLength / pointStep) : 0;
  for (let i = 0; i < total; i += 1) {
    const base = i * pointStep;
    const x = view.getFloat32(base + xField.offset, littleEndian);
    const y = view.getFloat32(base + yField.offset, littleEndian);
    const z = view.getFloat32(base + zField.offset, littleEndian);
    const intensity = intensityField
      ? view.getFloat32(base + intensityField.offset, littleEndian)
      : 0.0;
    if (Number.isFinite(x) && Number.isFinite(y) && Number.isFinite(z)) {
      points.push({ x, y, z, intensity });
    }
  }
  return points;
}

function setRiskPoints(msg) {
  if (riskPointsObject) {
    scene.remove(riskPointsObject);
    disposeObject(riskPointsObject);
    riskPointsObject = null;
  }

  const points = parsePointCloud2(msg);
  if (points.length === 0) {
    return;
  }

  const scale = voxelSize;
  const group = new THREE.Group();
  points.forEach((point) => {
    const alpha = Math.min(0.95, Math.max(0.12, 0.12 + point.intensity * 0.83));
    const mesh = new THREE.Mesh(
      new THREE.BoxGeometry(scale, scale, scale),
      new THREE.MeshStandardMaterial({
        color: 0x2659ff,
        transparent: true,
        opacity: alpha,
        roughness: 0.38,
        metalness: 0.03,
      }),
    );
    mesh.position.set(point.x, point.y, point.z);
    group.add(mesh);
  });
  riskPointsObject = group;
  riskPointsObject.visible = toggleRisk.checked;
  scene.add(riskPointsObject);
}

function clearObject(object) {
  if (!object) {
    return;
  }
  scene.remove(object);
  if (object.geometry) {
    object.geometry.dispose();
  }
  if (object.material) {
    object.material.dispose();
  }
}

function makeCube(center, size, color) {
  const geometry = new THREE.BoxGeometry(size, size, size);
  const material = new THREE.MeshStandardMaterial({ color });
  const cube = new THREE.Mesh(geometry, material);
  cube.position.set(center.x, center.y, center.z);
  return cube;
}

function makeArrow(marker, color) {
  if (!marker.points || marker.points.length < 2) {
    return null;
  }
  const start = new THREE.Vector3(marker.points[0].x, marker.points[0].y, marker.points[0].z);
  const end = new THREE.Vector3(marker.points[1].x, marker.points[1].y, marker.points[1].z);
  const dir = end.clone().sub(start);
  const length = dir.length();
  if (length < 1e-6) {
    return null;
  }
  dir.normalize();
  return new THREE.ArrowHelper(dir, start, length, color, Math.max(voxelSize * 1.5, 0.25), Math.max(voxelSize, 0.18), Math.max(voxelSize * 0.65, 0.12));
}

function setSelectionMarkers(markerArray) {
  clearObject(startArrow);
  clearObject(startCube);
  clearObject(goalArrow);
  clearObject(goalCube);
  startArrow = null;
  startCube = null;
  goalArrow = null;
  goalCube = null;

  for (const marker of markerArray.markers) {
    if (marker.type === 0 && marker.id === 0) {
      startArrow = makeArrow(marker, 0x58ef74);
      if (startArrow) {
        scene.add(startArrow);
      }
    } else if (marker.type === 1 && marker.id === 2) {
      startCube = makeCube(marker.pose.position, marker.scale.x, 0x58ef74);
      scene.add(startCube);
    } else if (marker.type === 0 && marker.id === 1) {
      goalArrow = makeArrow(marker, 0xff6767);
      if (goalArrow) {
        scene.add(goalArrow);
      }
    } else if (marker.type === 1 && marker.id === 3) {
      goalCube = makeCube(marker.pose.position, marker.scale.x, 0xff6767);
      scene.add(goalCube);
    }
  }
}

function setPath(pathMsg) {
  if (pathObject) {
    scene.remove(pathObject);
    pathObject.geometry.dispose();
    if (pathObject.material) {
      pathObject.material.dispose();
    }
    pathObject = null;
  }

  if (!pathMsg.poses || pathMsg.poses.length < 2) {
    return;
  }

  const points = pathMsg.poses.map(
    (pose) =>
      new THREE.Vector3(
        pose.pose.position.x,
        pose.pose.position.y,
        pose.pose.position.z,
      ),
  );
  const curve = new THREE.CatmullRomCurve3(points);
  const geometry = new THREE.TubeGeometry(
    curve,
    Math.max(16, points.length * 3),
    Math.max(voxelSize * 0.22, 0.06),
    12,
    false,
  );
  const material = new THREE.MeshStandardMaterial({
    color: 0xb067ff,
    emissive: 0x4f2586,
    roughness: 0.28,
    metalness: 0.08,
  });
  pathObject = new THREE.Mesh(geometry, material);
  scene.add(pathObject);
  scheduleNavigationConfirmation(pathMsg);
}

function setTrackingPoint(marker) {
  clearTrackingPointObjects();

  if (marker.action === 2) {
    return;
  }

  const scale = Math.max(
    0.08,
    Number(marker.scale?.x || marker.scale?.y || marker.scale?.z || voxelSize * 1.5),
  );
  const geometry = new THREE.SphereGeometry(scale * 0.5, 24, 16);
  const material = new THREE.MeshStandardMaterial({
    color: 0x24a6ff,
    emissive: 0x0b4e86,
    roughness: 0.22,
    metalness: 0.08,
  });
  trackingPointObject = new THREE.Mesh(geometry, material);
  trackingPointObject.name = "current-tracking-point";
  trackingPointObject.position.set(
    marker.pose.position.x,
    marker.pose.position.y,
    marker.pose.position.z,
  );
  scene.add(trackingPointObject);
}

function clearTrackingPointObjects() {
  const staleObjects = [];
  scene.traverse((object) => {
    if (object.name === "current-tracking-point") {
      staleObjects.push(object);
    }
  });

  for (const object of staleObjects) {
    if (object.parent) {
      object.parent.remove(object);
    } else {
      scene.remove(object);
    }
    disposeObject(object);
  }
  trackingPointObject = null;
}

function scheduleNavigationConfirmation(pathMsg) {
  if (!pendingNavigationGoal || !pathMsg.poses || pathMsg.poses.length < 2) {
    return;
  }
  if (navigationConfirmTimer) {
    window.clearTimeout(navigationConfirmTimer);
  }

  // goal_point and goal_pose are published back-to-back and can produce two plans.
  // Wait 2 seconds so the planned path is visible before asking for execution.
  navigationConfirmTimer = window.setTimeout(() => {
    navigationConfirmTimer = null;
    confirmNavigationExecution(pathMsg);
  }, 2000);
}

function publishStartNavigation(shouldStart) {
  if (!startNavigationTopic) {
    setSelectionStatus("ROSBridge 未连接，无法发送导航执行确认。");
    return;
  }
  startNavigationTopic.publish(new ROSLIB.Message({ data: shouldStart }));
}

function makeTwist(linearX = 0, linearY = 0, angularZ = 0) {
  return new ROSLIB.Message({
    linear: { x: linearX, y: linearY, z: 0 },
    angular: { x: 0, y: 0, z: angularZ },
  });
}

function updateManualVelocityDisplay() {
  if (!manualVelocityDisplay) {
    return;
  }
  manualVelocityDisplay.textContent =
    `x=${joystickCurrentLinearX.toFixed(3)} y=${joystickCurrentLinearY.toFixed(3)} wz=${joystickCurrentAngularZ.toFixed(3)}`;
}

function publishManualVelocity(force = false) {
  if (!cmdVelTopic) {
    setSelectionStatus("ROSBridge 未连接，无法发送手动速度。");
    updateManualVelocityDisplay();
    return;
  }
  const now = Date.now();
  if (!force && now - joystickLastPublishMs < joystickPublishIntervalMs) {
    return;
  }
  joystickLastPublishMs = now;
  cmdVelTopic.publish(makeTwist(joystickCurrentLinearX, joystickCurrentLinearY, joystickCurrentAngularZ));
  updateManualVelocityDisplay();
}

function publishZeroVelocity() {
  joystickCurrentLinearX = 0;
  joystickCurrentLinearY = 0;
  joystickCurrentAngularZ = 0;
  if (manualRotationSlider) {
    manualRotationSlider.value = "0";
  }
  publishManualVelocity(true);
  stopManualRepeatTimerIfIdle();
}

function applyJoystickSpeedCurve(normalizedValue, maxSpeed) {
  const sign = Math.sign(normalizedValue);
  const magnitude = Math.abs(normalizedValue);
  if (magnitude < joystickDeadband) {
    return 0;
  }

  const scaled = (magnitude - joystickDeadband) / (1 - joystickDeadband) * maxSpeed;
  return sign * Math.max(joystickMinCommandSpeed, scaled);
}

function manualVelocityIsActive() {
  return (
    Math.abs(joystickCurrentLinearX) > 1e-6 ||
    Math.abs(joystickCurrentLinearY) > 1e-6 ||
    Math.abs(joystickCurrentAngularZ) > 1e-6
  );
}

function ensureManualRepeatTimer() {
  if (joystickRepeatTimer) {
    return;
  }
  joystickRepeatTimer = window.setInterval(() => {
    if (manualVelocityIsActive()) {
      publishManualVelocity(true);
    }
  }, 100);
}

function stopManualRepeatTimerIfIdle() {
  if (!joystickRepeatTimer || manualVelocityIsActive()) {
    return;
  }
  window.clearInterval(joystickRepeatTimer);
  joystickRepeatTimer = null;
}

function stopNavigation() {
  if (!stopNavigationTopic) {
    setSelectionStatus("ROSBridge 未连接，无法发送停止导航命令。");
    return;
  }
  stopPathTrackingForManualMotion();
  publishZeroVelocity();
  setSelectionStatus("已发送停止导航命令：路径跟踪已中止，并请求底盘速度归零。");
}

function stopPathTrackingForManualMotion() {
  if (!stopNavigationTopic) {
    return;
  }
  pendingNavigationGoal = null;
  pendingNavigationPath = null;
  hideNavigationConfirmModal();
  if (navigationConfirmTimer) {
    window.clearTimeout(navigationConfirmTimer);
    navigationConfirmTimer = null;
  }
  publishStartNavigation(false);
  stopNavigationTopic.publish(new ROSLIB.Message({ data: true }));
}

function resetJoystickKnob() {
  joystickKnob.style.transform = "translate(-50%, -50%)";
  joystickKnob.classList.remove("active");
}

function updateJoystickFromEvent(event, forcePublish = false) {
  const rect = joystickPad.getBoundingClientRect();
  const radius = rect.width * 0.5;
  const knobRadius = joystickKnob.getBoundingClientRect().width * 0.5;
  const maxOffset = Math.max(1, radius - knobRadius - 6);
  const centerX = rect.left + radius;
  const centerY = rect.top + radius;
  let dx = event.clientX - centerX;
  let dy = event.clientY - centerY;
  const distance = Math.hypot(dx, dy);
  if (distance > maxOffset) {
    dx = dx / distance * maxOffset;
    dy = dy / distance * maxOffset;
  }

  joystickKnob.style.transform = `translate(calc(-50% + ${dx}px), calc(-50% + ${dy}px))`;

  const normalizedX = dx / maxOffset;
  const normalizedY = dy / maxOffset;
  const linearX = applyJoystickSpeedCurve(-normalizedY, joystickMaxLinearX);
  const linearY = applyJoystickSpeedCurve(-normalizedX, joystickMaxLinearY);
  joystickCurrentLinearX = linearX;
  joystickCurrentLinearY = linearY;
  publishManualVelocity(forcePublish);
}

function startJoystickControl(event) {
  if (event.button !== undefined && event.button !== 0) {
    return;
  }
  event.preventDefault();
  joystickActivePointerId = event.pointerId;
  joystickKnob.classList.add("active");
  joystickKnob.setPointerCapture(event.pointerId);
  updateJoystickFromEvent(event, true);
  ensureManualRepeatTimer();
  setSelectionStatus("手动运动控制中：松开小圆将回中并发送零速度。");
}

function moveJoystickControl(event) {
  if (joystickActivePointerId !== event.pointerId) {
    return;
  }
  event.preventDefault();
  updateJoystickFromEvent(event);
}

function endJoystickControl(event) {
  if (joystickActivePointerId !== event.pointerId) {
    return;
  }
  event.preventDefault();
  joystickActivePointerId = null;
  joystickCurrentLinearX = 0;
  joystickCurrentLinearY = 0;
  resetJoystickKnob();
  publishManualVelocity(true);
  stopManualRepeatTimerIfIdle();
  setSelectionStatus(
    joystickCurrentAngularZ === 0
      ? "手动运动已停止，已发送零速度。"
      : "平移控制已停止，旋转滑杆仍在发送角速度。"
  );
  if (joystickKnob.hasPointerCapture(event.pointerId)) {
    joystickKnob.releasePointerCapture(event.pointerId);
  }
}

function updateManualRotationFromSlider(forcePublish = false) {
  if (!manualRotationSlider) {
    return;
  }
  const sliderValue = Number(manualRotationSlider.value || 0);
  const normalized = Math.max(-1, Math.min(1, sliderValue / 100));
  joystickCurrentAngularZ = applyJoystickSpeedCurve(-normalized, joystickMaxAngularZ);
  publishManualVelocity(forcePublish);
  if (manualVelocityIsActive()) {
    ensureManualRepeatTimer();
  } else {
    stopManualRepeatTimerIfIdle();
  }
}

function resetManualRotationSlider(forcePublish = true) {
  if (!manualRotationSlider) {
    return;
  }
  manualRotationSlider.value = "0";
  updateManualRotationFromSlider(forcePublish);
}

function confirmNavigationExecution(pathMsg) {
  if (!pendingNavigationGoal) {
    return;
  }

  const goal = pendingNavigationGoal.goal;
  const yaw = pendingNavigationGoal.yaw;
  pendingNavigationPath = pathMsg;
  navigationConfirmMessage.textContent = [
    `路径点数：${pathMsg.poses.length}`,
    `目标位置：[${goal.x.toFixed(2)}, ${goal.y.toFixed(2)}, ${goal.z.toFixed(2)}]`,
    `目标朝向：${(yaw * 180 / Math.PI).toFixed(1)}°`,
    "",
    "点击“开始导航”会启动路径跟踪。",
    "点击“只显示路线”会显示三维路线，但不执行导航。",
  ].join("\n");
  navigationConfirmModal.hidden = false;
  setSelectionStatus("路径规划完成，请在弹窗中确认是否开始导航。");
}

function hideNavigationConfirmModal() {
  navigationConfirmModal.hidden = true;
}

function resolveNavigationConfirmation(shouldStart) {
  if (!pendingNavigationGoal && !pendingNavigationPath) {
    hideNavigationConfirmModal();
    return;
  }
  pendingNavigationGoal = null;
  pendingNavigationPath = null;
  hideNavigationConfirmModal();
  publishStartNavigation(shouldStart);
  setSelectionStatus(
    shouldStart
      ? "已确认开始导航，路径跟踪已启动。"
      : "已取消导航执行，仅显示规划路线，不进行路径跟踪。",
  );
}

function makePointStamped(x, y, z) {
  const now = Date.now();
  return new ROSLIB.Message({
    header: {
      frame_id: "map",
      stamp: {
        sec: Math.floor(now / 1000),
        nanosec: (now % 1000) * 1000000,
      },
    },
    point: { x, y, z },
  });
}

function makePoseStamped(x, y, z, yaw) {
  const now = Date.now();
  const halfYaw = yaw * 0.5;
  return new ROSLIB.Message({
    header: {
      frame_id: "map",
      stamp: {
        sec: Math.floor(now / 1000),
        nanosec: (now % 1000) * 1000000,
      },
    },
    pose: {
      position: { x, y, z },
      orientation: {
        x: 0,
        y: 0,
        z: Math.sin(halfYaw),
        w: Math.cos(halfYaw),
      },
    },
  });
}

function makePoseWithCovarianceStamped(x, y, z, yaw) {
  const now = Date.now();
  const halfYaw = yaw * 0.5;
  const covariance = new Array(36).fill(0);
  covariance[0] = 0.25;
  covariance[7] = 0.25;
  covariance[35] = 0.06853891909122467;
  return new ROSLIB.Message({
    header: {
      frame_id: "map",
      stamp: {
        sec: Math.floor(now / 1000),
        nanosec: (now % 1000) * 1000000,
      },
    },
    pose: {
      pose: {
        position: { x, y, z },
        orientation: {
          x: 0,
          y: 0,
          z: Math.sin(halfYaw),
          w: Math.cos(halfYaw),
        },
      },
      covariance,
    },
  });
}

function setPointVisual(kind, center, yaw) {
  const isStart = kind === "start";
  const color = isStart ? 0x58ef74 : 0xff6767;
  const arrowRef = isStart ? startArrow : goalArrow;
  const cubeRef = isStart ? startCube : goalCube;

  clearObject(arrowRef);
  clearObject(cubeRef);
  if (isStart) {
    startArrow = null;
    startCube = null;
  } else {
    goalArrow = null;
    goalCube = null;
  }

  const cube = makeCube(center, Math.max(voxelSize, 0.18), color);
  scene.add(cube);
  const direction = new THREE.Vector3(Math.cos(yaw), Math.sin(yaw), 0);
  const arrowLength = Math.max(voxelSize * 2.5, 0.45);
  const arrow = new THREE.ArrowHelper(
    direction.normalize(),
    new THREE.Vector3(center.x, center.y, center.z),
    arrowLength,
    color,
    Math.max(voxelSize * 1.5, 0.25),
    Math.max(voxelSize, 0.18),
  );
  scene.add(arrow);

  if (isStart) {
    startCube = cube;
    startArrow = arrow;
  } else {
    goalCube = cube;
    goalArrow = arrow;
  }
}

function yawFromQuaternion(qx, qy, qz, qw) {
  const sinyCosp = 2.0 * (qw * qz + qx * qy);
  const cosyCosp = 1.0 - 2.0 * (qy * qy + qz * qz);
  return Math.atan2(sinyCosp, cosyCosp);
}

function normalizeFrameId(frameId) {
  if (!frameId) {
    return "";
  }
  return String(frameId).replace(/^\/+/, "");
}

function tfKey(parent, child) {
  return `${normalizeFrameId(parent)}->${normalizeFrameId(child)}`;
}

function invertTransform(transform) {
  if (!transform) {
    return null;
  }
  const translation = transform.translation || { x: 0, y: 0, z: 0 };
  const rotation = transform.rotation || { x: 0, y: 0, z: 0, w: 1 };

  const q = new THREE.Quaternion(rotation.x, rotation.y, rotation.z, rotation.w).normalize();
  const qInv = q.clone().invert();
  const t = new THREE.Vector3(translation.x, translation.y, translation.z);
  const invT = t.clone().applyQuaternion(qInv).multiplyScalar(-1);

  return {
    translation: { x: invT.x, y: invT.y, z: invT.z },
    rotation: { x: qInv.x, y: qInv.y, z: qInv.z, w: qInv.w },
  };
}

function getTfTransform(parent, child) {
  const direct = tfState.get(tfKey(parent, child));
  if (direct) {
    return direct;
  }
  const reverse = tfState.get(tfKey(child, parent));
  if (reverse) {
    return invertTransform(reverse);
  }
  return null;
}

function getTfTransformAny(parents, children) {
  for (const parent of parents) {
    for (const child of children) {
      const tf = getTfTransform(parent, child);
      if (tf) {
        return tf;
      }
    }
  }
  return null;
}

function getTfTransformAnyMatch(parents, children) {
  for (const parent of parents) {
    for (const child of children) {
      const transform = getTfTransform(parent, child);
      if (transform) {
        return { transform, parent, child };
      }
    }
  }
  return null;
}

function findTransformByChildPrefix(parent, childSuffixes) {
  const match = findTransformByChildPrefixMatch(parent, childSuffixes);
  return match ? match.transform : null;
}

function findTransformByChildPrefixMatch(parent, childSuffixes) {
  const normalizedParent = normalizeFrameId(parent);
  for (const [key, transform] of tfState.entries()) {
    const parts = key.split("->");
    if (parts.length !== 2) {
      continue;
    }
    const keyParent = normalizeFrameId(parts[0]);
    const keyChild = normalizeFrameId(parts[1]);
    if (keyParent !== normalizedParent) {
      continue;
    }
    if (childSuffixes.some((suffix) => keyChild === suffix || keyChild.endsWith(`/${suffix}`) || keyChild.endsWith(`_${suffix}`))) {
      return { transform, parent: keyParent, child: keyChild };
    }
  }
  return null;
}

function applyRobotCenterOffset(pose, sourceFrame) {
  if (normalizeFrameId(sourceFrame) !== robotCenterOffsetFrame) {
    return pose;
  }

  const cosYaw = Math.cos(pose.yaw);
  const sinYaw = Math.sin(pose.yaw);
  return {
    x: pose.x + cosYaw * robotCenterOffset.x - sinYaw * robotCenterOffset.y,
    y: pose.y + sinYaw * robotCenterOffset.x + cosYaw * robotCenterOffset.y,
    z: pose.z + robotCenterOffset.z,
    yaw: pose.yaw,
  };
}

function storeTransformMessage(msg) {
  if (!msg || !msg.transforms) {
    return;
  }
  for (const transformStamped of msg.transforms) {
    const key = tfKey(
      transformStamped.header.frame_id,
      transformStamped.child_frame_id,
    );
    tfState.set(key, transformStamped.transform);
  }
}

function updateRelocalizationStatus() {
  setRelocalizationStatus(Boolean(getTfTransform("map", "odom")));
}

function resolveRobotPose() {
  const baseCandidates = [
    "odin1_base_link",
    "robot_base_link",
    "base_link",
    "base_footprint",
  ];
  const directMapBase =
    getTfTransformAnyMatch(["map"], baseCandidates) ||
    findTransformByChildPrefixMatch("map", ["odin1_base_link", "robot_base_link", "base_link", "base_footprint"]);
  if (directMapBase) {
    setRobotTfStatus(`直接使用 map -> ${directMapBase.child}`);
    return applyRobotCenterOffset(
      {
        x: directMapBase.transform.translation.x,
        y: directMapBase.transform.translation.y,
        z: directMapBase.transform.translation.z,
        yaw: yawFromQuaternion(
          directMapBase.transform.rotation.x,
          directMapBase.transform.rotation.y,
          directMapBase.transform.rotation.z,
          directMapBase.transform.rotation.w,
        ),
      },
      directMapBase.child,
    );
  }

  const mapToOdom = getTfTransform("map", "odom");
  const odomToBase =
    getTfTransformAnyMatch(["odom"], baseCandidates) ||
    findTransformByChildPrefixMatch("odom", ["odin1_base_link", "robot_base_link", "base_link", "base_footprint"]);
  if (mapToOdom && odomToBase) {
    setRobotTfStatus(`使用 map -> odom -> ${odomToBase.child}`);
    const mapYaw = yawFromQuaternion(
      mapToOdom.rotation.x,
      mapToOdom.rotation.y,
      mapToOdom.rotation.z,
      mapToOdom.rotation.w,
    );
    const odomYaw = yawFromQuaternion(
      odomToBase.transform.rotation.x,
      odomToBase.transform.rotation.y,
      odomToBase.transform.rotation.z,
      odomToBase.transform.rotation.w,
    );

    const cosYaw = Math.cos(mapYaw);
    const sinYaw = Math.sin(mapYaw);
    const bx = odomToBase.transform.translation.x;
    const by = odomToBase.transform.translation.y;
    return applyRobotCenterOffset(
      {
        x: mapToOdom.translation.x + cosYaw * bx - sinYaw * by,
        y: mapToOdom.translation.y + sinYaw * bx + cosYaw * by,
        z: mapToOdom.translation.z + odomToBase.transform.translation.z,
        yaw: mapYaw + odomYaw,
      },
      odomToBase.child,
    );
  }

  const availableKeys = Array.from(tfState.keys())
    .filter((key) => key.includes("base") || key.startsWith("map->") || key.startsWith("odom->"))
    .slice(0, 10)
    .join(", ");
  setRobotTfStatus(availableKeys ? `未匹配，已收到：${availableKeys}` : "未收到 map/odom/base 相关 TF");

  return null;
}

function updateRobotVisual() {
  if (!robotObject) {
    return;
  }
  const robotPose = resolveRobotPose();
  if (!robotPose) {
    robotObject.visible = false;
    return;
  }

  robotObject.visible = true;
  robotObject.position.set(
    robotPose.x + robotDisplayOffset.x,
    robotPose.y + robotDisplayOffset.y,
    robotPose.z + robotDisplayOffset.z,
  );
  robotObject.rotation.set(0, 0, robotPose.yaw);
}

function refreshLayerVisibility() {
  if (occupiedPointsObject) {
    occupiedPointsObject.visible = toggleOccupied.checked;
  }
  if (preblockedPointsObject) {
    preblockedPointsObject.visible = togglePreblocked.checked;
  }
  if (traversablePointsObject) {
    traversablePointsObject.visible = toggleTraversable.checked;
  }
  if (riskPointsObject) {
    riskPointsObject.visible = toggleRisk.checked;
  }
}

function publishStartPose(intersectionPoint, yaw) {
  if (!startTopic) {
    setSelectionStatus("ROSBridge 未连接。");
    return;
  }
  startTopic.publish(makePointStamped(intersectionPoint.x, intersectionPoint.y, intersectionPoint.z));
  setPointVisual("start", intersectionPoint, yaw);
  setSelectionStatus(
    `起始点已设置：[${intersectionPoint.x.toFixed(2)}, ${intersectionPoint.y.toFixed(2)}, ${intersectionPoint.z.toFixed(2)}]，朝向 ${(yaw * 180 / Math.PI).toFixed(1)}°。`,
  );
}

function publishCurrentPose(intersectionPoint, yaw) {
  if (!initialPoseTopic) {
    setSelectionStatus("ROSBridge 未连接。");
    return;
  }
  initialPoseTopic.publish(
    makePoseWithCovarianceStamped(
      intersectionPoint.x,
      intersectionPoint.y,
      intersectionPoint.z,
      yaw,
    ),
  );
  clearObject(startArrow);
  clearObject(startCube);
  startArrow = null;
  startCube = null;
  setSelectionStatus(
    `当前姿态已设置：[${intersectionPoint.x.toFixed(2)}, ${intersectionPoint.y.toFixed(2)}, ${intersectionPoint.z.toFixed(2)}]，朝向 ${(yaw * 180 / Math.PI).toFixed(1)}°。`,
  );
}

function publishGoalPose(intersectionPoint, yaw) {
  if (!goalTopic || !goalPoseTopic) {
    setSelectionStatus("ROSBridge 未连接。");
    return;
  }
  goalTopic.publish(makePointStamped(intersectionPoint.x, intersectionPoint.y, intersectionPoint.z));
  goalPoseTopic.publish(makePoseStamped(intersectionPoint.x, intersectionPoint.y, intersectionPoint.z, yaw));
  setPointVisual("goal", intersectionPoint, yaw);
  setSelectionStatus(
    `目标点已设置：[${intersectionPoint.x.toFixed(2)}, ${intersectionPoint.y.toFixed(2)}, ${intersectionPoint.z.toFixed(2)}]，朝向 ${(yaw * 180 / Math.PI).toFixed(1)}°，正在规划。`,
  );
}

function publishNavigationGoal(intersectionPoint, yaw) {
  if (!startTopic || !goalTopic || !goalPoseTopic || !startNavigationTopic) {
    setSelectionStatus("ROSBridge 未连接。");
    return;
  }
  const robotPose = resolveRobotPose();
  if (!robotPose) {
    setSelectionStatus("未收到机器人 TF，无法设置导航目标。");
    return;
  }
  publishStartNavigation(false);
  pendingNavigationGoal = {
    goal: { x: intersectionPoint.x, y: intersectionPoint.y, z: intersectionPoint.z },
    yaw,
  };
  startTopic.publish(makePointStamped(robotPose.x, robotPose.y, robotPose.z));
  goalTopic.publish(makePointStamped(intersectionPoint.x, intersectionPoint.y, intersectionPoint.z));
  goalPoseTopic.publish(
    makePoseStamped(intersectionPoint.x, intersectionPoint.y, intersectionPoint.z, yaw),
  );
  setPointVisual("start", { x: robotPose.x, y: robotPose.y, z: robotPose.z }, robotPose.yaw);
  setPointVisual("goal", intersectionPoint, yaw);
  setSelectionStatus(
    `导航目标已设置：[${intersectionPoint.x.toFixed(2)}, ${intersectionPoint.y.toFixed(2)}, ${intersectionPoint.z.toFixed(2)}]，朝向 ${(yaw * 180 / Math.PI).toFixed(1)}°。正在规划路径，路径显示后请确认是否开始导航。`,
  );
}

function updatePointerFromEvent(event) {
  const rect = canvas.getBoundingClientRect();
  pointer.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
  pointer.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
  raycaster.setFromCamera(pointer, camera);
}

function pickTraversablePoint(event) {
  if (!traversablePickObject) {
    return null;
  }
  updatePointerFromEvent(event);
  const hits = raycaster.intersectObject(traversablePickObject);
  if (hits.length === 0) {
    return null;
  }
  return hits[0].point.clone();
}

function pickPointOnHeightPlane(event, planeZ) {
  updatePointerFromEvent(event);
  const plane = new THREE.Plane(new THREE.Vector3(0, 0, 1), -planeZ);
  const hit = new THREE.Vector3();
  const ok = raycaster.ray.intersectPlane(plane, hit);
  return ok ? hit : null;
}

function computeYawFromPoints(start, end, fallbackYaw = 0) {
  const dx = end.x - start.x;
  const dy = end.y - start.y;
  if (Math.abs(dx) < 1e-6 && Math.abs(dy) < 1e-6) {
    return fallbackYaw;
  }
  return Math.atan2(dy, dx);
}

function scheduleReconnect() {
  if (reconnectTimer) {
    return;
  }
  reconnectTimer = window.setTimeout(() => {
    reconnectTimer = null;
    connectRosbridge(wsInput.value.trim(), false);
  }, 1500);
}

function connectRosbridge(url = wsInput.value.trim(), manual = false) {
  wsInput.value = url;
  if (ros) {
    ros.close();
  }

  setConnectionStatus("Connecting");
  setSelectionStatus(`正在连接 ${url} ...`);

  ros = new ROSLIB.Ros({ url });

  ros.on("connection", () => {
    setConnectionStatus("已连接");
    setSelectionStatus("已连接。点击“导航目标”“起始点”或“目标点”，再在可通行栅格上按下、拖动、松开设置姿态。");
    advancedSettings.open = false;

    startTopic = new ROSLIB.Topic({
      ros,
      name: "/start_point",
      messageType: "geometry_msgs/PointStamped",
    });
    goalTopic = new ROSLIB.Topic({
      ros,
      name: "/goal_point",
      messageType: "geometry_msgs/PointStamped",
    });
    goalPoseTopic = new ROSLIB.Topic({
      ros,
      name: "/goal_pose",
      messageType: "geometry_msgs/PoseStamped",
    });
    startNavigationTopic = new ROSLIB.Topic({
      ros,
      name: "/start_navigation",
      messageType: "std_msgs/Bool",
    });
    stopNavigationTopic = new ROSLIB.Topic({
      ros,
      name: "/stop_navigation",
      messageType: "std_msgs/Bool",
    });
    cmdVelTopic = new ROSLIB.Topic({
      ros,
      name: "/web_cmd_vel",
      messageType: "geometry_msgs/Twist",
    });
    initialPoseTopic = new ROSLIB.Topic({
      ros,
      name: "/initialpose",
      messageType: "geometry_msgs/PoseWithCovarianceStamped",
    });

    const occupiedTopic = new ROSLIB.Topic({
      ros,
      name: "/octomap_occupied_markers",
      messageType: "visualization_msgs/Marker",
    });
    occupiedTopic.subscribe(setOccupiedPoints);

    const preblockedTopic = new ROSLIB.Topic({
      ros,
      name: "/preblocked_cells_markers",
      messageType: "visualization_msgs/Marker",
    });
    preblockedTopic.subscribe(setPreblockedPoints);

    const traversableTopic = new ROSLIB.Topic({
      ros,
      name: "/traversable_cells_markers",
      messageType: "visualization_msgs/Marker",
    });
    traversableTopic.subscribe(setTraversablePoints);

    const riskTopic = new ROSLIB.Topic({
      ros,
      name: "/risk_cost_cells",
      messageType: "sensor_msgs/PointCloud2",
    });
    riskTopic.subscribe(setRiskPoints);

    const selectionTopic = new ROSLIB.Topic({
      ros,
      name: "/selection_markers",
      messageType: "visualization_msgs/MarkerArray",
    });
    selectionTopic.subscribe(setSelectionMarkers);

    const pathTopic = new ROSLIB.Topic({
      ros,
      name: "/planned_path",
      messageType: "nav_msgs/Path",
    });
    pathTopic.subscribe(setPath);

    const trackingPointTopic = new ROSLIB.Topic({
      ros,
      name: "/tracking_point_marker",
      messageType: "visualization_msgs/Marker",
    });
    trackingPointTopic.subscribe(setTrackingPoint);

    const statusTopic = new ROSLIB.Topic({
      ros,
      name: "/web_selection_status",
      messageType: "std_msgs/String",
    });
    statusTopic.subscribe((msg) => setSelectionStatus(msg.data));

    tfTopic = new ROSLIB.Topic({
      ros,
      name: "/tf",
      messageType: "tf2_msgs/TFMessage",
    });
    tfTopic.subscribe(storeTransformMessage);

    tfStaticTopic = new ROSLIB.Topic({
      ros,
      name: "/tf_static",
      messageType: "tf2_msgs/TFMessage",
    });
    tfStaticTopic.subscribe(storeTransformMessage);
    updateRelocalizationStatus();
  });

  ros.on("error", (error) => {
    setConnectionStatus("错误");
    setSelectionStatus(`ROSBridge 错误：${error}`);
    advancedSettings.open = true;
  });

  ros.on("close", () => {
    setConnectionStatus("未连接");
    startTopic = null;
    goalTopic = null;
    goalPoseTopic = null;
    initialPoseTopic = null;
    startNavigationTopic = null;
    stopNavigationTopic = null;
    cmdVelTopic = null;
    tfTopic = null;
    tfStaticTopic = null;
    tfState.clear();
    updateRelocalizationStatus();
    pendingNavigationGoal = null;
    pendingNavigationPath = null;
    hideNavigationConfirmModal();
    if (navigationConfirmTimer) {
      window.clearTimeout(navigationConfirmTimer);
      navigationConfirmTimer = null;
    }
    advancedSettings.open = true;
    if (!manual) {
      scheduleReconnect();
    }
  });
}

function setPlacementMode(mode) {
  placementMode = mode;
  if (setCurrentPoseBtn) {
    setCurrentPoseBtn.classList.toggle("active", mode === "current_pose");
  }
  setNavigateBtn.classList.toggle("active", mode === "navigate");
  setStartBtn.classList.toggle("active", mode === "start");
  setGoalBtn.classList.toggle("active", mode === "goal");
  if (mode === "current_pose") {
    setSelectionStatus("当前姿态模式：在绿色可通行栅格上按下，拖动调整朝向，松开确认。");
  } else if (mode === "navigate") {
    setSelectionStatus("导航目标模式：松开后先规划并显示路线，再确认是否开始导航。");
  } else if (mode === "start") {
    setSelectionStatus("起始点模式：在绿色可通行栅格上按下，拖动调整朝向，松开确认。");
  } else if (mode === "goal") {
    setSelectionStatus("目标点模式：在绿色可通行栅格上按下，拖动调整朝向，松开确认并开始规划。");
  } else {
    setSelectionStatus("已连接。点击“导航目标”“起始点”或“目标点”，再在可通行栅格上按下、拖动、松开设置姿态。");
  }
}

if (setCurrentPoseBtn) {
  setCurrentPoseBtn.addEventListener("click", () => setPlacementMode("current_pose"));
}
setNavigateBtn.addEventListener("click", () => setPlacementMode("navigate"));
setStartBtn.addEventListener("click", () => setPlacementMode("start"));
setGoalBtn.addEventListener("click", () => setPlacementMode("goal"));
stopNavigationBtn.addEventListener("click", stopNavigation);
joystickKnob.addEventListener("pointerdown", startJoystickControl);
joystickKnob.addEventListener("pointermove", moveJoystickControl);
joystickKnob.addEventListener("pointerup", endJoystickControl);
joystickKnob.addEventListener("pointercancel", endJoystickControl);
if (manualRotationSlider) {
  manualRotationSlider.addEventListener("input", () => {
    updateManualRotationFromSlider(true);
  });
  manualRotationSlider.addEventListener("change", () => resetManualRotationSlider(true));
  manualRotationSlider.addEventListener("pointerup", () => resetManualRotationSlider(true));
  manualRotationSlider.addEventListener("pointercancel", () => resetManualRotationSlider(true));
  manualRotationSlider.addEventListener("touchend", () => resetManualRotationSlider(true), { passive: true });
}
navigationConfirmStartBtn.addEventListener("click", () => resolveNavigationConfirmation(true));
navigationConfirmCancelBtn.addEventListener("click", () => resolveNavigationConfirmation(false));
toggleOccupied.addEventListener("change", refreshLayerVisibility);
toggleTraversable.addEventListener("change", refreshLayerVisibility);
togglePreblocked.addEventListener("change", refreshLayerVisibility);
toggleRisk.addEventListener("change", refreshLayerVisibility);
resetViewBtn.addEventListener("click", resetMapView);

canvas.addEventListener("pointerdown", (event) => {
  if (!placementMode) {
    setSelectionStatus("请先点击“导航目标”“起始点”或“目标点”按钮。");
    return;
  }
  if (event.button !== 0) {
    return;
  }
  const point = pickTraversablePoint(event);
  if (!point) {
    setSelectionStatus("没有点中可通行栅格，请按下绿色可通行栅格设置导航目标。");
    return;
  }
  activePointerId = event.pointerId;
  navigationDrag = {
    start: point,
    yaw: 0,
    mode: placementMode,
  };
  controls.enabled = false;
  canvas.setPointerCapture(event.pointerId);
  setPointVisual(
    placementMode === "current_pose" || placementMode === "start" ? "start" : "goal",
    point,
    0,
  );
  setSelectionStatus(
    placementMode === "current_pose"
      ? "当前姿态位置已设置。保持按下并拖动以调整绿色箭头朝向。"
      : placementMode === "navigate"
        ? "导航目标位置已设置。保持按下并拖动以调整红色箭头朝向。"
        : placementMode === "start"
          ? "起始点位置已设置。保持按下并拖动以调整绿色箭头朝向。"
          : "目标点位置已设置。保持按下并拖动以调整红色箭头朝向。"
  );
});

canvas.addEventListener("pointermove", (event) => {
  if (activePointerId !== event.pointerId || !navigationDrag) {
    return;
  }
  const point = pickPointOnHeightPlane(event, navigationDrag.start.z);
  if (!point) {
    return;
  }
  navigationDrag.yaw = computeYawFromPoints(navigationDrag.start, point, navigationDrag.yaw);
  setPointVisual(
    navigationDrag.mode === "current_pose" || navigationDrag.mode === "start" ? "start" : "goal",
    navigationDrag.start,
    navigationDrag.yaw,
  );
});

canvas.addEventListener("pointerup", (event) => {
  if (activePointerId !== event.pointerId || !navigationDrag) {
    return;
  }
  event.preventDefault();
  const point = pickPointOnHeightPlane(event, navigationDrag.start.z);
  if (point) {
    navigationDrag.yaw = computeYawFromPoints(navigationDrag.start, point, navigationDrag.yaw);
  }
  if (navigationDrag.mode === "current_pose") {
    publishCurrentPose(navigationDrag.start, navigationDrag.yaw);
  } else if (navigationDrag.mode === "start") {
    publishStartPose(navigationDrag.start, navigationDrag.yaw);
  } else if (navigationDrag.mode === "goal") {
    publishGoalPose(navigationDrag.start, navigationDrag.yaw);
  } else {
    publishNavigationGoal(navigationDrag.start, navigationDrag.yaw);
  }
  navigationDrag = null;
  activePointerId = null;
  controls.enabled = true;
  setPlacementMode(null);
  if (canvas.hasPointerCapture(event.pointerId)) {
    canvas.releasePointerCapture(event.pointerId);
  }
});

canvas.addEventListener("pointercancel", (event) => {
  if (activePointerId !== event.pointerId) {
    return;
  }
  navigationDrag = null;
  activePointerId = null;
  controls.enabled = true;
  setPlacementMode(null);
  if (canvas.hasPointerCapture(event.pointerId)) {
    canvas.releasePointerCapture(event.pointerId);
  }
});

connectBtn.addEventListener("click", () => connectRosbridge(wsInput.value.trim(), true));
reconnectBtn.addEventListener("click", () => connectRosbridge(defaultRosbridgeUrl(), true));
window.addEventListener("resize", updateRendererSize);
updateRendererSize();
connectRosbridge(defaultRosbridgeUrl(), false);

function animate() {
  controls.update();
  updateRobotVisual();
  renderer.render(scene, camera);
  requestAnimationFrame(animate);
}

window.setInterval(updateRelocalizationStatus, 1000);

animate();
