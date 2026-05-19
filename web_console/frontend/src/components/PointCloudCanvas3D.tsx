import { useEffect, useMemo, useRef, useState } from "react";

import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { PCDLoader } from "three/examples/jsm/loaders/PCDLoader.js";

import { buildMapFileUrl } from "../api";
import type { NavigationGoal, PointCloudSnapshot, RobotPose, SavedMapInfo, VirtualObstacleZone } from "../types";

interface PointCloudCanvas3DProps {
  pointcloud: PointCloudSnapshot | null;
  selectedMap: SavedMapInfo | null;
  selectedPointcloudPath: string | null;
  pose: RobotPose | null;
  obstacles: VirtualObstacleZone[];
  selectedGoal: NavigationGoal | null;
  activeGoal: NavigationGoal | null;
  onSelectGoal: (goal: NavigationGoal) => void;
}

interface SceneContext {
  renderer: THREE.WebGLRenderer;
  scene: THREE.Scene;
  camera: THREE.PerspectiveCamera;
  controls: OrbitControls;
  grid: THREE.GridHelper;
  obstacleGroup: THREE.Group;
  savedPoints: THREE.Points | null;
  livePoints: THREE.Points | null;
  robotMarker: THREE.Group;
  selectedGoalMarker: THREE.Group;
  activeGoalMarker: THREE.Group;
  savedCount: number;
  liveCount: number;
  activeBounds: THREE.Box3 | null;
  preset: ViewPreset;
  savedAssetKey: string | null;
  sceneOrigin: SceneOrigin | null;
  hasAutoFramed: boolean;
  animationFrame: number | null;
}

type ViewPreset = "iso" | "front" | "top";
type SceneOrigin = { x: number; y: number };

const SAVED_POINT_COLOR = new THREE.Color("#8ce7ff");
const LIVE_POINT_COLOR = new THREE.Color("#f97316");
const SELECTED_GOAL_COLOR = new THREE.Color("#f59e0b");
const ACTIVE_GOAL_COLOR = new THREE.Color("#22c55e");
const ROBOT_BODY_COLOR = new THREE.Color("#facc15");
const ROBOT_OUTLINE_COLOR = new THREE.Color("#111827");
const ROBOT_HEADING_COLOR = new THREE.Color("#ef4444");
const GROUND_SEARCH_RADIUS_M = 0.85;
const GROUND_HEIGHT_QUANTILE = 0.12;
const DISPLAY_GROUND_HEIGHT_QUANTILE = 0.03;
const GROUND_PICK_RADIUS_PX = 24;
const ZERO_SCENE_ORIGIN: SceneOrigin = { x: 0, y: 0 };

export function PointCloudCanvas3D({
  pointcloud,
  selectedMap,
  selectedPointcloudPath,
  pose,
  obstacles,
  selectedGoal,
  activeGoal,
  onSelectGoal,
}: PointCloudCanvas3DProps) {
  const viewportRef = useRef<HTMLDivElement | null>(null);
  const sceneRef = useRef<SceneContext | null>(null);
  const showSavedMapRef = useRef(true);
  const poseFrameRef = useRef<string | null>(null);
  const pointcloudFrameRef = useRef<string | null>(null);
  const onSelectGoalRef = useRef(onSelectGoal);
  const lastRobotPoseRef = useRef<{ x: number; y: number; yaw: number }>({ x: 0, y: 0, yaw: 0 });
  const [artifactState, setArtifactState] = useState("等待 3D 资产");
  const [showSavedMap, setShowSavedMap] = useState(true);
  const [showLiveOverlay, setShowLiveOverlay] = useState(false);
  const [renderStats, setRenderStats] = useState({ saved: 0, live: 0 });
  const [renderError, setRenderError] = useState<string | null>(null);
  const [sceneOriginVersion, setSceneOriginVersion] = useState(0);
  const [sceneOriginLabel, setSceneOriginLabel] = useState("origin=等待定位");

  const selectedSnapshotArtifact = useMemo(
    () => selectedMap?.artifacts.find((artifact) => artifact.kind === "pointcloud_snapshot_3d") ?? null,
    [selectedMap],
  );
  const selectedNativeArtifact = useMemo(
    () => selectedMap?.artifacts.find((artifact) => artifact.kind === "native_pointcloud_map_3d") ?? null,
    [selectedMap],
  );
  const activeSavedPointcloudPath = useMemo(
    () => selectedPointcloudPath ?? selectedSnapshotArtifact?.path ?? selectedNativeArtifact?.path ?? null,
    [selectedNativeArtifact, selectedPointcloudPath, selectedSnapshotArtifact],
  );
  const selectedMapId = selectedMap?.map_id ?? null;
  const savedAssetKey = selectedMapId && activeSavedPointcloudPath ? `${selectedMapId}:${activeSavedPointcloudPath}` : null;
  const livePointCount = pointcloud?.loaded ? pointcloud.points.length : 0;
  const hasSavedMap = renderStats.saved > 0;
  const sourceLabel = hasSavedMap ? activeSavedPointcloudPath ?? "saved pointcloud_map_3d" : pointcloud?.source_topic ?? "none";
  const compactPoseLabel =
    pose?.available && pose.x !== null && pose.y !== null
      ? `robot ${pose.x.toFixed(2)}, ${pose.y.toFixed(2)} yaw ${((pose.yaw ?? 0) * 180 / Math.PI).toFixed(0)}deg`
      : "robot 暂无";
  const compactPoseStampLabel = pose?.stamp ? `stamp ${pose.stamp.slice(11, 19)}` : "stamp 暂无";

  useEffect(() => {
    poseFrameRef.current = pose?.frame_id ?? null;
  }, [pose?.frame_id]);

  useEffect(() => {
    pointcloudFrameRef.current = pointcloud?.frame_id ?? null;
  }, [pointcloud?.frame_id]);

  useEffect(() => {
    onSelectGoalRef.current = onSelectGoal;
  }, [onSelectGoal]);

  useEffect(() => {
    const viewport = viewportRef.current;
    if (!viewport) {
      return undefined;
    }

    const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    renderer.setClearColor(0x08111d, 1);
    renderer.domElement.className = "pointcloud-webgl-canvas";
    viewport.appendChild(renderer.domElement);

    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x08111d);

    const camera = new THREE.PerspectiveCamera(52, 1, 0.05, 4000);
    camera.position.set(6, 4, 6);

    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.07;
    controls.rotateSpeed = 0.55;
    controls.zoomSpeed = 0.9;
    controls.panSpeed = 0.75;
    controls.zoomToCursor = true;
    controls.minDistance = 0.25;
    controls.maxDistance = 800;
    controls.mouseButtons = {
      LEFT: THREE.MOUSE.ROTATE,
      MIDDLE: THREE.MOUSE.PAN,
      RIGHT: THREE.MOUSE.PAN,
    };
    controls.target.set(0, 0.6, 0);

    const ambient = new THREE.HemisphereLight(0xdbeafe, 0x1e293b, 1.05);
    const directional = new THREE.DirectionalLight(0xffffff, 0.45);
    directional.position.set(6, 10, 8);
    const grid = new THREE.GridHelper(24, 48, 0x3b82f6, 0x1e293b);
    grid.position.y = -0.02;
    const obstacleGroup = new THREE.Group();
    scene.add(ambient, directional, grid, obstacleGroup);

    const robotMarker = createRobotMarker();
    const selectedGoalMarker = createGoalMarker(SELECTED_GOAL_COLOR);
    const activeGoalMarker = createGoalMarker(ACTIVE_GOAL_COLOR);
    scene.add(robotMarker, selectedGoalMarker, activeGoalMarker);

    const context: SceneContext = {
      renderer,
      scene,
      camera,
      controls,
      grid,
      obstacleGroup,
      savedPoints: null,
      livePoints: null,
      robotMarker,
      selectedGoalMarker,
      activeGoalMarker,
      savedCount: 0,
      liveCount: 0,
      activeBounds: null,
      preset: "iso",
      savedAssetKey: null,
      sceneOrigin: null,
      hasAutoFramed: false,
      animationFrame: null,
    };
    sceneRef.current = context;

    const resize = () => {
      const width = Math.max(1, viewport.clientWidth);
      const height = Math.max(1, viewport.clientHeight);
      renderer.setSize(width, height, false);
      camera.aspect = width / height;
      camera.updateProjectionMatrix();
    };
    resize();

    const resizeObserver = new ResizeObserver(resize);
    resizeObserver.observe(viewport);

    const raycaster = new THREE.Raycaster();
    raycaster.params.Points.threshold = 0.2;

    const onDoubleClick = (event: MouseEvent) => {
      const current = sceneRef.current;
      const host = viewportRef.current;
      if (!current || !host) {
        return;
      }
      const rect = host.getBoundingClientRect();
      if (rect.width <= 0 || rect.height <= 0) {
        return;
      }
      const mouse = new THREE.Vector2(
        ((event.clientX - rect.left) / rect.width) * 2 - 1,
        -((event.clientY - rect.top) / rect.height) * 2 + 1,
      );
      let hitPoint = pickGroundPointFromScreen(current, mouse, rect.width, rect.height);
      if (!hitPoint) {
        raycaster.setFromCamera(mouse, current.camera);
        const targets = [current.savedPoints, current.livePoints].filter((item): item is THREE.Points => Boolean(item));
        const intersections = raycaster.intersectObjects(targets, false);
        const hit = intersections.find((item: THREE.Intersection<THREE.Object3D>) => Boolean(item.point));
        if (!hit?.point) {
          raycaster.setFromCamera(mouse, current.camera);
          const planeHit = pickGroundPlanePoint(raycaster);
          if (!planeHit) {
            return;
          }
          hitPoint = planeHit;
        } else {
          hitPoint = hit.point.clone();
          const surfaceY = groundSurfaceY(current, hitPoint.x, hitPoint.z);
          if (surfaceY !== null) {
            hitPoint.y = surfaceY;
          }
        }
      }
      const world = threeToRos(hitPoint, current.sceneOrigin);
      onSelectGoalRef.current({
        x: world.x,
        y: world.y,
        yaw: 0,
        frame_id: "map",
      });
    };
    renderer.domElement.addEventListener("dblclick", onDoubleClick);

    const animate = () => {
      const current = sceneRef.current;
      if (!current) {
        return;
      }
      current.controls.update();
      current.renderer.render(current.scene, current.camera);
      current.animationFrame = window.requestAnimationFrame(animate);
    };
    animate();

    return () => {
      renderer.domElement.removeEventListener("dblclick", onDoubleClick);
      resizeObserver.disconnect();
      const activeScene = sceneRef.current;
      if (activeScene?.animationFrame !== null && activeScene?.animationFrame !== undefined) {
        window.cancelAnimationFrame(activeScene.animationFrame);
      }
      disposePointCloud(activeScene?.savedPoints ?? null);
      disposePointCloud(activeScene?.livePoints ?? null);
      robotMarker.traverse(disposeObjectMaterial);
      selectedGoalMarker.traverse(disposeObjectMaterial);
      activeGoalMarker.traverse(disposeObjectMaterial);
      disposeObstacleGroup(obstacleGroup);
      controls.dispose();
      renderer.dispose();
      if (renderer.domElement.parentElement === viewport) {
        viewport.removeChild(renderer.domElement);
      }
      sceneRef.current = null;
    };
  }, []);

  useEffect(() => {
    showSavedMapRef.current = showSavedMap;
  }, [showSavedMap]);

  useEffect(() => {
    let cancelled = false;
    const current = sceneRef.current;
    if (!current) {
      return undefined;
    }

    if (!selectedMapId || !activeSavedPointcloudPath || !savedAssetKey) {
      disposePointCloud(current.savedPoints);
      current.savedPoints = null;
      current.savedCount = 0;
      current.activeBounds = null;
      current.savedAssetKey = null;
      current.hasAutoFramed = false;
      setRenderStats({ saved: 0, live: current.liveCount });
      setArtifactState(selectedMapId ? "当前地图没有可显示的 3D 点云资产" : "等待 3D 资产");
      setRenderError(null);
      return () => {
        cancelled = true;
      };
    }

    const loader = new PCDLoader();
    setArtifactState("正在加载 Three.js 点云...");
    setRenderError(null);
    loader.load(
      buildMapFileUrl(selectedMapId, activeSavedPointcloudPath),
      (points: THREE.Points) => {
        if (cancelled || !sceneRef.current) {
          disposePointCloud(points);
          return;
        }
        const next = sceneRef.current;
        disposePointCloud(next.savedPoints);
        next.savedPoints = normalizeLoadedPcd(points, SAVED_POINT_COLOR, next.sceneOrigin);
        next.savedPoints.visible = showSavedMapRef.current;
        next.scene.add(next.savedPoints);
        next.savedCount = getPointCount(next.savedPoints);
        next.activeBounds = new THREE.Box3().setFromObject(next.savedPoints);
        next.savedAssetKey = savedAssetKey;
        setRenderStats({ saved: next.savedCount, live: next.liveCount });
        setArtifactState(next.savedCount > 0 ? `Three.js 已加载 ${next.savedCount} 点` : "已加载点云但没有可显示点");
        if (!next.hasAutoFramed) {
          applyViewPreset(next, next.preset);
          next.hasAutoFramed = true;
        }
      },
      undefined,
      (error: unknown) => {
        if (cancelled) {
          return;
        }
        const message = error instanceof Error ? error.message : "加载历史点云失败";
        const next = sceneRef.current;
        if (next) {
          disposePointCloud(next.savedPoints);
          next.savedPoints = null;
          next.savedCount = 0;
          next.activeBounds = null;
          setRenderStats({ saved: 0, live: next.liveCount });
        }
        setArtifactState(message);
        setRenderError(message);
      },
    );

    return () => {
      cancelled = true;
    };
  }, [activeSavedPointcloudPath, savedAssetKey, sceneOriginVersion, selectedMapId]);

  useEffect(() => {
    const current = sceneRef.current;
    if (!current) {
      return;
    }
    disposePointCloud(current.livePoints);
    current.livePoints = null;
    current.liveCount = 0;

    if (pointcloud?.loaded && pointcloud.points.length > 0) {
      current.livePoints = createLivePointCloud(pointcloud.points, current.sceneOrigin);
      current.livePoints.visible = !hasSavedMap || showLiveOverlay;
      current.scene.add(current.livePoints);
      current.liveCount = pointcloud.points.length;
      if (!hasSavedMap) {
        const shouldFrame = !current.hasAutoFramed || current.activeBounds === null;
        current.activeBounds = new THREE.Box3().setFromObject(current.livePoints);
        if (shouldFrame) {
          applyViewPreset(current, current.preset);
          current.hasAutoFramed = true;
        }
      }
    }

    setRenderStats({ saved: current.savedCount, live: current.liveCount });
  }, [hasSavedMap, pointcloud, sceneOriginVersion, showLiveOverlay]);

  useEffect(() => {
    const current = sceneRef.current;
    if (!current) {
      return;
    }
    if (current.savedPoints) {
      current.savedPoints.visible = showSavedMap;
    }
  }, [showSavedMap]);

  useEffect(() => {
    const current = sceneRef.current;
    if (!current) {
      return;
    }
    if (current.livePoints) {
      current.livePoints.visible = !hasSavedMap || showLiveOverlay;
    }
  }, [hasSavedMap, showLiveOverlay]);

  useEffect(() => {
    const current = sceneRef.current;
    const hasPose = Boolean(pose?.available && pose.x !== null && pose.y !== null);
    if (current && hasPose && pose) {
      lastRobotPoseRef.current = {
        x: pose.x ?? 0,
        y: pose.y ?? 0,
        yaw: pose.yaw ?? 0,
      };
      ensureSceneOriginFromPose(
        current,
        pose,
        () => setSceneOriginVersion((value) => value + 1),
        setSceneOriginLabel,
      );
    }
    const markerPose = lastRobotPoseRef.current;
    updateMarker(
      current?.robotMarker ?? null,
      markerPositionFromRos(current, { x: markerPose.x, y: markerPose.y, z: 0 }, false),
      markerPose.yaw,
    );
  }, [pose?.available, pose?.source, pose?.stamp, pose?.stale, pose?.x, pose?.y, pose?.yaw, sceneOriginVersion]);

  useEffect(() => {
    const current = sceneRef.current;
    updateMarker(
      current?.selectedGoalMarker ?? null,
      selectedGoal ? markerPositionFromRos(current, { x: selectedGoal.x, y: selectedGoal.y, z: 0 }, true) : null,
      selectedGoal?.yaw ?? 0,
    );
  }, [sceneOriginVersion, selectedGoal]);

  useEffect(() => {
    const current = sceneRef.current;
    updateMarker(
      current?.activeGoalMarker ?? null,
      activeGoal ? markerPositionFromRos(current, { x: activeGoal.x, y: activeGoal.y, z: 0 }, true) : null,
      activeGoal?.yaw ?? 0,
    );
  }, [activeGoal, sceneOriginVersion]);

  useEffect(() => {
    const current = sceneRef.current;
    if (!current) {
      return;
    }
    disposeObstacleGroup(current.obstacleGroup);
    for (const obstacle of obstacles) {
      current.obstacleGroup.add(createObstacleMarker(obstacle, current.sceneOrigin));
    }
  }, [obstacles, sceneOriginVersion]);

  const setPresetView = (preset: ViewPreset) => {
    const current = sceneRef.current;
    if (!current) {
      return;
    }
    current.preset = preset;
    applyViewPreset(current, preset);
  };

  const recenterSceneOrigin = () => {
    const current = sceneRef.current;
    if (!current || !pose?.available || pose.x === null || pose.y === null) {
      return;
    }
    setSceneOriginFromPose(
      current,
      pose,
      () => setSceneOriginVersion((value) => value + 1),
      setSceneOriginLabel,
    );
    lastRobotPoseRef.current = { x: pose.x, y: pose.y, yaw: pose.yaw ?? 0 };
    updateMarker(current.robotMarker, markerPositionFromRos(current, { x: pose.x, y: pose.y, z: 0 }, false), pose.yaw ?? 0);
  };

  return (
    <div className="map-shell pointcloud-shell">
      <div ref={viewportRef} className="pointcloud-render-root" />
      <div className="scene-card scene-card-left scene-card-dark">
        <div className="scene-card-title">3D 点云主视图</div>
        <div className="scene-card-meta">{selectedMap?.map_id ?? "未选择地图"}</div>
        <div className="scene-card-meta">{activeSavedPointcloudPath ?? "使用默认 3D 资产"}</div>
        <div className="scene-card-meta">{artifactState}</div>
      </div>
      <div className="scene-toolbar scene-toolbar-top-right scene-toolbar-dark">
        <button type="button" className="hud-button hud-button-dark" onClick={() => setPresetView("iso")}>
          等轴
        </button>
        <button type="button" className="hud-button hud-button-dark" onClick={() => setPresetView("front")}>
          正视
        </button>
        <button type="button" className="hud-button hud-button-dark" onClick={() => setPresetView("top")}>
          俯视
        </button>
        <button
          type="button"
          className="hud-button hud-button-dark"
          onClick={() => setShowSavedMap((current) => !current)}
          disabled={!hasSavedMap}
        >
          {showSavedMap ? "隐藏全局地图" : "显示全局地图"}
        </button>
        <button
          type="button"
          className="hud-button hud-button-dark"
          onClick={() => setShowLiveOverlay((current) => !current)}
          disabled={livePointCount === 0 || !hasSavedMap}
        >
          {showLiveOverlay ? "隐藏当前雷达" : "显示当前雷达"}
        </button>
        <button
          type="button"
          className="hud-button hud-button-dark"
          onClick={recenterSceneOrigin}
          disabled={!pose?.available || pose.x === null || pose.y === null}
        >
          原点归位
        </button>
      </div>
      <div className="map-overlay pointcloud-overlay">
        <span className="pointcloud-status-source">{`3D ${sourceLabel}`}</span>
        <span>{compactPoseLabel}</span>
        <span>{compactPoseStampLabel}</span>
        <span>{sceneOriginLabel}</span>
        <span>{`saved ${renderStats.saved} / live ${renderStats.live}`}</span>
        <span>{renderError ?? "three.js"}</span>
      </div>
    </div>
  );
}

function createRobotMarker(): THREE.Group {
  const group = new THREE.Group();
  const base = new THREE.Mesh(
    new THREE.TorusGeometry(0.22, 0.035, 12, 48),
    new THREE.MeshStandardMaterial({
      color: ROBOT_OUTLINE_COLOR,
      emissive: ROBOT_OUTLINE_COLOR,
      emissiveIntensity: 0.35,
      roughness: 0.45,
    }),
  );
  base.rotation.x = Math.PI / 2;
  base.position.y = 0.012;
  const body = new THREE.Mesh(
    new THREE.CylinderGeometry(0.17, 0.17, 0.14, 24),
    new THREE.MeshStandardMaterial({
      color: ROBOT_BODY_COLOR,
      emissive: ROBOT_BODY_COLOR,
      emissiveIntensity: 0.25,
      metalness: 0.05,
      roughness: 0.35,
    }),
  );
  body.position.y = 0.082;
  const mast = new THREE.Mesh(
    new THREE.CylinderGeometry(0.025, 0.025, 0.45, 12),
    new THREE.MeshStandardMaterial({
      color: ROBOT_BODY_COLOR,
      emissive: ROBOT_BODY_COLOR,
      emissiveIntensity: 0.45,
    }),
  );
  mast.position.y = 0.3;
  const heading = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0.1, 0), 0.58, ROBOT_HEADING_COLOR.getHex(), 0.18, 0.1);
  group.add(base, body, mast, heading);
  group.visible = false;
  return group;
}

function createGoalMarker(color: THREE.Color): THREE.Group {
  const group = new THREE.Group();
  const ring = new THREE.Mesh(
    new THREE.TorusGeometry(0.18, 0.028, 12, 48),
    new THREE.MeshStandardMaterial({ color }),
  );
  ring.rotation.x = Math.PI / 2;
  ring.position.y = 0.03;
  const arrow = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0.05, 0), 0.42, color.getHex(), 0.14, 0.08);
  group.add(ring, arrow);
  group.visible = false;
  return group;
}

function createObstacleMarker(obstacle: VirtualObstacleZone, origin: SceneOrigin | null): THREE.Group {
  const group = new THREE.Group();
  const base = new THREE.Mesh(
    new THREE.CylinderGeometry(obstacle.radius, obstacle.radius, 0.05, 48),
    new THREE.MeshStandardMaterial({
      color: 0xef4444,
      transparent: true,
      opacity: 0.18,
    }),
  );
  base.position.y = 0.025;
  const ring = new THREE.Mesh(
    new THREE.TorusGeometry(obstacle.radius, 0.03, 12, 64),
    new THREE.MeshStandardMaterial({
      color: 0xdc2626,
      transparent: true,
      opacity: 0.78,
    }),
  );
  ring.rotation.x = Math.PI / 2;
  ring.position.y = 0.05;
  group.add(base, ring);
  group.position.copy(rosToThree({ x: obstacle.x, y: obstacle.y, z: 0 }, origin));
  return group;
}

function updateMarker(group: THREE.Group | null, position: THREE.Vector3 | null, yaw: number) {
  if (!group) {
    return;
  }
  if (!position) {
    group.visible = false;
    return;
  }
  group.visible = true;
  group.position.copy(position);
  group.rotation.set(0, -yaw, 0);
}

function ensureSceneOriginFromPose(
  context: SceneContext,
  pose: RobotPose,
  bumpSceneOriginVersion: () => void,
  setSceneOriginLabel: (label: string) => void,
) {
  if (context.sceneOrigin || pose.x === null || pose.y === null) {
    return;
  }
  setSceneOriginFromPose(context, pose, bumpSceneOriginVersion, setSceneOriginLabel);
}

function setSceneOriginFromPose(
  context: SceneContext,
  pose: RobotPose,
  bumpSceneOriginVersion: () => void,
  setSceneOriginLabel: (label: string) => void,
) {
  if (pose.x === null || pose.y === null) {
    return;
  }
  context.sceneOrigin = { x: pose.x, y: pose.y };
  context.hasAutoFramed = false;
  setSceneOriginLabel(`origin=${pose.x.toFixed(2)}, ${pose.y.toFixed(2)}`);
  bumpSceneOriginVersion();
}

function markerPositionFromRos(context: SceneContext | null, point: { x: number; y: number; z: number }, snapToSurface: boolean) {
  const position = rosToThree(point, context?.sceneOrigin ?? null);
  const surfaceY = snapToSurface && context ? groundSurfaceY(context, position.x, position.z) : null;
  if (surfaceY !== null) {
    position.y = surfaceY;
  }
  return position;
}

function groundSurfaceY(context: SceneContext, x: number, z: number): number | null {
  const localHeights: number[] = [];
  const searchRadiusSq = GROUND_SEARCH_RADIUS_M * GROUND_SEARCH_RADIUS_M;
  const candidates = [context.savedPoints, context.livePoints].filter((item): item is THREE.Points => Boolean(item));
  for (const points of candidates) {
    const attribute = points.geometry.getAttribute("position");
    if (!attribute || !(attribute instanceof THREE.BufferAttribute)) {
      continue;
    }
    for (let index = 0; index < attribute.count; index += 1) {
      const dx = attribute.getX(index) - x;
      const dz = attribute.getZ(index) - z;
      const distanceSq = dx * dx + dz * dz;
      if (distanceSq <= searchRadiusSq) {
        localHeights.push(attribute.getY(index));
      }
    }
  }
  return quantile(localHeights, GROUND_HEIGHT_QUANTILE);
}

function pickGroundPointFromScreen(context: SceneContext, mouse: THREE.Vector2, width: number, height: number): THREE.Vector3 | null {
  const candidates: Array<{ point: THREE.Vector3; distanceSq: number }> = [];
  const maxDistanceSq = GROUND_PICK_RADIUS_PX * GROUND_PICK_RADIUS_PX;
  const world = new THREE.Vector3();
  const projected = new THREE.Vector3();
  const pointClouds = [context.savedPoints, context.livePoints].filter((item): item is THREE.Points => Boolean(item && item.visible));

  for (const points of pointClouds) {
    const attribute = points.geometry.getAttribute("position");
    if (!attribute || !(attribute instanceof THREE.BufferAttribute)) {
      continue;
    }
    for (let index = 0; index < attribute.count; index += 1) {
      world.fromBufferAttribute(attribute, index);
      points.localToWorld(world);
      projected.copy(world).project(context.camera);
      if (projected.z < -1 || projected.z > 1) {
        continue;
      }
      const dx = ((projected.x - mouse.x) * width) / 2;
      const dy = ((projected.y - mouse.y) * height) / 2;
      const distanceSq = dx * dx + dy * dy;
      if (distanceSq <= maxDistanceSq) {
        candidates.push({ point: world.clone(), distanceSq });
      }
    }
  }

  if (candidates.length === 0) {
    return null;
  }

  const groundY = quantile(
    candidates.map((candidate) => candidate.point.y),
    GROUND_HEIGHT_QUANTILE,
  );
  if (groundY === null) {
    return null;
  }

  const groundBand = Math.max(0.08, GROUND_SEARCH_RADIUS_M * 0.18);
  const groundCandidates = candidates
    .filter((candidate) => candidate.point.y <= groundY + groundBand)
    .sort((left, right) => left.distanceSq - right.distanceSq);
  const picked = (groundCandidates[0] ?? candidates.sort((left, right) => left.distanceSq - right.distanceSq)[0]).point.clone();
  const surfaceY = groundSurfaceY(context, picked.x, picked.z);
  if (surfaceY !== null) {
    picked.y = surfaceY;
  }
  return picked;
}

function pickGroundPlanePoint(raycaster: THREE.Raycaster): THREE.Vector3 | null {
  const plane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
  const hit = new THREE.Vector3();
  return raycaster.ray.intersectPlane(plane, hit) ? hit : null;
}

function quantile(values: number[], q: number): number | null {
  if (values.length === 0) {
    return null;
  }
  const sorted = [...values].sort((left, right) => left - right);
  const index = Math.min(sorted.length - 1, Math.max(0, Math.floor((sorted.length - 1) * q)));
  return sorted[index];
}

function normalizeLoadedPcd(points: THREE.Points, fallbackColor: THREE.Color, origin: SceneOrigin | null): THREE.Points {
  const geometry = points.geometry;
  const hasColors = geometry.getAttribute("color") !== undefined;
  const material = new THREE.PointsMaterial({
    size: 0.05,
    sizeAttenuation: true,
    color: fallbackColor,
    transparent: true,
    opacity: 0.84,
    vertexColors: hasColors,
  });
  disposeObjectMaterial(points);
  points.material = material;
  points.rotateX(0);
  remapGeometryToThreeGroundPlane(geometry, origin);
  alignGeometryGroundToZero(geometry);
  geometry.computeBoundingSphere();
  return points;
}

function createLivePointCloud(points: number[][], origin: SceneOrigin | null): THREE.Points {
  const positions = new Float32Array(points.length * 3);
  for (let index = 0; index < points.length; index += 1) {
    const [x, y, z] = points[index];
    const remapped = rosToThree({ x, y, z }, origin);
    positions[index * 3] = remapped.x;
    positions[index * 3 + 1] = remapped.y;
    positions[index * 3 + 2] = remapped.z;
  }
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
  alignGeometryGroundToZero(geometry);
  geometry.computeBoundingSphere();
  const material = new THREE.PointsMaterial({
    size: 0.04,
    sizeAttenuation: true,
    color: LIVE_POINT_COLOR,
    transparent: true,
    opacity: 0.86,
  });
  return new THREE.Points(geometry, material);
}

function remapGeometryToThreeGroundPlane(geometry: THREE.BufferGeometry, origin: SceneOrigin | null) {
  const attribute = geometry.getAttribute("position");
  if (!attribute || !(attribute instanceof THREE.BufferAttribute)) {
    return;
  }
  const activeOrigin = origin ?? ZERO_SCENE_ORIGIN;
  for (let index = 0; index < attribute.count; index += 1) {
    const x = attribute.getX(index);
    const y = attribute.getY(index);
    const z = attribute.getZ(index);
    attribute.setXYZ(index, x - activeOrigin.x, z, y - activeOrigin.y);
  }
  attribute.needsUpdate = true;
}

function alignGeometryGroundToZero(geometry: THREE.BufferGeometry) {
  const attribute = geometry.getAttribute("position");
  if (!attribute || !(attribute instanceof THREE.BufferAttribute)) {
    return;
  }
  const heights: number[] = [];
  for (let index = 0; index < attribute.count; index += 1) {
    heights.push(attribute.getY(index));
  }
  const groundY = quantile(heights, DISPLAY_GROUND_HEIGHT_QUANTILE);
  if (groundY === null || Math.abs(groundY) <= 1.0e-4) {
    return;
  }
  for (let index = 0; index < attribute.count; index += 1) {
    attribute.setY(index, attribute.getY(index) - groundY);
  }
  attribute.needsUpdate = true;
}

function applyViewPreset(context: SceneContext, preset: ViewPreset) {
  const bounds = context.activeBounds;
  const fallbackCenter = new THREE.Vector3(0, 0.4, 0);
  const center = bounds && !bounds.isEmpty() ? bounds.getCenter(new THREE.Vector3()) : fallbackCenter;
  const size = bounds && !bounds.isEmpty() ? bounds.getSize(new THREE.Vector3()) : new THREE.Vector3(4, 2, 4);
  const radius = Math.max(size.length() * 0.45, 2.6);
  const distance = radius / Math.tan(THREE.MathUtils.degToRad(context.camera.fov * 0.5)) * 0.72;

  let direction = new THREE.Vector3(1, 0.75, 1);
  if (preset === "front") {
    direction = new THREE.Vector3(0, 0.32, 1.4);
  } else if (preset === "top") {
    direction = new THREE.Vector3(0.001, 1.9, 0.001);
  }

  direction.normalize();
  context.camera.position.copy(center.clone().add(direction.multiplyScalar(distance)));
  context.controls.target.copy(center);
  context.controls.update();
}

function getPointCount(points: THREE.Points | null): number {
  if (!points) {
    return 0;
  }
  const attribute = points.geometry.getAttribute("position");
  return attribute?.count ?? 0;
}

function disposePointCloud(points: THREE.Points | null) {
  if (!points) {
    return;
  }
  points.removeFromParent();
  points.geometry.dispose();
  disposeObjectMaterial(points);
}

function disposeObstacleGroup(group: THREE.Group) {
  const children = [...group.children];
  for (const child of children) {
    group.remove(child);
    child.traverse((object) => {
      const mesh = object as THREE.Mesh;
      mesh.geometry?.dispose?.();
      disposeObjectMaterial(object);
    });
  }
}

function disposeObjectMaterial(object: THREE.Object3D) {
  const material = (object as { material?: THREE.Material | THREE.Material[] }).material;
  if (Array.isArray(material)) {
    material.forEach((item) => item.dispose());
  } else {
    material?.dispose();
  }
}

function rosToThree(point: { x: number; y: number; z: number }, origin: SceneOrigin | null = null) {
  const activeOrigin = origin ?? ZERO_SCENE_ORIGIN;
  return new THREE.Vector3(point.x - activeOrigin.x, point.z, point.y - activeOrigin.y);
}

function threeToRos(point: THREE.Vector3, origin: SceneOrigin | null = null) {
  const activeOrigin = origin ?? ZERO_SCENE_ORIGIN;
  return { x: point.x + activeOrigin.x, y: point.z + activeOrigin.y, z: point.y };
}
