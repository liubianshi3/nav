import { Suspense, lazy, useEffect, useMemo, useState } from "react";
import type { Dispatch, ReactNode, SetStateAction } from "react";

import {
  cancelNavigationGoal,
  deleteMapObstacle,
  deleteTaskRoute,
  fetchHealth,
  fetchMapObstacles,
  fetchMaps,
  fetchSnapshot,
  fetchStackStatus,
  fetchTaskRoute,
  fetchTaskRoutes,
  runTaskRoute,
  saveCurrentMap,
  projectPcdTo2d,
  saveMapObstacle,
  saveTaskRoute,
  sendInitialPose,
  sendNavigationGoal,
  startMappingStack,
  startNavigationStack,
  stopTaskRoute,
  stopStack,
} from "./api";
import {
  MapManagementSection,
  ModeControlSection,
  NavigationTaskSection,
  ObstacleManagerSection,
  RecentNoticeSection,
  SelectedGoalSection,
  TaskRouteManagerSection,
  TaskStateChip,
} from "./components/ControlSidebar";
import { MapCanvas } from "./components/MapCanvas";
import { MediaDock } from "./components/MediaDock";
import {
  BatterySection,
  RecoverySection,
  ConnectionStatusSection,
  HealthSection,
  NodeStatusSection,
  PoseSection,
  RuntimeInfoSection,
  SystemStatusSection,
} from "./components/StatusSidebar";
import { LightDebugPanel } from "./components/LightDebugPanel";
import { useBackendSocket } from "./hooks/useBackendSocket";
import type {
  BackendEvent,
  DashboardSnapshot,
  NavigationGoal,
  NavigationTaskState,
  SavedMapInfo,
  StackStatus,
  TaskRouteStatus,
  TaskRouteSummary,
  VirtualObstacleZone,
} from "./types";

const PointCloudCanvas3D = lazy(async () => ({
  default: (await import("./components/PointCloudCanvas3D")).PointCloudCanvas3D,
}));

type DrawerKey = "task" | "map" | "nav" | "obstacle" | "function" | null;
type ViewMode = "auto" | "2d" | "3d";

function createEmptySnapshot(): DashboardSnapshot {
  return {
    map: {
      loaded: false,
      representation: "occupancy_grid_2d",
      frame_id: null,
      width: 0,
      height: 0,
      resolution: 0,
      origin: { x: 0, y: 0, yaw: 0 },
      stamp: null,
      data: [],
    },
    pointcloud: {
      loaded: false,
      representation: "pointcloud_map_3d",
      frame_id: null,
      stamp: null,
      source_topic: null,
      points: [],
      points_total: 0,
      points_sampled: 0,
      sample_stride: 1,
    },
    pose: {
      available: false,
      source: "localization_pose",
      frame_id: null,
      stamp: null,
      x: null,
      y: null,
      yaw: null,
      stale: true,
    },
    status: {
      system_ready: null,
      localization_ok: null,
      real_report: { raw: null, mode: null, state: null, ready: null, reason: null, fields: {} },
      lidar_status: { raw: null, mode: null, state: null, ready: null, reason: null, fields: {} },
      camera_status: { raw: null, mode: null, state: null, ready: null, reason: null, fields: {} },
      localization_status: { raw: null, mode: null, state: null, ready: null, reason: null, fields: {} },
      map_manager_status: { raw: null, mode: null, state: null, ready: null, reason: null, fields: {} },
      task_manager_status: { raw: null, mode: null, state: null, ready: null, reason: null, fields: {} },
      sdk_status: { raw: null, mode: null, state: null, ready: null, reason: null, fields: {} },
      active_map: null,
      velocity_linear_x: null,
      velocity_angular_z: null,
      raw_state: null,
      ndt_score: null,
      ndt_healthy: null,
      planner_type: null,
      bt_filename: null,
    },
    navigation: {
      state: "idle",
      message: null,
      backend: "pose_topic_3d",
      action_server_ready: false,
      goal: null,
      feedback: {},
      updated_at: null,
    },
    camera: {
      available: false,
      topic: null,
      frame_id: null,
      stamp: null,
      width: null,
      height: null,
      encoding: null,
      format: null,
      data_url: null,
      stale: true,
    },
    battery: {
      available: false,
      percentage: null,
      voltage: null,
      charging: null,
      stamp: null,
    },
    recovery: {
      active: false,
      step: null,
      sequence: [],
      recovered: null,
      duration_sec: null,
      attempts: 0,
      raw: null,
    },
    health: {
      backend_ok: false,
      ros_connected: false,
      ros_thread_alive: false,
      websocket_clients: 0,
      action_server_ready: false,
      map_received: false,
      pose_received: false,
      camera_received: false,
      last_map_update: null,
      last_pose_update: null,
      last_camera_update: null,
      last_error: null,
    },
  };
}

function createRouteTemplate(routeId: string, goal: NavigationGoal | null = null): string {
  const waypointBlock = goal
    ? `\n  - id: wp_01\n    x: ${goal.x.toFixed(3)}\n    y: ${goal.y.toFixed(3)}\n    yaw: ${goal.yaw.toFixed(3)}\n    dwell_sec: 0.0\n    note: selected goal`
    : "";
  return `mission_name: ${routeId || "route_demo"}\nwaypoints:${waypointBlock}\n`;
}

function appendGoalToRouteYaml(currentYaml: string, routeId: string, goal: NavigationGoal): string {
  const trimmed = currentYaml.trimEnd();
  const matches = [...trimmed.matchAll(/- id:\s*wp_(\d+)/g)];
  const nextIndex = matches.length + 1;
  const snippet =
    `  - id: wp_${String(nextIndex).padStart(2, "0")}\n` +
    `    x: ${goal.x.toFixed(3)}\n` +
    `    y: ${goal.y.toFixed(3)}\n` +
    `    yaw: ${goal.yaw.toFixed(3)}\n` +
    `    dwell_sec: 0.0\n` +
    `    note: selected goal`;
  if (!trimmed) {
    return createRouteTemplate(routeId, goal);
  }
  if (!trimmed.includes("waypoints:")) {
    return `${trimmed}\nwaypoints:\n${snippet}\n`;
  }
  return `${trimmed}\n${snippet}\n`;
}

function mapsSignature(items: SavedMapInfo[]): string {
  return items
    .map((map) => {
      const artifactSignature = map.artifacts
        .map((artifact) =>
          [
            artifact.kind,
            artifact.path,
            artifact.frame_id ?? "",
            artifact.points_total ?? "",
            artifact.points_saved ?? "",
            artifact.stamp_sec ?? "",
            artifact.stamp_nanosec ?? "",
          ].join(":"),
        )
        .join("|");
      return [
        map.map_id,
        map.created_at ?? "",
        map.representation ?? "",
        map.pointcloud_topic_3d ?? "",
        String(map.has_pointcloud_3d),
        String(map.navigation_compatible),
        artifactSignature,
      ].join("#");
    })
    .join("\n");
}

function setMapsIfChanged(setMaps: Dispatch<SetStateAction<SavedMapInfo[]>>, nextMaps: SavedMapInfo[]) {
  setMaps((currentMaps) => (mapsSignature(currentMaps) === mapsSignature(nextMaps) ? currentMaps : nextMaps));
}

export default function App() {
  const [snapshot, setSnapshot] = useState<DashboardSnapshot>(createEmptySnapshot());
  const [selectedGoal, setSelectedGoal] = useState<NavigationGoal | null>(null);
  const [stack, setStack] = useState<StackStatus | null>(null);
  const [maps, setMaps] = useState<SavedMapInfo[]>([]);
  const [selectedMapId, setSelectedMapId] = useState("");
  const [saveMapId, setSaveMapId] = useState(() => {
    const now = new Date();
    const pad = (value: number) => String(value).padStart(2, "0");
    return `site_map_${now.getFullYear()}${pad(now.getMonth() + 1)}${pad(now.getDate())}_${pad(now.getHours())}${pad(now.getMinutes())}`;
  });
  const [stackBusy, setStackBusy] = useState(false);
  const [backendConnected, setBackendConnected] = useState(false);
  const [lastError, setLastError] = useState<string | null>(null);
  const [lastSuccess, setLastSuccess] = useState<string | null>(null);
  const [activeDrawer, setActiveDrawer] = useState<DrawerKey>(null);
  const [viewMode, setViewMode] = useState<ViewMode>("auto");
  const [showMediaDock, setShowMediaDock] = useState(true);
  const [selectedPointcloudPath, setSelectedPointcloudPath] = useState<string | null>(null);
  const [routes, setRoutes] = useState<TaskRouteSummary[]>([]);
  const [routeStatus, setRouteStatus] = useState<TaskRouteStatus | null>(null);
  const [selectedRouteId, setSelectedRouteId] = useState("");
  const [routeDraftId, setRouteDraftId] = useState("office_loop");
  const [routeYaml, setRouteYaml] = useState("mission_name: office_loop\nwaypoints:\n");
  const [routeBusy, setRouteBusy] = useState(false);
  const [obstacles, setObstacles] = useState<VirtualObstacleZone[]>([]);
  const [obstacleBusy, setObstacleBusy] = useState(false);
  const [obstacleLabel, setObstacleLabel] = useState("");
  const [obstacleRadius, setObstacleRadius] = useState("0.60");

  useEffect(() => {
    let cancelled = false;
    fetchSnapshot()
      .then((data) => {
        if (cancelled) {
          return;
        }
        setSnapshot(data);
        setBackendConnected(true);
      })
      .catch((error) => {
        if (cancelled) {
          return;
        }
        setLastError(error instanceof Error ? error.message : "无法获取初始快照");
        setBackendConnected(false);
      });

    fetchHealth()
      .then((health) => {
        if (cancelled) {
          return;
        }
        setSnapshot((current) => ({ ...current, health }));
      })
      .catch(() => undefined);

    fetchStackStatus()
      .then((status) => {
        if (cancelled) {
          return;
        }
        setStack(status);
        setMapsIfChanged(setMaps, status.maps);
        if (!selectedMapId && status.selected_map_id) {
          setSelectedMapId(status.selected_map_id);
        }
      })
      .catch(() => undefined);

    fetchMaps()
      .then((items) => {
        if (cancelled) {
          return;
        }
        setMapsIfChanged(setMaps, items);
      })
      .catch(() => undefined);

    return () => {
      cancelled = true;
    };
  }, [selectedMapId]);

  useEffect(() => {
    if (!selectedMapId) {
      return;
    }
    if (maps.some((map) => map.map_id === selectedMapId)) {
      return;
    }
    setSelectedMapId(maps[0]?.map_id ?? "");
  }, [maps, selectedMapId]);

  useEffect(() => {
    if (stackBusy) {
      return undefined;
    }
    const interval = window.setInterval(() => {
      fetchStackStatus()
        .then((status) => {
          setStack(status);
          setMapsIfChanged(setMaps, status.maps);
          if (!selectedMapId && status.selected_map_id) {
            setSelectedMapId(status.selected_map_id);
          }
        })
        .catch(() => undefined);
    }, 2500);
    return () => window.clearInterval(interval);
  }, [selectedMapId, stackBusy]);

  useEffect(() => {
    const interval = window.setInterval(() => {
      fetchHealth()
        .then((health) => {
          setSnapshot((current) => ({ ...current, health }));
        })
        .catch(() => undefined);
    }, 2500);
    return () => window.clearInterval(interval);
  }, []);

  const refreshRoutes = async () => {
    const payload = await fetchTaskRoutes();
    setRoutes(payload.routes);
    setRouteStatus(payload.status);
    if (!selectedRouteId && payload.status.route_id) {
      setSelectedRouteId(payload.status.route_id);
    }
    return payload;
  };

  useEffect(() => {
    let cancelled = false;
    refreshRoutes()
      .catch(() => undefined)
      .finally(() => {
        if (cancelled) {
          return;
        }
      });
    return () => {
      cancelled = true;
    };
  }, []);

  useEffect(() => {
    if (routeBusy) {
      return undefined;
    }
    const interval = window.setInterval(() => {
      refreshRoutes().catch(() => undefined);
    }, 3000);
    return () => window.clearInterval(interval);
  }, [routeBusy, selectedRouteId]);

  useEffect(() => {
    let cancelled = false;
    if (!selectedRouteId) {
      return () => {
        cancelled = true;
      };
    }
    fetchTaskRoute(selectedRouteId)
      .then((detail) => {
        if (cancelled) {
          return;
        }
        setRouteDraftId(detail.route_id);
        setRouteYaml(detail.route_yaml);
      })
      .catch((error) => {
        if (cancelled) {
          return;
        }
        setLastError(error instanceof Error ? error.message : "加载路线失败");
      });
    return () => {
      cancelled = true;
    };
  }, [selectedRouteId]);

  const refreshObstacles = async (mapId: string) => {
    const payload = await fetchMapObstacles(mapId);
    setObstacles(payload.obstacles);
    return payload;
  };

  useEffect(() => {
    let cancelled = false;
    if (!selectedMapId) {
      setObstacles([]);
      return () => {
        cancelled = true;
      };
    }
    refreshObstacles(selectedMapId).catch((error) => {
      if (cancelled) {
        return;
      }
      setLastError(error instanceof Error ? error.message : "加载虚拟障碍物失败");
    });
    return () => {
      cancelled = true;
    };
  }, [selectedMapId]);

  const { connected: websocketConnected, lastError: websocketError } = useBackendSocket({
    onEvent: (event: BackendEvent<unknown>) => {
      setBackendConnected(true);
      if (event.type === "snapshot") {
        setSnapshot(event.payload as DashboardSnapshot);
        return;
      }
      if (event.type === "map") {
        setSnapshot((current) => ({ ...current, map: event.payload as DashboardSnapshot["map"] }));
        return;
      }
      if (event.type === "pointcloud") {
        setSnapshot((current) => ({ ...current, pointcloud: event.payload as DashboardSnapshot["pointcloud"] }));
        return;
      }
      if (event.type === "pose") {
        setSnapshot((current) => ({ ...current, pose: event.payload as DashboardSnapshot["pose"] }));
        return;
      }
      if (event.type === "status") {
        setSnapshot((current) => ({ ...current, status: event.payload as DashboardSnapshot["status"] }));
        return;
      }
      if (event.type === "navigation") {
        setSnapshot((current) => ({ ...current, navigation: event.payload as DashboardSnapshot["navigation"] }));
        return;
      }
      if (event.type === "camera") {
        setSnapshot((current) => ({ ...current, camera: event.payload as DashboardSnapshot["camera"] }));
        return;
      }
      if (event.type === "health") {
        setSnapshot((current) => ({ ...current, health: event.payload as DashboardSnapshot["health"] }));
      }
    },
    onError: (message) => {
      setLastError(message);
    },
  });

  useEffect(() => {
    if (websocketError) {
      setLastError(websocketError);
    }
  }, [websocketError]);

  const selectedMap = maps.find((map) => map.map_id === selectedMapId) ?? null;
  const has3DViewerData = snapshot.pointcloud.loaded || Boolean(selectedMap?.has_pointcloud_3d);
  const navigationUses3D = snapshot.navigation.backend === "pose_topic_3d" || has3DViewerData;
  const localizationReason =
    snapshot.status.localization_status.reason ?? snapshot.status.localization_status.state ?? null;
  const rosRuntimeHealthy = snapshot.health.ros_thread_alive && snapshot.health.ros_connected;
  const poseFresh = snapshot.pose.available && !snapshot.pose.stale && rosRuntimeHealthy;
  const localizationReady = rosRuntimeHealthy && snapshot.status.localization_ok === true;
  const systemReady = rosRuntimeHealthy && snapshot.status.system_ready === true;
  const navigationMessage =
    snapshot.navigation.state === "idle" && snapshot.navigation.goal === null
      ? "当前没有活动导航目标，控制器空闲"
      : snapshot.navigation.message;
  const displayNavigation = useMemo<NavigationTaskState>(
    () => ({ ...snapshot.navigation, message: navigationMessage }),
    [navigationMessage, snapshot.navigation],
  );
  const canSendGoal =
    stack?.mode === "navigation" &&
    localizationReady &&
    snapshot.health.action_server_ready &&
    poseFresh &&
    (navigationUses3D ? snapshot.pose.available : snapshot.map.loaded);
  const canSetInitialPose =
    rosRuntimeHealthy &&
    stack?.mode === "navigation" &&
    (navigationUses3D ? selectedMap !== null || has3DViewerData : snapshot.map.loaded) &&
    snapshot.navigation.state !== "navigating";
  const stackTransitioning = stackBusy || stack?.mode === "starting" || stack?.mode === "stopping";
  const startMappingReason =
    stackTransitioning
      ? "栈正在启动或停止，暂时不能切换模式"
      : stack?.mode === "mapping"
        ? "当前已经在建图模式"
        : "会停止当前栈并切到建图链";
  const startNavigationReason =
    stackTransitioning
      ? "栈正在启动或停止，暂时不能切换模式"
      : !selectedMapId
        ? "请先选择一张导航地图"
        : stack?.mode === "navigation"
          ? "当前已经在导航模式"
          : "会停止当前栈并加载所选地图进入导航";
  const saveMapReason =
    stackTransitioning
      ? "栈正在启动或停止，暂时不能保存地图"
      : stack?.mode !== "mapping"
        ? "只有建图模式下才能保存当前地图"
        : !saveMapId
          ? "请先填写地图名"
          : "当前建图结果可保存到地图目录";
  const projectPcdReason =
    stackTransitioning
      ? "栈正在启动或停止，暂时不能执行投影"
      : !selectedMapId
        ? "请先选择一个地图"
        : selectedMap?.has_pointcloud_3d !== true
          ? "所选地图不含3D点云，无法投影"
          : "从已保存的3D点云生成Nav2导航用2D地图";
  const setInitialPoseReason =
    !selectedGoal
      ? "请先在 2D 或 3D 视图里选一个点"
      : !rosRuntimeHealthy
        ? "后端 ROS 线程未运行，页面状态已过期"
      : stack?.mode !== "navigation"
        ? "当前不在导航模式，不能设置初始位姿"
        : navigationUses3D && !selectedMap && !has3DViewerData
          ? "3D 地图尚未加载，不能发送重定位初始位姿"
          : !navigationUses3D && !snapshot.map.loaded
            ? "2D 地图尚未加载，不能发送初始位姿"
            : snapshot.navigation.state === "navigating"
              ? "导航进行中，不能重设初始位姿"
              : null;
  const sendGoalReason =
    !selectedGoal
      ? "请先在 2D 或 3D 视图里选一个目标点"
      : !rosRuntimeHealthy
        ? "后端 ROS 线程未运行，页面状态已过期"
      : stack?.mode !== "navigation"
        ? "当前不在导航模式，不能发送导航目标"
        : !localizationReady
          ? `定位未就绪${localizationReason ? `: ${localizationReason}` : ""}`
          : !snapshot.health.action_server_ready
            ? "导航后端未就绪"
            : !poseFresh
              ? "机器人当前位姿未刷新或已过期"
              : navigationUses3D && !snapshot.pose.available
                ? "当前 3D 位姿不可用"
                : !navigationUses3D && !snapshot.map.loaded
                  ? "2D 地图尚未加载"
                  : null;

  const effectiveViewMode = useMemo<"2d" | "3d">(() => {
    if (viewMode === "2d" && snapshot.map.loaded) {
      return "2d";
    }
    if (viewMode === "3d" && has3DViewerData) {
      return "3d";
    }
    return has3DViewerData ? "3d" : "2d";
  }, [has3DViewerData, snapshot.map.loaded, viewMode]);

  const refreshStack = async () => {
    const [status, health] = await Promise.all([fetchStackStatus(), fetchHealth().catch(() => null)]);
    setStack(status);
    setMapsIfChanged(setMaps, status.maps);
    if (health) {
      setSnapshot((current) => ({ ...current, health }));
    }
    return status;
  };

  const runStackAction = async (action: () => Promise<StackStatus>, successMessage: string) => {
    setStackBusy(true);
    try {
      const status = await action();
      setStack(status);
      setMapsIfChanged(setMaps, status.maps);
      setLastSuccess(successMessage);
      setLastError(null);
      await refreshStack();
    } catch (error) {
      setLastSuccess(null);
      setLastError(error instanceof Error ? error.message : "栈控制失败");
    } finally {
      setStackBusy(false);
    }
  };

  const handleStartMapping = () => {
    if (!window.confirm("启动建图模式会停止当前栈。确认继续？")) {
      return;
    }
    void runStackAction(startMappingStack, "建图模式已启动");
  };

  const handleStartNavigation = () => {
    if (!selectedMapId) {
      setLastError("请先选择地图");
      return;
    }
    if (!window.confirm(`启动导航模式会停止当前栈并加载地图 ${selectedMapId}。确认继续？`)) {
      return;
    }
    void runStackAction(() => startNavigationStack(selectedMapId), "导航模式已启动");
  };

  const handleStopStack = () => {
    if (!window.confirm("确认停止当前栈？")) {
      return;
    }
    void runStackAction(stopStack, "当前栈已停止");
  };

  const handleSaveMap = async () => {
    setStackBusy(true);
    try {
      const result = await saveCurrentMap(saveMapId);
      setMapsIfChanged(setMaps, result.maps);
      setSelectedMapId(result.map.map_id);
      setLastSuccess(`地图已保存：${result.map.map_id}`);
      setLastError(null);
      await refreshStack();
    } catch (error) {
      setLastSuccess(null);
      setLastError(error instanceof Error ? error.message : "保存地图失败");
    } finally {
      setStackBusy(false);
    }
  };

  const handleProjectPcd = async () => {
    if (!selectedMapId) {
      setLastError("请先选择一个地图");
      return;
    }
    setStackBusy(true);
    try {
      const result = await projectPcdTo2d(selectedMapId);
      setLastSuccess(`PCD→2D 投影完成: ${result.map_yaml} (导航${result.navigation_ready ? "可用" : "未就绪"})`);
      setLastError(null);
      await refreshStack();
    } catch (error) {
      setLastSuccess(null);
      setLastError(error instanceof Error ? error.message : "PCD投影失败");
    } finally {
      setStackBusy(false);
    }
  };

  const handleSetInitialPose = async () => {
    if (!selectedGoal) {
      setLastError("请先在地图上点击选点");
      return;
    }
    try {
      const result = await sendInitialPose({ pose: selectedGoal, map_id: selectedMapId || null });
      setSelectedGoal(result.pose);
      setLastSuccess(result.message);
      setLastError(null);
    } catch (error) {
      setLastSuccess(null);
      setLastError(error instanceof Error ? error.message : "设置初始位姿失败");
    }
  };

  const handleSendGoal = async () => {
    if (!selectedGoal) {
      setLastError("请先在地图上点击目标点");
      return;
    }
    try {
      const navigation = await sendNavigationGoal(selectedGoal, selectedMapId || null);
      setSnapshot((current) => ({ ...current, navigation }));
      setLastSuccess("导航目标已发送");
      setLastError(null);
    } catch (error) {
      setLastSuccess(null);
      setLastError(error instanceof Error ? error.message : "发送导航目标失败");
    }
  };

  const handleCancelGoal = async () => {
    try {
      const navigation = await cancelNavigationGoal();
      setSnapshot((current) => ({ ...current, navigation }));
      setLastSuccess("已发送停止导航请求");
      setLastError(null);
    } catch (error) {
      setLastSuccess(null);
      setLastError(error instanceof Error ? error.message : "停止导航失败");
    }
  };

  const openDrawer = (drawer: Exclude<DrawerKey, null>) => {
    setActiveDrawer((current) => (current === drawer ? null : drawer));
  };

  const handleCreateRouteTemplate = () => {
    const nextRouteId = routeDraftId.trim() || selectedRouteId || "office_loop";
    setRouteDraftId(nextRouteId);
    setRouteYaml(createRouteTemplate(nextRouteId, selectedGoal));
  };

  const handleAppendSelectedGoal = () => {
    if (!selectedGoal) {
      setLastError("请先在 2D 或 3D 视图中选择一个点");
      return;
    }
    const nextRouteId = routeDraftId.trim() || selectedRouteId || "office_loop";
    setRouteDraftId(nextRouteId);
    setRouteYaml((current) => appendGoalToRouteYaml(current, nextRouteId, selectedGoal));
    setLastSuccess("已把当前选点追加到路线 YAML");
    setLastError(null);
  };

  const handleSaveRoute = async () => {
    const routeId = routeDraftId.trim();
    if (!routeId) {
      setLastError("请先填写路线 ID");
      return;
    }
    setRouteBusy(true);
    try {
      const detail = await saveTaskRoute(routeId, routeYaml, selectedMapId || null);
      setSelectedRouteId(detail.route_id);
      setRouteDraftId(detail.route_id);
      setRouteYaml(detail.route_yaml);
      await refreshRoutes();
      setLastSuccess(`路线已保存: ${detail.route_id}`);
      setLastError(null);
    } catch (error) {
      setLastSuccess(null);
      setLastError(error instanceof Error ? error.message : "保存路线失败");
    } finally {
      setRouteBusy(false);
    }
  };

  const handleDeleteRoute = async () => {
    if (!selectedRouteId) {
      setLastError("请先选择要删除的路线");
      return;
    }
    if (!window.confirm(`确认删除路线 ${selectedRouteId}？`)) {
      return;
    }
    setRouteBusy(true);
    try {
      await deleteTaskRoute(selectedRouteId);
      setSelectedRouteId("");
      setRouteDraftId("office_loop");
      setRouteYaml("mission_name: office_loop\nwaypoints:\n");
      await refreshRoutes();
      setLastSuccess(`路线已删除: ${selectedRouteId}`);
      setLastError(null);
    } catch (error) {
      setLastSuccess(null);
      setLastError(error instanceof Error ? error.message : "删除路线失败");
    } finally {
      setRouteBusy(false);
    }
  };

  const handleRunRoute = async () => {
    if (!selectedRouteId) {
      setLastError("请先选择路线");
      return;
    }
    if (!selectedMapId) {
      setLastError("执行路线前必须先选择地图");
      return;
    }
    if (!window.confirm(`确认在地图 ${selectedMapId} 上执行路线 ${selectedRouteId}？`)) {
      return;
    }
    setRouteBusy(true);
    try {
      const status = await runTaskRoute({
        route_id: selectedRouteId,
        map_id: selectedMapId,
        mission_name: selectedRouteId,
        dry_run: false,
        stop_on_failure: true,
        save_map_on_finish: false,
        save_map_on_failure: false,
      });
      setRouteStatus(status);
      await refreshRoutes();
      setLastSuccess(`路线任务已启动: ${selectedRouteId}`);
      setLastError(null);
    } catch (error) {
      setLastSuccess(null);
      setLastError(error instanceof Error ? error.message : "执行路线失败");
    } finally {
      setRouteBusy(false);
    }
  };

  const handleStopRoute = async () => {
    setRouteBusy(true);
    try {
      const status = await stopTaskRoute();
      setRouteStatus(status);
      await refreshRoutes();
      setLastSuccess("路线任务已停止");
      setLastError(null);
    } catch (error) {
      setLastSuccess(null);
      setLastError(error instanceof Error ? error.message : "停止路线失败");
    } finally {
      setRouteBusy(false);
    }
  };

  const handleSaveObstacle = async (x: number, y: number) => {
    if (!selectedMapId) {
      setLastError("请先选择地图，再创建虚拟障碍物");
      return;
    }
    const radius = Number(obstacleRadius);
    if (!Number.isFinite(radius) || radius <= 0.0) {
      setLastError("障碍物半径必须是大于 0 的数字");
      return;
    }
    setObstacleBusy(true);
    try {
      const listing = await saveMapObstacle(selectedMapId, {
        label: obstacleLabel.trim() || null,
        x,
        y,
        radius,
      });
      setObstacles(listing.obstacles);
      setLastSuccess(`已创建虚拟禁入区，共 ${listing.obstacles.length} 个`);
      setLastError(null);
      setObstacleLabel("");
    } catch (error) {
      setLastSuccess(null);
      setLastError(error instanceof Error ? error.message : "创建虚拟障碍物失败");
    } finally {
      setObstacleBusy(false);
    }
  };

  const handleAddObstacleFromGoal = () => {
    if (!selectedGoal) {
      setLastError("请先选点，再从目标点创建虚拟障碍物");
      return;
    }
    void handleSaveObstacle(selectedGoal.x, selectedGoal.y);
  };

  const handleAddObstacleFromPose = () => {
    if (!snapshot.pose.available || snapshot.pose.x === null || snapshot.pose.y === null) {
      setLastError("当前没有可用机器人位姿");
      return;
    }
    void handleSaveObstacle(snapshot.pose.x, snapshot.pose.y);
  };

  const handleDeleteObstacle = async (obstacleId: string) => {
    if (!selectedMapId) {
      return;
    }
    if (!window.confirm(`确认删除虚拟障碍物 ${obstacleId}？`)) {
      return;
    }
    setObstacleBusy(true);
    try {
      const listing = await deleteMapObstacle(selectedMapId, obstacleId);
      setObstacles(listing.obstacles);
      setLastSuccess(`已删除虚拟障碍物: ${obstacleId}`);
      setLastError(null);
    } catch (error) {
      setLastSuccess(null);
      setLastError(error instanceof Error ? error.message : "删除虚拟障碍物失败");
    } finally {
      setObstacleBusy(false);
    }
  };

  const sceneSubtitle =
    effectiveViewMode === "3d"
      ? "JT128 + DLIO 3D 主视图 / 双击点云选导航目标"
      : "2D 栅格兼容视图 / 单击地图选初始位姿或目标";

  const showMediaDockNow = showMediaDock;

  useEffect(() => {
    setSelectedPointcloudPath(null);
  }, [selectedMapId]);

  return (
    <div className="legacy-console-shell">
      <div className="legacy-console-page">
        <header className="legacy-console-header">
          <div>
            <div className="legacy-console-title">A2 Web Console</div>
            <div className="legacy-console-subtitle">{sceneSubtitle}</div>
          </div>
          <div className="legacy-console-header-actions">
            <button
              type="button"
              className={`view-switch ${viewMode === "auto" ? "view-switch-active" : ""}`}
              onClick={() => setViewMode("auto")}
            >
              自动视图
            </button>
            <button
              type="button"
              className={`view-switch ${effectiveViewMode === "2d" && viewMode === "2d" ? "view-switch-active" : ""}`}
              onClick={() => setViewMode("2d")}
              disabled={!snapshot.map.loaded}
            >
              2D 地图
            </button>
            <button
              type="button"
              className={`view-switch ${effectiveViewMode === "3d" && viewMode === "3d" ? "view-switch-active" : ""}`}
              onClick={() => setViewMode("3d")}
              disabled={!has3DViewerData}
            >
              3D 点云
            </button>
            <span className={`indicator ${systemReady ? "indicator-ok" : "indicator-warn"}`}>
              ready={String(systemReady)}
            </span>
            <span className={`indicator ${localizationReady ? "indicator-ok" : "indicator-warn"}`}>
              localization={String(localizationReady)}
            </span>
            <TaskStateChip state={stack?.mode ?? "stopped"} />
          </div>
        </header>

        <div className="legacy-console-stage">
          <button type="button" className="legacy-function-button" onClick={() => openDrawer("function")}>
            功能菜单
          </button>

          <div className="scene-rail scene-rail-left">
            <RailButton label="任务选择" active={activeDrawer === "task"} onClick={() => openDrawer("task")} tone="blue" />
            <RailButton label="地图选择" active={activeDrawer === "map"} onClick={() => openDrawer("map")} tone="red" />
          </div>
          <div className="scene-rail scene-rail-right">
            <RailButton label="导航选择" active={activeDrawer === "nav"} onClick={() => openDrawer("nav")} tone="green" />
            <RailButton
              label="障碍物选择"
              active={activeDrawer === "obstacle"}
              onClick={() => openDrawer("obstacle")}
              tone="green"
            />
          </div>

          <div className="scene-hud scene-hud-left">
            <StatusSummaryCard
              title="机器人状态"
              rows={[
                ["后端", backendConnected ? "online" : "offline"],
                ["WebSocket", websocketConnected ? "connected" : "disconnected"],
                ["栈模式", stack?.mode ?? "stopped"],
                ["地图", stack?.selected_map_id ?? snapshot.status.active_map ?? "暂无"],
              ]}
            />
          </div>
          <div className="scene-hud scene-hud-right">
            <StatusSummaryCard
              title="定位与执行"
              rows={[
                ["pose", snapshot.pose.available ? `${snapshot.pose.x?.toFixed(2)}, ${snapshot.pose.y?.toFixed(2)}` : "暂无"],
                ["frame", snapshot.pose.frame_id ?? "unknown"],
                ["3d asset", selectedPointcloudPath ?? "默认地图点云"],
                ["goal", snapshot.navigation.goal ? `${snapshot.navigation.goal.x.toFixed(2)}, ${snapshot.navigation.goal.y.toFixed(2)}` : "暂无"],
                ["task", snapshot.navigation.state],
              ]}
            />
          </div>

          <DrawerPanel side="left" open={activeDrawer === "task"}>
            <TaskRouteManagerSection
              routes={routes}
              routeStatus={routeStatus}
              selectedRouteId={selectedRouteId}
              routeDraftId={routeDraftId}
              routeYaml={routeYaml}
              selectedMapId={selectedMapId}
              routeBusy={routeBusy}
              selectedGoal={selectedGoal}
              onSelectedRouteChange={setSelectedRouteId}
              onRouteDraftIdChange={setRouteDraftId}
              onRouteYamlChange={setRouteYaml}
              onRefreshRoutes={() => {
                void refreshRoutes().catch((error) => {
                  setLastError(error instanceof Error ? error.message : "刷新路线失败");
                });
              }}
              onCreateRouteTemplate={handleCreateRouteTemplate}
              onAppendSelectedGoal={handleAppendSelectedGoal}
              onSaveRoute={handleSaveRoute}
              onDeleteRoute={handleDeleteRoute}
              onRunRoute={handleRunRoute}
              onStopRoute={handleStopRoute}
            />
            <NavigationTaskSection navigation={displayNavigation} />
            <RecentNoticeSection stack={stack} lastError={lastError} lastSuccess={lastSuccess} />
          </DrawerPanel>

          <DrawerPanel side="left" open={activeDrawer === "map"}>
            <ModeControlSection
              stack={stack}
              stackBusy={stackTransitioning}
              startMappingReason={startMappingReason}
              onStartMapping={handleStartMapping}
              onStopStack={handleStopStack}
            />
            <MapManagementSection
              stack={stack}
              maps={maps}
              selectedMapId={selectedMapId}
              saveMapId={saveMapId}
              stackBusy={stackTransitioning}
              startNavigationReason={startNavigationReason}
              saveMapReason={saveMapReason}
              projectPcdReason={projectPcdReason}
              onSelectedMapChange={setSelectedMapId}
              onSaveMapIdChange={setSaveMapId}
              onStartNavigation={handleStartNavigation}
              onSaveMap={handleSaveMap}
              onProjectPcd={handleProjectPcd}
            />
          </DrawerPanel>

          <DrawerPanel side="right" open={activeDrawer === "nav"}>
            <SelectedGoalSection
              selectedGoal={selectedGoal}
              canSendGoal={canSendGoal}
              canSetInitialPose={canSetInitialPose}
              sendGoalReason={sendGoalReason}
              setInitialPoseReason={setInitialPoseReason}
              onSetInitialPose={handleSetInitialPose}
              onSendGoal={handleSendGoal}
              onCancelGoal={handleCancelGoal}
            />
            <NavigationTaskSection navigation={snapshot.navigation} />
          </DrawerPanel>

          <DrawerPanel side="right" open={activeDrawer === "obstacle"}>
            <ObstacleManagerSection
              selectedMapId={selectedMapId}
              obstacles={obstacles}
              obstacleBusy={obstacleBusy}
              obstacleLabel={obstacleLabel}
              obstacleRadius={obstacleRadius}
              selectedGoal={selectedGoal}
              pose={snapshot.pose.available ? snapshot.pose : null}
              onObstacleLabelChange={setObstacleLabel}
              onObstacleRadiusChange={setObstacleRadius}
              onAddObstacleFromGoal={handleAddObstacleFromGoal}
              onAddObstacleFromPose={handleAddObstacleFromPose}
              onDeleteObstacle={handleDeleteObstacle}
            />
            <NodeStatusSection stack={stack} />
            <BatterySection battery={snapshot.battery} />
            <RecoverySection recovery={snapshot.recovery} />
            <RuntimeInfoSection status={snapshot.status} />
            <HealthSection health={snapshot.health} />
          </DrawerPanel>

          <FunctionMenuPanel open={activeDrawer === "function"}>
            <section className="panel panel-compact">
              <h2>可视化控制</h2>
              <div className="function-button-grid">
                <button type="button" className="secondary-button" onClick={() => setViewMode("2d")} disabled={!snapshot.map.loaded}>
                  切到 2D 视图
                </button>
                <button type="button" className="secondary-button" onClick={() => setViewMode("3d")} disabled={!has3DViewerData}>
                  切到 3D 视图
                </button>
                <button type="button" className="secondary-button" onClick={() => setShowMediaDock((current) => !current)}>
                  {showMediaDockNow ? "隐藏媒体区" : "显示媒体区"}
                </button>
                <button
                  type="button"
                  className="secondary-button"
                  disabled={!selectedPointcloudPath}
                  onClick={() => setSelectedPointcloudPath(null)}
                >
                  回到默认地图点云
                </button>
                <button type="button" className="danger-button" onClick={handleStopStack} disabled={stackTransitioning || stack?.mode === "stopped"}>
                  关闭建图导航节点
                </button>
              </div>
            </section>
            <LightDebugPanel />
            <ConnectionStatusSection
              backendConnected={backendConnected}
              websocketConnected={websocketConnected}
              health={snapshot.health}
            />
            <SystemStatusSection status={snapshot.status} pose={snapshot.pose} stack={stack} />
            <PoseSection pose={snapshot.pose} />
          </FunctionMenuPanel>

          <div className={`scene-main-grid ${showMediaDockNow ? "scene-main-grid-with-media" : ""}`}>
            <div className="scene-main-view">
              {effectiveViewMode === "3d" ? (
                <Suspense fallback={<ThreeViewLoadingCard />}>
                  <PointCloudCanvas3D
                    pointcloud={snapshot.pointcloud.loaded ? snapshot.pointcloud : null}
                    selectedMap={selectedMap}
                    selectedPointcloudPath={selectedPointcloudPath}
                    pose={snapshot.pose.available ? snapshot.pose : null}
                    obstacles={obstacles}
                    selectedGoal={selectedGoal}
                    activeGoal={snapshot.navigation.goal}
                    onSelectGoal={setSelectedGoal}
                  />
                </Suspense>
              ) : (
                <MapCanvas
                  map={snapshot.map.loaded ? snapshot.map : null}
                  pose={snapshot.pose.available ? snapshot.pose : null}
                  obstacles={obstacles}
                  selectedGoal={selectedGoal}
                  activeGoal={snapshot.navigation.goal}
                  disabled={!canSendGoal}
                  onSelectGoal={setSelectedGoal}
                />
              )}
            </div>

            {showMediaDockNow ? (
              <div className="scene-media-dock">
                <MediaDock
                  camera={snapshot.camera}
                  selectedMap={selectedMap}
                  selectedPointcloudPath={selectedPointcloudPath}
                  onSelectPointcloudPath={setSelectedPointcloudPath}
                />
              </div>
            ) : null}
          </div>

          <div className="scene-bottom-strip">
            <span>{snapshot.status.lidar_status.reason ? `lidar: ${snapshot.status.lidar_status.reason}` : "lidar 状态待更新"}</span>
            <span>{snapshot.status.localization_status.reason ? `localization: ${snapshot.status.localization_status.reason}` : "localization 状态待更新"}</span>
            <span>{navigationMessage ?? "等待导航任务"}</span>
            <span>{stack?.log_file ? `log: ${stack.log_file}` : "尚未生成运行日志"}</span>
          </div>
        </div>
      </div>
    </div>
  );
}

function RailButton({
  label,
  active,
  onClick,
  tone,
}: {
  label: string;
  active: boolean;
  onClick: () => void;
  tone: "red" | "blue" | "green";
}) {
  return (
    <button
      type="button"
      className={`scene-rail-button scene-rail-button-${tone} ${active ? "scene-rail-button-active" : ""}`}
      onClick={onClick}
    >
      {label}
    </button>
  );
}

function ThreeViewLoadingCard() {
  return (
    <div className="map-shell pointcloud-shell pointcloud-loading-shell">
      <div className="pointcloud-loading-card">
        <div className="scene-card-title">正在加载 3D 渲染器</div>
        <div className="scene-card-meta">Three.js / 历史点云主视图准备中...</div>
      </div>
    </div>
  );
}

function DrawerPanel({ side, open, children }: { side: "left" | "right"; open: boolean; children: ReactNode }) {
  return <div className={`legacy-drawer legacy-drawer-${side} ${open ? "legacy-drawer-open" : ""}`}>{children}</div>;
}

function FunctionMenuPanel({ open, children }: { open: boolean; children: ReactNode }) {
  return <div className={`function-menu-panel ${open ? "function-menu-panel-open" : ""}`}>{children}</div>;
}

function StatusSummaryCard({ title, rows }: { title: string; rows: Array<[string, string]> }) {
  return (
    <section className="scene-summary-card">
      <div className="scene-summary-title">{title}</div>
      {rows.map(([label, value]) => (
        <div key={label} className="scene-summary-row">
          <span>{label}</span>
          <span>{value}</span>
        </div>
      ))}
    </section>
  );
}
