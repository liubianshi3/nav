import { useEffect, useState } from "react";

import {
  cancelNavigationGoal,
  fetchHealth,
  fetchMaps,
  fetchSnapshot,
  fetchStackStatus,
  saveCurrentMap,
  sendInitialPose,
  sendNavigationGoal,
  startMappingStack,
  startNavigationStack,
  stopStack,
} from "./api";
import { CameraPanel } from "./components/CameraPanel";
import { ControlSidebar } from "./components/ControlSidebar";
import { MapCanvas } from "./components/MapCanvas";
import { PointCloudCanvas3D } from "./components/PointCloudCanvas3D";
import { StatusSidebar } from "./components/StatusSidebar";
import { useBackendSocket } from "./hooks/useBackendSocket";
import type { BackendEvent, DashboardSnapshot, NavigationGoal, SavedMapInfo, StackStatus } from "./types";

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
      localization_status: { raw: null, mode: null, state: null, ready: null, reason: null, fields: {} },
      map_manager_status: { raw: null, mode: null, state: null, ready: null, reason: null, fields: {} },
      task_manager_status: { raw: null, mode: null, state: null, ready: null, reason: null, fields: {} },
      sdk_status: { raw: null, mode: null, state: null, ready: null, reason: null, fields: {} },
      active_map: null,
      velocity_linear_x: null,
      velocity_angular_z: null,
      raw_state: null,
    },
    navigation: {
      state: "idle",
      message: null,
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
        setMaps(status.maps);
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
        setMaps(items);
      })
      .catch(() => undefined);

    return () => {
      cancelled = true;
    };
  }, [selectedMapId]);

  useEffect(() => {
    if (stackBusy) {
      return undefined;
    }
    const interval = window.setInterval(() => {
      fetchStackStatus()
        .then((status) => {
          setStack(status);
          setMaps(status.maps);
          if (!selectedMapId && status.selected_map_id) {
            setSelectedMapId(status.selected_map_id);
          }
        })
        .catch(() => undefined);
    }, 2500);
    return () => window.clearInterval(interval);
  }, [selectedMapId, stackBusy]);

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

  const poseAgeMs = snapshot.pose.stamp ? Date.now() - Date.parse(snapshot.pose.stamp) : Number.POSITIVE_INFINITY;
  const canSendGoal =
    stack?.mode === "navigation" &&
    snapshot.map.loaded &&
    snapshot.status.localization_ok === true &&
    snapshot.health.action_server_ready &&
    poseAgeMs < 10000;
  const canSetInitialPose = stack?.mode === "navigation" && snapshot.map.loaded && snapshot.navigation.state !== "navigating";
  const stackTransitioning = stackBusy || stack?.mode === "starting" || stack?.mode === "stopping";
  const selectedMap = maps.find((map) => map.map_id === selectedMapId) ?? null;
  const use3DViewer = snapshot.pointcloud.loaded || Boolean(selectedMap?.has_pointcloud_3d);

  const refreshStack = async () => {
    const [status, health] = await Promise.all([
      fetchStackStatus(),
      fetchHealth().catch(() => null),
    ]);
    setStack(status);
    setMaps(status.maps);
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
      setMaps(status.maps);
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
      setMaps(result.maps);
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

  const handleSetInitialPose = async () => {
    if (!selectedGoal) {
      setLastError("请先在地图上点击选点");
      return;
    }
    try {
      const result = await sendInitialPose(selectedGoal);
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
      const navigation = await sendNavigationGoal(selectedGoal);
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

  return (
    <div className="app-shell">
      <StatusSidebar
        status={snapshot.status}
        pose={snapshot.pose}
        health={snapshot.health}
        stack={stack}
        backendConnected={backendConnected}
        websocketConnected={websocketConnected}
      />

      <main className="main-panel">
        <header className="topbar">
          <div>
            <h1>A2 Web Console</h1>
            <p>建图模式 + 地图选择 + 点选导航</p>
          </div>
          <div className="topbar-indicators">
            <button className="mode-button" disabled={stackTransitioning || stack?.mode === "mapping"} onClick={handleStartMapping}>
              建图模式
            </button>
            <button className="mode-button" disabled={stackTransitioning || !selectedMapId || stack?.mode === "navigation"} onClick={handleStartNavigation}>
              导航模式
            </button>
            <span className={`indicator ${snapshot.status.system_ready ? "indicator-ok" : "indicator-warn"}`}>
              ready={String(snapshot.status.system_ready)}
            </span>
            <span className={`indicator ${snapshot.status.localization_ok ? "indicator-ok" : "indicator-warn"}`}>
              localization={String(snapshot.status.localization_ok)}
            </span>
          </div>
        </header>

        {use3DViewer ? (
          <PointCloudCanvas3D pointcloud={snapshot.pointcloud.loaded ? snapshot.pointcloud : null} selectedMap={selectedMap} />
        ) : (
          <MapCanvas
            map={snapshot.map.loaded ? snapshot.map : null}
            pose={snapshot.pose.available ? snapshot.pose : null}
            selectedGoal={selectedGoal}
            activeGoal={snapshot.navigation.goal}
            disabled={!canSendGoal}
            onSelectGoal={setSelectedGoal}
          />
        )}
        <CameraPanel camera={snapshot.camera} />
      </main>

      <ControlSidebar
        navigation={snapshot.navigation}
        stack={stack}
        maps={maps}
        selectedMapId={selectedMapId}
        saveMapId={saveMapId}
        selectedGoal={selectedGoal}
        canSendGoal={canSendGoal}
        canSetInitialPose={canSetInitialPose}
        stackBusy={stackTransitioning}
        onSelectedMapChange={setSelectedMapId}
        onSaveMapIdChange={setSaveMapId}
        onStartMapping={handleStartMapping}
        onStartNavigation={handleStartNavigation}
        onStopStack={handleStopStack}
        onSaveMap={handleSaveMap}
        onSetInitialPose={handleSetInitialPose}
        onSendGoal={handleSendGoal}
        onCancelGoal={handleCancelGoal}
        lastError={lastError}
        lastSuccess={lastSuccess}
      />
    </div>
  );
}
