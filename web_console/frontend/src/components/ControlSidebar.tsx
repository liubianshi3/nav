import type {
  NavigationGoal,
  NavigationTaskState,
  RobotPose,
  SavedMapInfo,
  StackStatus,
  TaskRouteStatus,
  TaskRouteSummary,
  VirtualObstacleZone,
} from "../types";
import { formatNumber, formatNullable } from "../utils/format";

export interface ControlSidebarProps {
  navigation: NavigationTaskState | null;
  stack: StackStatus | null;
  maps: SavedMapInfo[];
  selectedMapId: string;
  saveMapId: string;
  selectedGoal: NavigationGoal | null;
  canSendGoal: boolean;
  canSetInitialPose: boolean;
  startMappingReason: string | null;
  startNavigationReason: string | null;
  saveMapReason: string | null;
  projectPcdReason: string | null;
  sendGoalReason: string | null;
  setInitialPoseReason: string | null;
  stackBusy: boolean;
  onSelectedMapChange: (mapId: string) => void;
  onSaveMapIdChange: (mapId: string) => void;
  onStartMapping: () => void;
  onStartNavigation: () => void;
  onStopStack: () => void;
  onSaveMap: () => void;
  onProjectPcd: () => void;
  onSetInitialPose: () => void;
  onSendGoal: () => void;
  onCancelGoal: () => void;
  lastError: string | null;
  lastSuccess: string | null;
}

export function ControlSidebar(props: ControlSidebarProps) {
  return (
    <div className="console-section-stack">
      <ModeControlSection
        stack={props.stack}
        stackBusy={props.stackBusy}
        startMappingReason={props.startMappingReason}
        onStartMapping={props.onStartMapping}
        onStopStack={props.onStopStack}
      />
      <MapManagementSection
        stack={props.stack}
        maps={props.maps}
        selectedMapId={props.selectedMapId}
        saveMapId={props.saveMapId}
        stackBusy={props.stackBusy}
        startNavigationReason={props.startNavigationReason}
        saveMapReason={props.saveMapReason}
        projectPcdReason={props.projectPcdReason}
        onSelectedMapChange={props.onSelectedMapChange}
        onSaveMapIdChange={props.onSaveMapIdChange}
        onStartNavigation={props.onStartNavigation}
        onSaveMap={props.onSaveMap}
        onProjectPcd={props.onProjectPcd}
      />
      <NavigationTaskSection navigation={props.navigation} />
      <SelectedGoalSection
        selectedGoal={props.selectedGoal}
        canSendGoal={props.canSendGoal}
        canSetInitialPose={props.canSetInitialPose}
        sendGoalReason={props.sendGoalReason}
        setInitialPoseReason={props.setInitialPoseReason}
        onSetInitialPose={props.onSetInitialPose}
        onSendGoal={props.onSendGoal}
        onCancelGoal={props.onCancelGoal}
      />
      <RecentNoticeSection stack={props.stack} lastError={props.lastError} lastSuccess={props.lastSuccess} />
    </div>
  );
}

export function ModeControlSection({
  stack,
  stackBusy,
  startMappingReason,
  onStartMapping,
  onStopStack,
}: Pick<ControlSidebarProps, "stack" | "stackBusy" | "startMappingReason" | "onStartMapping" | "onStopStack">) {
  const isMapping = stack?.mode === "mapping";

  return (
    <section className="panel">
      <h2>模式控制</h2>
      <TaskStateChip state={stack?.mode ?? "stopped"} />
      <StatusMini label="pid" value={formatNullable(stack?.pid)} />
      <StatusMini label="地图" value={formatNullable(stack?.selected_map_id)} />
      <p className="panel-message">
        {isMapping ? "当前是建图模式。导航按钮仍会显示，但不应在这个模式下操作。" : formatNullable(startMappingReason, "可切换到建图模式")}
      </p>
      <div className="button-group">
        <button className="secondary-button" disabled={stackBusy || isMapping} onClick={onStartMapping}>
          启动建图模式
        </button>
        <button className="danger-button" disabled={stackBusy || stack?.mode === "stopped"} onClick={onStopStack}>
          停止当前栈
        </button>
      </div>
    </section>
  );
}

export function MapManagementSection({
  stack,
  maps,
  selectedMapId,
  saveMapId,
  stackBusy,
  startNavigationReason,
  saveMapReason,
  projectPcdReason,
  onSelectedMapChange,
  onSaveMapIdChange,
  onStartNavigation,
  onSaveMap,
  onProjectPcd,
}: Pick<
  ControlSidebarProps,
  | "stack"
  | "maps"
  | "selectedMapId"
  | "saveMapId"
  | "stackBusy"
  | "startNavigationReason"
  | "saveMapReason"
  | "projectPcdReason"
  | "onSelectedMapChange"
  | "onSaveMapIdChange"
  | "onStartNavigation"
  | "onSaveMap"
  | "onProjectPcd"
>) {
  const isMapping = stack?.mode === "mapping";
  const isNavigation = stack?.mode === "navigation";
  const selectedMap = maps.find((map) => map.map_id === selectedMapId) ?? null;
  const pointcloudArtifact =
    selectedMap?.artifacts.find((artifact) => artifact.kind === "native_pointcloud_map_3d") ??
    selectedMap?.artifacts.find((artifact) => artifact.kind === "pointcloud_snapshot_3d") ??
    null;

  return (
    <section className="panel">
      <h2>地图管理</h2>
      <label className="form-label" htmlFor="map-select">
        导航地图
      </label>
      <select
        id="map-select"
        className="select-input"
        value={selectedMapId}
        onChange={(event) => onSelectedMapChange(event.target.value)}
      >
        <option value="">选择地图</option>
        {maps.map((map) => (
          <option key={map.map_id} value={map.map_id}>
            {map.map_id}
          </option>
        ))}
      </select>
      <button
        className="primary-button full-width-button"
        disabled={stackBusy || !selectedMapId || isNavigation}
        onClick={onStartNavigation}
      >
        启动导航模式
      </button>
      <p className="panel-message">{formatNullable(startNavigationReason, "当前地图可用于导航启动")}</p>
      {selectedMap ? (
        <div className="map-asset-card">
          <StatusMini label="representation" value={formatNullable(selectedMap.representation)} />
          <StatusMini label="2D source" value={formatNullable(selectedMap.source_topic)} />
          <StatusMini
            label="3D asset"
            value={
              pointcloudArtifact?.kind === "native_pointcloud_map_3d"
                ? "native accumulated pcd"
                : selectedMap.has_pointcloud_3d
                  ? "snapshot pcd"
                  : "none"
            }
          />
          <StatusMini label="3D topic" value={formatNullable(selectedMap.pointcloud_topic_3d)} />
          <StatusMini
            label="3D points"
            value={
              pointcloudArtifact?.points_saved === null || pointcloudArtifact?.points_saved === undefined
                ? "-"
                : String(pointcloudArtifact.points_saved)
            }
          />
        </div>
      ) : null}
      <label className="form-label" htmlFor="save-map-id">
        新地图名
      </label>
      <input
        id="save-map-id"
        className="text-input"
        value={saveMapId}
        onChange={(event) => onSaveMapIdChange(event.target.value)}
        placeholder="site_map_20260424_0945"
      />
      <button className="secondary-button full-width-button" disabled={stackBusy || !isMapping || !saveMapId} onClick={onSaveMap}>
        保存当前地图
      </button>
      <p className="panel-message">{formatNullable(saveMapReason, "当前可保存建图结果")}</p>
      <button
        className="secondary-button full-width-button"
        disabled={stackBusy || !selectedMapId || !selectedMap?.has_pointcloud_3d}
        onClick={onProjectPcd}
      >
        PCD → 2D 投影
      </button>
      <p className="panel-message">{formatNullable(projectPcdReason, "从已保存的3D点云生成Nav2导航用2D地图")}</p>
    </section>
  );
}

export function NavigationTaskSection({
  navigation,
}: Pick<ControlSidebarProps, "navigation">) {
  return (
    <section className="panel">
      <h2>任务状态</h2>
      <TaskStateChip state={navigation?.state ?? "idle"} />
      <p className="panel-message">{formatNullable(navigation?.message)}</p>
      <StatusMini label="backend" value={formatNullable(navigation?.backend)} />
      <StatusMini label="goal backend" value={navigation?.action_server_ready ? "ready" : "unavailable"} />
      <StatusMini
        label="distance remaining"
        value={formatNullable(
          navigation?.feedback?.distance_remaining === undefined
            ? null
            : `${formatNumber(Number(navigation.feedback.distance_remaining), 2)} m`,
        )}
      />
    </section>
  );
}

export function SelectedGoalSection({
  selectedGoal,
  canSendGoal,
  canSetInitialPose,
  sendGoalReason,
  setInitialPoseReason,
  onSetInitialPose,
  onSendGoal,
  onCancelGoal,
}: Pick<
  ControlSidebarProps,
  | "selectedGoal"
  | "canSendGoal"
  | "canSetInitialPose"
  | "sendGoalReason"
  | "setInitialPoseReason"
  | "onSetInitialPose"
  | "onSendGoal"
  | "onCancelGoal"
>) {
  return (
    <section className="panel">
      <h2>当前选点</h2>
      <StatusMini label="x" value={formatNumber(selectedGoal?.x, 2)} />
      <StatusMini label="y" value={formatNumber(selectedGoal?.y, 2)} />
      <StatusMini label="yaw" value={formatNumber(selectedGoal?.yaw, 2)} />
      <div className="button-group">
        <button className="secondary-button" disabled={!selectedGoal || !canSetInitialPose} onClick={onSetInitialPose}>
          设置初始位姿
        </button>
        <button className="primary-button" disabled={!selectedGoal || !canSendGoal} onClick={onSendGoal}>
          发送导航
        </button>
        <button className="danger-button" onClick={onCancelGoal}>
          停止导航
        </button>
      </div>
      <p className="panel-message">{formatNullable(setInitialPoseReason, "当前模式允许设置初始位姿")}</p>
      <p className="panel-message">{formatNullable(sendGoalReason, "当前模式允许发送导航目标")}</p>
    </section>
  );
}

export function RecentNoticeSection({
  stack,
  lastError,
  lastSuccess,
}: Pick<ControlSidebarProps, "stack" | "lastError" | "lastSuccess">) {
  return (
    <section className="panel">
      <h2>最近提示</h2>
      <p className="panel-message">{formatNullable(stack?.message, "暂无栈状态提示")}</p>
      <p className="panel-message">{`log: ${formatNullable(stack?.log_file)}`}</p>
      <p className={`notice ${lastError ? "notice-error" : ""}`}>{formatNullable(lastError, "暂无错误")}</p>
      <p className={`notice ${lastSuccess ? "notice-success" : ""}`}>{formatNullable(lastSuccess, "暂无成功提示")}</p>
    </section>
  );
}

export function TaskStateChip({ state }: { state: string }) {
  return <div className={`task-chip task-chip-${state}`}>{state}</div>;
}

interface TaskRouteManagerSectionProps {
  routes: TaskRouteSummary[];
  routeStatus: TaskRouteStatus | null;
  selectedRouteId: string;
  routeDraftId: string;
  routeYaml: string;
  selectedMapId: string;
  routeBusy: boolean;
  selectedGoal: NavigationGoal | null;
  onSelectedRouteChange: (routeId: string) => void;
  onRouteDraftIdChange: (routeId: string) => void;
  onRouteYamlChange: (routeYaml: string) => void;
  onRefreshRoutes: () => void;
  onCreateRouteTemplate: () => void;
  onAppendSelectedGoal: () => void;
  onSaveRoute: () => void;
  onDeleteRoute: () => void;
  onRunRoute: () => void;
  onStopRoute: () => void;
}

export function TaskRouteManagerSection({
  routes,
  routeStatus,
  selectedRouteId,
  routeDraftId,
  routeYaml,
  selectedMapId,
  routeBusy,
  selectedGoal,
  onSelectedRouteChange,
  onRouteDraftIdChange,
  onRouteYamlChange,
  onRefreshRoutes,
  onCreateRouteTemplate,
  onAppendSelectedGoal,
  onSaveRoute,
  onDeleteRoute,
  onRunRoute,
  onStopRoute,
}: TaskRouteManagerSectionProps) {
  return (
    <section className="panel">
      <h2>任务选择</h2>
      <TaskStateChip state={routeStatus?.route_state ?? routeStatus?.state ?? "idle"} />
      <StatusMini label="active map" value={formatNullable(routeStatus?.active_map ?? (selectedMapId || null))} />
      <StatusMini label="current mode" value={formatNullable(routeStatus?.current_mode)} />
      <StatusMini label="report" value={formatNullable(routeStatus?.report_path)} />
      <label className="form-label" htmlFor="route-select">
        路线列表
      </label>
      <select
        id="route-select"
        className="select-input"
        value={selectedRouteId}
        onChange={(event) => onSelectedRouteChange(event.target.value)}
      >
        <option value="">选择路线</option>
        {routes.map((route) => (
          <option key={route.route_id} value={route.route_id}>
            {route.route_id}
          </option>
        ))}
      </select>
      {selectedRouteId ? (
        <div className="route-summary-box">
          {routes
            .filter((route) => route.route_id === selectedRouteId)
            .map((route) => (
              <div key={route.route_id} className="route-summary-grid">
                <StatusMini label="mission" value={formatNullable(route.mission_name)} />
                <StatusMini label="waypoints" value={route.waypoint_count} />
                <StatusMini label="updated" value={formatNullable(route.updated_at)} />
              </div>
            ))}
        </div>
      ) : null}
      <div className="button-group route-inline-buttons">
        <button type="button" className="secondary-button" disabled={routeBusy} onClick={onRefreshRoutes}>
          刷新路线
        </button>
        <button type="button" className="secondary-button" disabled={routeBusy} onClick={onCreateRouteTemplate}>
          新建模板
        </button>
      </div>
      <label className="form-label" htmlFor="route-id-input">
        路线 ID
      </label>
      <input
        id="route-id-input"
        className="text-input"
        value={routeDraftId}
        onChange={(event) => onRouteDraftIdChange(event.target.value)}
        placeholder="office_loop"
      />
      <label className="form-label" htmlFor="route-yaml-input">
        路线 YAML
      </label>
      <textarea
        id="route-yaml-input"
        className="text-area-input"
        value={routeYaml}
        onChange={(event) => onRouteYamlChange(event.target.value)}
        placeholder="mission_name: office_loop"
        rows={12}
      />
      <div className="button-group">
        <button type="button" className="secondary-button" disabled={routeBusy || !selectedGoal} onClick={onAppendSelectedGoal}>
          追加当前选点
        </button>
        <button type="button" className="primary-button" disabled={routeBusy || !routeDraftId || !routeYaml.trim()} onClick={onSaveRoute}>
          保存路线
        </button>
        <button type="button" className="danger-button" disabled={routeBusy || !selectedRouteId} onClick={onDeleteRoute}>
          删除路线
        </button>
        <button type="button" className="primary-button" disabled={routeBusy || !selectedRouteId || !selectedMapId} onClick={onRunRoute}>
          执行路线
        </button>
        <button type="button" className="danger-button" disabled={routeBusy} onClick={onStopRoute}>
          停止路线
        </button>
      </div>
    </section>
  );
}

interface ObstacleManagerSectionProps {
  selectedMapId: string;
  obstacles: VirtualObstacleZone[];
  obstacleBusy: boolean;
  obstacleLabel: string;
  obstacleRadius: string;
  selectedGoal: NavigationGoal | null;
  pose: RobotPose | null;
  onObstacleLabelChange: (value: string) => void;
  onObstacleRadiusChange: (value: string) => void;
  onAddObstacleFromGoal: () => void;
  onAddObstacleFromPose: () => void;
  onDeleteObstacle: (obstacleId: string) => void;
}

export function ObstacleManagerSection({
  selectedMapId,
  obstacles,
  obstacleBusy,
  obstacleLabel,
  obstacleRadius,
  selectedGoal,
  pose,
  onObstacleLabelChange,
  onObstacleRadiusChange,
  onAddObstacleFromGoal,
  onAddObstacleFromPose,
  onDeleteObstacle,
}: ObstacleManagerSectionProps) {
  return (
    <section className="panel">
      <h2>障碍物选择</h2>
      <p className="panel-message">当前是 ROS2 虚拟禁入区：保存在地图目录里，并在发目标、发初始位姿、执行路线前统一校验。</p>
      <StatusMini label="selected map" value={formatNullable(selectedMapId || null)} />
      <StatusMini
        label="selected goal"
        value={selectedGoal ? `${formatNumber(selectedGoal.x, 2)}, ${formatNumber(selectedGoal.y, 2)}` : "暂无"}
      />
      <StatusMini
        label="robot pose"
        value={pose?.available && pose.x !== null && pose.y !== null ? `${formatNumber(pose.x, 2)}, ${formatNumber(pose.y, 2)}` : "暂无"}
      />
      <label className="form-label" htmlFor="obstacle-label">
        障碍物名称
      </label>
      <input
        id="obstacle-label"
        className="text-input"
        value={obstacleLabel}
        onChange={(event) => onObstacleLabelChange(event.target.value)}
        placeholder="dock_keepout"
      />
      <label className="form-label" htmlFor="obstacle-radius">
        半径（米）
      </label>
      <input
        id="obstacle-radius"
        className="text-input"
        value={obstacleRadius}
        onChange={(event) => onObstacleRadiusChange(event.target.value)}
        placeholder="0.60"
      />
      <div className="button-group">
        <button
          type="button"
          className="secondary-button"
          disabled={obstacleBusy || !selectedMapId || !selectedGoal}
          onClick={onAddObstacleFromGoal}
        >
          用当前选点创建禁入区
        </button>
        <button
          type="button"
          className="secondary-button"
          disabled={obstacleBusy || !selectedMapId || !pose?.available || pose.x === null || pose.y === null}
          onClick={onAddObstacleFromPose}
        >
          用机器人当前位置创建禁入区
        </button>
      </div>
      <div className="obstacle-list">
        {obstacles.length > 0 ? (
          obstacles.map((obstacle) => (
            <div key={obstacle.obstacle_id} className="obstacle-card">
              <div>
                <div className="obstacle-card-title">{obstacle.label || obstacle.obstacle_id}</div>
                <div className="obstacle-card-meta">
                  {`${formatNumber(obstacle.x, 2)}, ${formatNumber(obstacle.y, 2)} / r=${formatNumber(obstacle.radius, 2)} m`}
                </div>
              </div>
              <button
                type="button"
                className="danger-button"
                disabled={obstacleBusy}
                onClick={() => onDeleteObstacle(obstacle.obstacle_id)}
              >
                删除
              </button>
            </div>
          ))
        ) : (
          <p className="panel-message">当前地图还没有虚拟禁入区。</p>
        )}
      </div>
    </section>
  );
}

export function StatusMini({ label, value }: { label: string; value: string | number | null | undefined }) {
  return (
    <div className="status-row">
      <span className="status-label">{label}</span>
      <span className="status-value">{String(value ?? "-")}</span>
    </div>
  );
}
