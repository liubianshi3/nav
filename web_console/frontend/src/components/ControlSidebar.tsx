import type { NavigationGoal, NavigationTaskState, SavedMapInfo, StackStatus } from "../types";
import { formatNumber, formatNullable } from "../utils/format";

interface ControlSidebarProps {
  navigation: NavigationTaskState | null;
  stack: StackStatus | null;
  maps: SavedMapInfo[];
  selectedMapId: string;
  saveMapId: string;
  selectedGoal: NavigationGoal | null;
  canSendGoal: boolean;
  canSetInitialPose: boolean;
  stackBusy: boolean;
  onSelectedMapChange: (mapId: string) => void;
  onSaveMapIdChange: (mapId: string) => void;
  onStartMapping: () => void;
  onStartNavigation: () => void;
  onStopStack: () => void;
  onSaveMap: () => void;
  onSetInitialPose: () => void;
  onSendGoal: () => void;
  onCancelGoal: () => void;
  lastError: string | null;
  lastSuccess: string | null;
}

export function ControlSidebar({
  navigation,
  stack,
  maps,
  selectedMapId,
  saveMapId,
  selectedGoal,
  canSendGoal,
  canSetInitialPose,
  stackBusy,
  onSelectedMapChange,
  onSaveMapIdChange,
  onStartMapping,
  onStartNavigation,
  onStopStack,
  onSaveMap,
  onSetInitialPose,
  onSendGoal,
  onCancelGoal,
  lastError,
  lastSuccess,
}: ControlSidebarProps) {
  const isMapping = stack?.mode === "mapping";
  const isNavigation = stack?.mode === "navigation";

  return (
    <aside className="sidebar">
      <section className="panel">
        <h2>模式控制</h2>
        <TaskStateChip state={stack?.mode ?? "stopped"} />
        <StatusMini label="pid" value={formatNullable(stack?.pid)} />
        <StatusMini label="地图" value={formatNullable(stack?.selected_map_id)} />
        <div className="button-group">
          <button className="secondary-button" disabled={stackBusy || isMapping} onClick={onStartMapping}>
            启动建图模式
          </button>
          <button className="danger-button" disabled={stackBusy || stack?.mode === "stopped"} onClick={onStopStack}>
            停止当前栈
          </button>
        </div>
      </section>

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
      </section>

      <section className="panel">
        <h2>任务状态</h2>
        <TaskStateChip state={navigation?.state ?? "idle"} />
        <p className="panel-message">{formatNullable(navigation?.message)}</p>
        <StatusMini label="action server" value={navigation?.action_server_ready ? "ready" : "unavailable"} />
        <StatusMini
          label="distance remaining"
          value={formatNullable(
            navigation?.feedback?.distance_remaining === undefined
              ? null
              : `${formatNumber(Number(navigation.feedback.distance_remaining), 2)} m`,
          )}
        />
      </section>

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
      </section>

      <section className="panel">
        <h2>最近提示</h2>
        <p className={`notice ${lastError ? "notice-error" : ""}`}>{formatNullable(lastError, "暂无错误")}</p>
        <p className={`notice ${lastSuccess ? "notice-success" : ""}`}>{formatNullable(lastSuccess, "暂无成功提示")}</p>
      </section>
    </aside>
  );
}

function TaskStateChip({ state }: { state: string }) {
  return <div className={`task-chip task-chip-${state}`}>{state}</div>;
}

function StatusMini({ label, value }: { label: string; value: string | number | null | undefined }) {
  return (
    <div className="status-row">
      <span className="status-label">{label}</span>
      <span className="status-value">{String(value ?? "-")}</span>
    </div>
  );
}
