import type { RobotPose, RobotStatus, StackStatus, SystemHealth } from "../types";
import { formatNullable, formatNumber, formatStatusSummary } from "../utils/format";

interface StatusSidebarProps {
  status: RobotStatus | null;
  pose: RobotPose | null;
  health: SystemHealth | null;
  stack: StackStatus | null;
  backendConnected: boolean;
  websocketConnected: boolean;
}

export function StatusSidebar({
  status,
  pose,
  health,
  stack,
  backendConnected,
  websocketConnected,
}: StatusSidebarProps) {
  const poseAgeMs = pose?.stamp ? Date.now() - Date.parse(pose.stamp) : Number.POSITIVE_INFINITY;
  const localizationLabel =
    status?.localization_ok !== true
      ? "localization lost"
      : pose?.available && poseAgeMs > 2000
        ? "amcl stale"
        : "localization ok";

  return (
    <aside className="sidebar">
      <section className="panel">
        <h2>连接状态</h2>
        <StatusRow label="后端连接" value={backendConnected ? "online" : "offline"} />
        <StatusRow label="WebSocket" value={websocketConnected ? "connected" : "disconnected"} />
        <StatusRow label="ROS 线程" value={health?.ros_thread_alive ? "alive" : "unknown"} />
      </section>

      <section className="panel">
        <h2>系统状态</h2>
        <StatusRow label="栈模式" value={formatNullable(stack?.mode)} />
        <StatusRow label="ready" value={status?.system_ready === true ? "true" : "false"} />
        <StatusRow label="定位" value={localizationLabel} />
        <StatusRow label="lidar" value={formatStatusSummary(status?.lidar_status)} />
        <StatusRow label="SDK" value={formatStatusSummary(status?.sdk_status)} />
        <StatusRow label="task mgr" value={formatStatusSummary(status?.task_manager_status)} />
      </section>

      <section className="panel">
        <h2>节点状态</h2>
        {stack?.nodes.length ? (
          stack.nodes.map((node) => (
            <StatusRow
              key={node.key}
              label={node.label}
              value={node.state || (node.running ? "running" : node.required ? "missing" : "optional")}
            />
          ))
        ) : (
          <StatusRow label="节点" value="未启动" />
        )}
      </section>

      <section className="panel">
        <h2>运行信息</h2>
        <StatusRow label="线速度 x" value={`${formatNumber(status?.velocity_linear_x, 3)} m/s`} />
        <StatusRow label="角速度 z" value={`${formatNumber(status?.velocity_angular_z, 3)} rad/s`} />
        <StatusRow label="active_map" value={formatNullable(status?.active_map)} />
        <StatusRow label="map manager" value={formatStatusSummary(status?.map_manager_status)} />
      </section>

      <section className="panel">
        <h2>位姿</h2>
        <StatusRow label="x" value={formatNumber(pose?.x, 2)} />
        <StatusRow label="y" value={formatNumber(pose?.y, 2)} />
        <StatusRow label="yaw" value={formatNumber(pose?.yaw, 2)} />
        <StatusRow label="pose frame" value={formatNullable(pose?.frame_id)} />
      </section>

      <section className="panel">
        <h2>健康检查</h2>
        <StatusRow label="map received" value={health?.map_received ? "true" : "false"} />
        <StatusRow label="pose received" value={health?.pose_received ? "true" : "false"} />
        <StatusRow label="camera received" value={health?.camera_received ? "true" : "false"} />
        <StatusRow label="action ready" value={health?.action_server_ready ? "true" : "false"} />
        <StatusRow label="last error" value={formatNullable(health?.last_error)} />
      </section>
    </aside>
  );
}

function StatusRow({ label, value }: { label: string; value: string }) {
  return (
    <div className="status-row">
      <span className="status-label">{label}</span>
      <span className="status-value">{value}</span>
    </div>
  );
}
