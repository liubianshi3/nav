import type { BatterySnapshot, RecoveryStatus, RobotPose, RobotStatus, StackStatus, SystemHealth } from "../types";
import { formatNullable, formatNumber, formatStatusSummary } from "../utils/format";

export interface StatusSidebarProps {
  status: RobotStatus | null;
  pose: RobotPose | null;
  health: SystemHealth | null;
  stack: StackStatus | null;
  battery: BatterySnapshot | null;
  recovery: RecoveryStatus | null;
  backendConnected: boolean;
  websocketConnected: boolean;
}

export function StatusSidebar(props: StatusSidebarProps) {
  return (
    <div className="console-section-stack">
      <ConnectionStatusSection
        backendConnected={props.backendConnected}
        websocketConnected={props.websocketConnected}
        health={props.health}
      />
      <SystemStatusSection status={props.status} pose={props.pose} stack={props.stack} />
      <NodeStatusSection stack={props.stack} />
      <BatterySection battery={props.battery} />
      <RecoverySection recovery={props.recovery} />
      <RuntimeInfoSection status={props.status} stack={props.stack} />
      <PoseSection pose={props.pose} />
      <HealthSection health={props.health} />
    </div>
  );
}

function localizationLabel(status: RobotStatus | null, pose: RobotPose | null) {
  const poseAgeMs = pose?.stamp ? Date.now() - Date.parse(pose.stamp) : Number.POSITIVE_INFINITY;
  if (status?.localization_ok !== true) {
    return "localization lost";
  }
  if (pose?.available && poseAgeMs > 2000) {
    return "pose stale";
  }
  return "localization ok";
}

function ndtScoreLabel(status: RobotStatus | null) {
  if (status?.ndt_score == null) return "—";
  const s = status.ndt_score;
  const ok = status.ndt_healthy !== false;
  return `${s.toFixed(3)} ${ok ? "✓" : "✗"}`;
}

export function ConnectionStatusSection({
  backendConnected,
  websocketConnected,
  health,
}: Pick<StatusSidebarProps, "backendConnected" | "websocketConnected" | "health">) {
  return (
    <section className="panel">
      <h2>连接状态</h2>
      <StatusRow label="后端连接" value={backendConnected ? "online" : "offline"} />
      <StatusRow label="WebSocket" value={websocketConnected ? "connected" : "disconnected"} />
      <StatusRow label="ROS 线程" value={health?.ros_thread_alive ? "alive" : "unknown"} />
    </section>
  );
}

export function SystemStatusSection({
  status,
  pose,
  stack,
}: Pick<StatusSidebarProps, "status" | "pose" | "stack">) {
  const robotProfile = status?.sdk_status?.fields?.robot_profile || null;
  const robotModel = status?.sdk_status?.fields?.robot_model || null;
  const lidarProfile = status?.lidar_status?.fields?.profile || null;
  const lidarModel = status?.lidar_status?.fields?.model || status?.lidar_status?.fields?.detected_model || null;
  const lidarTopic = status?.lidar_status?.fields?.topic || null;
  const cameraProfile = status?.camera_status?.fields?.profile || null;
  const cameraModel = status?.camera_status?.fields?.model || status?.camera_status?.fields?.detected_model || null;
  const cameraTopic = status?.camera_status?.fields?.topic || null;
  return (
    <section className="panel">
      <h2>系统状态</h2>
      <StatusRow label="栈模式" value={formatNullable(stack?.mode)} />
      <StatusRow label="定位模式" value={formatNullable(stack?.localization_mode ?? status?.safety_status?.fields?.localization_mode)} />
      <StatusRow label="运动模式" value={formatNullable(stack?.motion_mode)} />
      <StatusRow label="防撞配置" value={formatNullable(stack?.collision_monitor_profile)} />
      <StatusRow label="ready" value={status?.system_ready === true ? "true" : "false"} />
      <StatusRow label="robot" value={formatNullable(robotModel || robotProfile)} />
      <StatusRow label="lidar model" value={formatNullable(lidarModel || lidarProfile)} />
      <StatusRow label="lidar topic" value={formatNullable(lidarTopic)} />
      <StatusRow label="camera model" value={formatNullable(cameraModel || cameraProfile)} />
      <StatusRow label="camera topic" value={formatNullable(cameraTopic)} />
      <StatusRow label="定位" value={localizationLabel(status, pose)} />
      <StatusRow label="NDT score" value={ndtScoreLabel(status)} />
      <StatusRow label="NDT state" value={formatStatusSummary(status?.relocalization_status)} />
      <StatusRow label="safety" value={formatStatusSummary(status?.safety_status)} />
      <StatusRow label="lidar" value={formatStatusSummary(status?.lidar_status)} />
      <StatusRow label="camera" value={formatStatusSummary(status?.camera_status)} />
      <StatusRow label="SDK" value={formatStatusSummary(status?.sdk_status)} />
      <StatusRow label="control" value={formatStatusSummary(status?.control_status)} />
      <StatusRow label="task mgr" value={formatStatusSummary(status?.task_manager_status)} />
    </section>
  );
}

export function NodeStatusSection({ stack }: Pick<StatusSidebarProps, "stack">) {
  return (
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
  );
}

export function RuntimeInfoSection({ status, stack }: Pick<StatusSidebarProps, "status" | "stack">) {
  return (
    <section className="panel">
      <h2>运行信息</h2>
      <StatusRow label="线速度 x" value={`${formatNumber(status?.velocity_linear_x, 3)} m/s`} />
      <StatusRow label="角速度 z" value={`${formatNumber(status?.velocity_angular_z, 3)} rad/s`} />
      <StatusRow label="active_map" value={formatNullable(status?.active_map)} />
      <StatusRow label="Nav2 3D" value={stack?.enable_nav2_3d == null ? "—" : String(stack.enable_nav2_3d)} />
      <StatusRow label="motion" value={stack?.live_motion ? "live" : "planning-only"} />
      <StatusRow label="collision cfg" value={formatNullable(stack?.collision_monitor_config)} />
      <StatusRow label="规划器" value={formatNullable(status?.planner_type)} />
      <StatusRow label="行为树" value={formatNullable(status?.bt_filename)} />
      <StatusRow label="gait" value={formatNullable(status?.control_status?.fields?.gait_type)} />
      <StatusRow label="gait state" value={formatNullable(status?.control_status?.fields?.gait_state)} />
      <StatusRow label="map manager" value={formatStatusSummary(status?.map_manager_status)} />
      <StatusRow label="score/pose Δ" value={formatNullable(status?.relocalization_status?.fields?.last_score_pose_delta_sec)} />
      <StatusRow label="odom age" value={formatNullable(status?.relocalization_status?.fields?.odom_receive_age)} />
    </section>
  );
}

export function PoseSection({ pose }: Pick<StatusSidebarProps, "pose">) {
  return (
    <section className="panel">
      <h2>位姿</h2>
      <StatusRow label="x" value={formatNumber(pose?.x, 2)} />
      <StatusRow label="y" value={formatNumber(pose?.y, 2)} />
      <StatusRow label="yaw" value={formatNumber(pose?.yaw, 2)} />
      <StatusRow label="pose frame" value={formatNullable(pose?.frame_id)} />
    </section>
  );
}

export function BatterySection({ battery }: Pick<StatusSidebarProps, "battery">) {
  if (!battery?.available) {
    return (
      <section className="panel">
        <h2>电池</h2>
        <StatusRow label="状态" value="无数据" />
      </section>
    );
  }
  const pct = battery.percentage;
  const level =
    pct === null ? "unknown"
    : pct <= 10 ? "critical"
    : pct <= 20 ? "low"
    : pct <= 50 ? "mid"
    : "good";
  const label =
    pct !== null ? `${pct.toFixed(0)}%` : "—";
  const charging = battery.charging ? " 🔌" : "";
  return (
    <section className="panel">
      <h2>电池</h2>
      <StatusRow label="电量" value={`${label}${charging}`} />
      {battery.voltage != null && (
        <StatusRow label="电压" value={`${battery.voltage.toFixed(1)}V`} />
      )}
      <StatusRow label="状态" value={level} />
    </section>
  );
}

export function RecoverySection({ recovery }: Pick<StatusSidebarProps, "recovery">) {
  if (!recovery?.active && !recovery?.sequence.length) {
    return (
      <section className="panel">
        <h2>恢复</h2>
        <StatusRow label="状态" value="无恢复事件" />
      </section>
    );
  }
  return (
    <section className="panel">
      <h2>恢复</h2>
      <StatusRow
        label="状态"
        value={recovery.active ? `进行中: ${recovery.step ?? "—"}` : "待命中"}
      />
      <StatusRow label="序列" value={recovery.sequence.length ? recovery.sequence.join(" → ") : "—"} />
      <StatusRow label="恢复" value={recovery.recovered === true ? "✓ 成功" : recovery.recovered === false ? "✗ 未恢复" : "—"} />
      <StatusRow label="尝试" value={recovery.attempts > 0 ? String(recovery.attempts) : "—"} />
    </section>
  );
}

export function HealthSection({ health }: Pick<StatusSidebarProps, "health">) {
  return (
    <section className="panel">
      <h2>健康检查</h2>
      <StatusRow label="map received" value={health?.map_received ? "true" : "false"} />
      <StatusRow label="pose received" value={health?.pose_received ? "true" : "false"} />
      <StatusRow label="camera received" value={health?.camera_received ? "true" : "false"} />
      <StatusRow label="goal backend" value={health?.action_server_ready ? "true" : "false"} />
      <StatusRow label="last error" value={formatNullable(health?.last_error)} />
    </section>
  );
}

export function StatusRow({ label, value }: { label: string; value: string }) {
  return (
    <div className="status-row">
      <span className="status-label">{label}</span>
      <span className="status-value">{value}</span>
    </div>
  );
}
