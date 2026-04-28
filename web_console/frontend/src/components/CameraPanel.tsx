import type { CameraFrame } from "../types";
import { formatNullable } from "../utils/format";

interface CameraPanelProps {
  camera: CameraFrame | null;
}

export function CameraPanel({ camera }: CameraPanelProps) {
  const ageMs = camera?.stamp ? Date.now() - Date.parse(camera.stamp) : Number.POSITIVE_INFINITY;
  const stale = !camera?.available || ageMs > 3000;

  return (
    <section className="camera-panel">
      <div className="camera-panel-header">
        <div>
          <h2>A2 前向相机</h2>
          <p>{formatNullable(camera?.topic, "等待相机 topic")}</p>
        </div>
        <span className={`indicator ${!stale ? "indicator-ok" : "indicator-warn"}`}>
          {!stale ? "live" : "stale"}
        </span>
      </div>
      <div className="camera-frame">
        {camera?.data_url ? (
          <img src={camera.data_url} alt="A2 camera stream" />
        ) : (
          <div className="camera-placeholder">暂无相机图像</div>
        )}
      </div>
      <div className="camera-meta">
        <span>frame={formatNullable(camera?.frame_id)}</span>
        <span>encoding={formatNullable(camera?.encoding)}</span>
        <span>size={camera?.width && camera?.height ? `${camera.width}x${camera.height}` : "暂无数据"}</span>
      </div>
    </section>
  );
}
