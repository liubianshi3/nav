import { useEffect, useMemo, useRef, useState } from "react";
import type { MouseEvent as ReactMouseEvent } from "react";

import type { MapArtifactInfo, PointCloudSnapshot, SavedMapInfo } from "../types";

interface PointCloudCanvas3DProps {
  pointcloud: PointCloudSnapshot | null;
  selectedMap: SavedMapInfo | null;
}

interface View3DState {
  yaw: number;
  pitch: number;
  zoom: number;
}

interface Point3D {
  x: number;
  y: number;
  z: number;
}

export function PointCloudCanvas3D({ pointcloud, selectedMap }: PointCloudCanvas3DProps) {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const dragRef = useRef({ active: false, x: 0, y: 0 });
  const [view, setView] = useState<View3DState>({ yaw: 0.8, pitch: -0.45, zoom: 72 });
  const [artifactPoints, setArtifactPoints] = useState<Point3D[]>([]);
  const [artifactState, setArtifactState] = useState("等待 3D 资产");

  const selectedArtifact = useMemo(
    () => selectedMap?.artifacts.find((artifact) => artifact.kind === "pointcloud_snapshot_3d") ?? null,
    [selectedMap],
  );

  const livePoints = useMemo(
    () => (pointcloud?.loaded ? pointcloud.points.map(([x, y, z]) => ({ x, y, z })) : []),
    [pointcloud],
  );

  const sourcePoints = livePoints.length > 0 ? livePoints : artifactPoints;
  const sourceLabel = livePoints.length > 0 ? "live pointcloud" : selectedArtifact ? "saved pointcloud" : "none";

  useEffect(() => {
    if (!selectedMap || !selectedArtifact || livePoints.length > 0) {
      if (!selectedArtifact) {
        setArtifactState("当前地图没有 3D 点云资产");
      }
      if (livePoints.length > 0) {
        setArtifactState("使用实时点云");
      }
      return;
    }

    let cancelled = false;
    setArtifactState("正在加载已保存 3D 点云...");
    fetch(`/api/maps/${encodeURIComponent(selectedMap.map_id)}/artifacts/${encodeURIComponent(selectedArtifact.path)}`)
      .then((response) => {
        if (!response.ok) {
          throw new Error(`HTTP ${response.status}`);
        }
        return response.text();
      })
      .then((text) => {
        if (cancelled) {
          return;
        }
        const parsed = parseAsciiPcd(text, 12000);
        setArtifactPoints(parsed);
        setArtifactState(parsed.length > 0 ? `已加载 ${parsed.length} 个采样点` : "PCD 中没有可显示点");
      })
      .catch((error) => {
        if (cancelled) {
          return;
        }
        setArtifactPoints([]);
        setArtifactState(error instanceof Error ? error.message : "加载 3D 点云失败");
      });
    return () => {
      cancelled = true;
    };
  }, [selectedArtifact, selectedMap, livePoints.length]);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) {
      return;
    }
    const context = canvas.getContext("2d");
    if (!context) {
      return;
    }

    const dpr = window.devicePixelRatio || 1;
    const width = Math.floor(canvas.clientWidth * dpr);
    const height = Math.floor(canvas.clientHeight * dpr);
    if (canvas.width !== width || canvas.height !== height) {
      canvas.width = width;
      canvas.height = height;
    }

    context.setTransform(dpr, 0, 0, dpr, 0, 0);
    context.clearRect(0, 0, canvas.clientWidth, canvas.clientHeight);
    context.fillStyle = "#08111d";
    context.fillRect(0, 0, canvas.clientWidth, canvas.clientHeight);

    drawGrid(context, canvas.clientWidth, canvas.clientHeight);

    if (sourcePoints.length === 0) {
      context.fillStyle = "#cbd5e1";
      context.font = "600 16px 'Segoe UI', sans-serif";
      context.fillText("暂无 3D 点云", 24, 36);
      return;
    }

    const center = computeCenter(sourcePoints);
    const projected = sourcePoints
      .map((point) => projectPoint(point, center, view, canvas.clientWidth, canvas.clientHeight))
      .filter((point): point is ProjectedPoint => point !== null)
      .sort((left, right) => left.depth - right.depth);

    for (const point of projected) {
      context.globalAlpha = point.alpha;
      context.fillStyle = point.color;
      context.fillRect(point.x, point.y, point.size, point.size);
    }
    context.globalAlpha = 1;
  }, [artifactPoints, livePoints, sourcePoints, view]);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) {
      return;
    }
    const onWheel = (event: WheelEvent) => {
      event.preventDefault();
      setView((current) => ({
        ...current,
        zoom: clamp(current.zoom * (event.deltaY < 0 ? 1.08 : 0.92), 18, 260),
      }));
    };
    canvas.addEventListener("wheel", onWheel, { passive: false });
    return () => canvas.removeEventListener("wheel", onWheel);
  }, []);

  const handleMouseDown = (event: ReactMouseEvent<HTMLCanvasElement>) => {
    dragRef.current = { active: true, x: event.clientX, y: event.clientY };
  };

  const handleMouseMove = (event: ReactMouseEvent<HTMLCanvasElement>) => {
    if (!dragRef.current.active) {
      return;
    }
    const dx = event.clientX - dragRef.current.x;
    const dy = event.clientY - dragRef.current.y;
    dragRef.current.x = event.clientX;
    dragRef.current.y = event.clientY;
    setView((current) => ({
      ...current,
      yaw: current.yaw + dx * 0.008,
      pitch: clamp(current.pitch + dy * 0.008, -1.4, 1.4),
    }));
  };

  const handleMouseUp = () => {
    dragRef.current.active = false;
  };

  return (
    <div className="map-shell pointcloud-shell">
      <canvas
        ref={canvasRef}
        className="map-canvas"
        onMouseDown={handleMouseDown}
        onMouseMove={handleMouseMove}
        onMouseUp={handleMouseUp}
        onMouseLeave={handleMouseUp}
      />
      <div className="map-overlay pointcloud-overlay">
        <span>{`3D source: ${sourceLabel}`}</span>
        <span>{artifactState}</span>
        <span>{`points=${sourcePoints.length}`}</span>
      </div>
    </div>
  );
}

interface ProjectedPoint {
  x: number;
  y: number;
  depth: number;
  size: number;
  alpha: number;
  color: string;
}

function parseAsciiPcd(text: string, maxPoints: number): Point3D[] {
  const lines = text.split(/\r?\n/);
  const dataIndex = lines.findIndex((line) => line.trim().toUpperCase() === "DATA ASCII");
  if (dataIndex < 0) {
    return [];
  }
  const pointLines = lines.slice(dataIndex + 1).filter((line) => line.trim().length > 0);
  const stride = Math.max(1, Math.ceil(pointLines.length / maxPoints));
  const points: Point3D[] = [];
  for (let index = 0; index < pointLines.length; index += stride) {
    const parts = pointLines[index].trim().split(/\s+/);
    if (parts.length < 3) {
      continue;
    }
    const x = Number(parts[0]);
    const y = Number(parts[1]);
    const z = Number(parts[2]);
    if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) {
      continue;
    }
    points.push({ x, y, z });
  }
  return points;
}

function computeCenter(points: Point3D[]) {
  let sumX = 0;
  let sumY = 0;
  let sumZ = 0;
  for (const point of points) {
    sumX += point.x;
    sumY += point.y;
    sumZ += point.z;
  }
  return {
    x: sumX / points.length,
    y: sumY / points.length,
    z: sumZ / points.length,
  };
}

function projectPoint(
  point: Point3D,
  center: Point3D,
  view: View3DState,
  width: number,
  height: number,
): ProjectedPoint | null {
  const translatedX = point.x - center.x;
  const translatedY = point.y - center.y;
  const translatedZ = point.z - center.z;

  const cosYaw = Math.cos(view.yaw);
  const sinYaw = Math.sin(view.yaw);
  const yawX = translatedX * cosYaw - translatedY * sinYaw;
  const yawY = translatedX * sinYaw + translatedY * cosYaw;

  const cosPitch = Math.cos(view.pitch);
  const sinPitch = Math.sin(view.pitch);
  const pitchY = yawY * cosPitch - translatedZ * sinPitch;
  const pitchZ = yawY * sinPitch + translatedZ * cosPitch;

  const cameraZ = pitchZ + view.zoom;
  if (cameraZ <= 1) {
    return null;
  }

  const focal = 220;
  const scale = focal / cameraZ;
  const screenX = width / 2 + yawX * scale;
  const screenY = height / 2 - pitchY * scale;
  const alpha = clamp(0.15 + scale * 0.9, 0.12, 0.95);
  const size = clamp(scale * 1.6, 1, 3.2);
  const hue = clamp(210 + pitchZ * 4, 180, 230);
  return {
    x: screenX,
    y: screenY,
    depth: cameraZ,
    size,
    alpha,
    color: `hsl(${hue}, 85%, 72%)`,
  };
}

function drawGrid(context: CanvasRenderingContext2D, width: number, height: number) {
  context.strokeStyle = "rgba(148, 163, 184, 0.12)";
  context.lineWidth = 1;
  for (let x = 0; x <= width; x += 48) {
    context.beginPath();
    context.moveTo(x, 0);
    context.lineTo(x, height);
    context.stroke();
  }
  for (let y = 0; y <= height; y += 48) {
    context.beginPath();
    context.moveTo(0, y);
    context.lineTo(width, y);
    context.stroke();
  }
}

function clamp(value: number, min: number, max: number) {
  return Math.min(max, Math.max(min, value));
}
