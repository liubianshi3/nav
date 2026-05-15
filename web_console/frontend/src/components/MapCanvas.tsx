import { useEffect, useRef, useState } from "react";
import type { MouseEvent as ReactMouseEvent } from "react";

import type { MapSnapshot, NavigationGoal, RobotPose, VirtualObstacleZone } from "../types";
import { clamp, mapPixelToWorld, worldToMapPixel } from "../utils/map";

interface MapCanvasProps {
  map: MapSnapshot | null;
  pose: RobotPose | null;
  obstacles: VirtualObstacleZone[];
  selectedGoal: NavigationGoal | null;
  activeGoal: NavigationGoal | null;
  disabled: boolean;
  onSelectGoal: (goal: NavigationGoal) => void;
}

interface ViewState {
  zoom: number;
  panX: number;
  panY: number;
}

export function MapCanvas({
  map,
  pose,
  obstacles,
  selectedGoal,
  activeGoal,
  disabled,
  onSelectGoal,
}: MapCanvasProps) {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const offscreenRef = useRef<HTMLCanvasElement | null>(null);
  const dragRef = useRef({
    active: false,
    moved: false,
    startX: 0,
    startY: 0,
    panX: 0,
    panY: 0,
  });
  const [view, setView] = useState<ViewState>({ zoom: 1.2, panX: 32, panY: 32 });
  const [hoverText, setHoverText] = useState("未悬停地图");
  const viewRef = useRef(view);

  const resetView = () => {
    setView({ zoom: 1.2, panX: 32, panY: 32 });
  };

  const zoomBy = (factor: number) => {
    setView((current) => ({
      ...current,
      zoom: clamp(current.zoom * factor, 0.2, 12),
    }));
  };

  useEffect(() => {
    viewRef.current = view;
  }, [view]);

  useEffect(() => {
    if (!map?.loaded) {
      offscreenRef.current = null;
      return;
    }

    const offscreen = document.createElement("canvas");
    offscreen.width = map.width;
    offscreen.height = map.height;
    const context = offscreen.getContext("2d");
    if (!context) {
      return;
    }

    const imageData = context.createImageData(map.width, map.height);
    for (let y = 0; y < map.height; y += 1) {
      for (let x = 0; x < map.width; x += 1) {
        const sourceY = map.height - 1 - y;
        const sourceIndex = x + sourceY * map.width;
        const value = map.data[sourceIndex] ?? -1;
        const targetIndex = (x + y * map.width) * 4;

        let color = 230;
        if (value < 0) {
          color = 214;
        } else if (value >= 65) {
          color = 54;
        } else if (value > 0) {
          color = 170;
        }

        imageData.data[targetIndex] = color;
        imageData.data[targetIndex + 1] = color;
        imageData.data[targetIndex + 2] = color;
        imageData.data[targetIndex + 3] = 255;
      }
    }

    context.putImageData(imageData, 0, 0);
    offscreenRef.current = offscreen;
  }, [map]);

  useEffect(() => {
    if (map?.loaded) {
      resetView();
    }
  }, [map?.loaded, map?.width, map?.height, map?.resolution]);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) {
      return;
    }
    const context = canvas.getContext("2d");
    if (!context) {
      return;
    }

    const displayWidth = Math.floor(canvas.clientWidth * window.devicePixelRatio);
    const displayHeight = Math.floor(canvas.clientHeight * window.devicePixelRatio);
    if (canvas.width !== displayWidth || canvas.height !== displayHeight) {
      canvas.width = displayWidth;
      canvas.height = displayHeight;
    }

    context.setTransform(window.devicePixelRatio, 0, 0, window.devicePixelRatio, 0, 0);
    context.clearRect(0, 0, canvas.clientWidth, canvas.clientHeight);
    context.fillStyle = "#e8edf3";
    context.fillRect(0, 0, canvas.clientWidth, canvas.clientHeight);

    if (map?.loaded && offscreenRef.current) {
      context.drawImage(
        offscreenRef.current,
        view.panX,
        view.panY,
        map.width * view.zoom,
        map.height * view.zoom,
      );

      context.strokeStyle = "#7691aa";
      context.lineWidth = 1;
      context.strokeRect(view.panX, view.panY, map.width * view.zoom, map.height * view.zoom);

      if (activeGoal) {
        drawGoal(context, map, activeGoal, view, "#b91c1c");
      }
      if (selectedGoal) {
        drawGoal(context, map, selectedGoal, view, "#2563eb");
      }
      if (obstacles.length > 0) {
        for (const obstacle of obstacles) {
          drawObstacle(context, map, obstacle, view);
        }
      }
      if (pose?.available && pose.x !== null && pose.y !== null && pose.yaw !== null) {
        drawRobot(context, map, pose, view);
      }
    } else {
      context.fillStyle = "#62748a";
      context.font = "600 16px 'Segoe UI', sans-serif";
      context.fillText("地图尚未加载", 24, 36);
    }
  }, [map, pose, obstacles, selectedGoal, activeGoal, view]);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) {
      return;
    }

    const handleWheel = (event: WheelEvent) => {
      if (!map?.loaded) {
        return;
      }
      event.preventDefault();
      event.stopPropagation();

      const currentView = viewRef.current;
      const nextZoom = clamp(currentView.zoom * (event.deltaY < 0 ? 1.1 : 0.9), 0.2, 12);
      const rect = canvas.getBoundingClientRect();
      const pointerX = event.clientX - rect.left;
      const pointerY = event.clientY - rect.top;
      const mapX = (pointerX - currentView.panX) / currentView.zoom;
      const mapY = (pointerY - currentView.panY) / currentView.zoom;
      setView({
        zoom: nextZoom,
        panX: pointerX - mapX * nextZoom,
        panY: pointerY - mapY * nextZoom,
      });
    };

    canvas.addEventListener("wheel", handleWheel, { passive: false });
    return () => {
      canvas.removeEventListener("wheel", handleWheel);
    };
  }, [map]);

  const handleMouseDown = (event: ReactMouseEvent<HTMLCanvasElement>) => {
    dragRef.current = {
      active: true,
      moved: false,
      startX: event.clientX,
      startY: event.clientY,
      panX: view.panX,
      panY: view.panY,
    };
  };

  const handleMouseMove = (event: ReactMouseEvent<HTMLCanvasElement>) => {
    if (map?.loaded) {
      const rect = event.currentTarget.getBoundingClientRect();
      const pixelX = (event.clientX - rect.left - view.panX) / view.zoom;
      const pixelY = (event.clientY - rect.top - view.panY) / view.zoom;
      if (pixelX >= 0 && pixelY >= 0 && pixelX <= map.width && pixelY <= map.height) {
        const world = mapPixelToWorld(map, { x: pixelX, y: pixelY });
        setHoverText(`x=${world.x.toFixed(2)} m, y=${world.y.toFixed(2)} m`);
      } else {
        setHoverText("光标不在地图范围内");
      }
    }

    if (!dragRef.current.active) {
      return;
    }

    const dx = event.clientX - dragRef.current.startX;
    const dy = event.clientY - dragRef.current.startY;
    if (Math.abs(dx) > 3 || Math.abs(dy) > 3) {
      dragRef.current.moved = true;
    }
    setView({
      zoom: view.zoom,
      panX: dragRef.current.panX + dx,
      panY: dragRef.current.panY + dy,
    });
  };

  const handleMouseUp = (event: ReactMouseEvent<HTMLCanvasElement>) => {
    if (!map?.loaded) {
      dragRef.current.active = false;
      return;
    }
    const wasClick = dragRef.current.active && !dragRef.current.moved;
    dragRef.current.active = false;
    if (!wasClick) {
      return;
    }

    const rect = event.currentTarget.getBoundingClientRect();
    const pixelX = (event.clientX - rect.left - view.panX) / view.zoom;
    const pixelY = (event.clientY - rect.top - view.panY) / view.zoom;
    if (pixelX < 0 || pixelY < 0 || pixelX > map.width || pixelY > map.height) {
      return;
    }
    const world = mapPixelToWorld(map, { x: pixelX, y: pixelY });
    onSelectGoal({
      x: world.x,
      y: world.y,
      yaw: pose?.yaw ?? 0,
      frame_id: "map",
    });
  };

  return (
    <div className="map-shell">
      <canvas
        ref={canvasRef}
        className={`map-canvas ${disabled ? "map-canvas-disabled" : ""}`}
        onMouseDown={handleMouseDown}
        onMouseMove={handleMouseMove}
        onMouseUp={handleMouseUp}
        onMouseLeave={() => {
          dragRef.current.active = false;
        }}
      />
      <div className="scene-card scene-card-left">
        <div className="scene-card-title">2D 栅格地图</div>
        <div className="scene-card-meta">{map?.frame_id ?? "frame=unknown"}</div>
        <div className="scene-card-meta">
          {map?.loaded ? `${map.width}x${map.height} @ ${map.resolution.toFixed(3)}m` : "等待地图"}
        </div>
      </div>
      <div className="scene-toolbar scene-toolbar-top-right">
        <button type="button" className="hud-button" onClick={() => zoomBy(1.18)}>
          放大
        </button>
        <button type="button" className="hud-button" onClick={() => zoomBy(0.84)}>
          缩小
        </button>
        <button type="button" className="hud-button" onClick={resetView}>
          复位
        </button>
      </div>
      <div className="map-overlay">
        <span>{hoverText}</span>
        <span>{selectedGoal ? `选点 ${selectedGoal.x.toFixed(2)}, ${selectedGoal.y.toFixed(2)}` : "单击地图选择目标/初始位姿"}</span>
        {disabled ? <span className="warning-chip">定位未就绪，禁止发送导航</span> : null}
      </div>
    </div>
  );
}

function drawRobot(
  context: CanvasRenderingContext2D,
  map: MapSnapshot,
  pose: RobotPose,
  view: ViewState,
) {
  if (pose.x === null || pose.y === null || pose.yaw === null) {
    return;
  }
  const point = worldToMapPixel(map, { x: pose.x, y: pose.y });
  const screenX = view.panX + point.x * view.zoom;
  const screenY = view.panY + point.y * view.zoom;
  const heading = -(pose.yaw - map.origin.yaw);
  const radius = Math.max(6, view.zoom * 0.8);
  const tipX = screenX + Math.cos(heading) * radius * 2.2;
  const tipY = screenY + Math.sin(heading) * radius * 2.2;

  context.fillStyle = "#0f172a";
  context.beginPath();
  context.arc(screenX, screenY, radius, 0, Math.PI * 2);
  context.fill();

  context.strokeStyle = "#f97316";
  context.lineWidth = 2;
  context.beginPath();
  context.moveTo(screenX, screenY);
  context.lineTo(tipX, tipY);
  context.stroke();
}

function drawGoal(
  context: CanvasRenderingContext2D,
  map: MapSnapshot,
  goal: NavigationGoal,
  view: ViewState,
  color: string,
) {
  const point = worldToMapPixel(map, { x: goal.x, y: goal.y });
  const screenX = view.panX + point.x * view.zoom;
  const screenY = view.panY + point.y * view.zoom;
  const heading = -(goal.yaw - map.origin.yaw);
  const radius = Math.max(7, view.zoom);
  const tipX = screenX + Math.cos(heading) * radius * 2;
  const tipY = screenY + Math.sin(heading) * radius * 2;

  context.strokeStyle = color;
  context.lineWidth = 2;
  context.beginPath();
  context.arc(screenX, screenY, radius, 0, Math.PI * 2);
  context.stroke();

  context.beginPath();
  context.moveTo(screenX - radius, screenY);
  context.lineTo(screenX + radius, screenY);
  context.moveTo(screenX, screenY - radius);
  context.lineTo(screenX, screenY + radius);
  context.moveTo(screenX, screenY);
  context.lineTo(tipX, tipY);
  context.stroke();
}

function drawObstacle(
  context: CanvasRenderingContext2D,
  map: MapSnapshot,
  obstacle: VirtualObstacleZone,
  view: ViewState,
) {
  const point = worldToMapPixel(map, { x: obstacle.x, y: obstacle.y });
  const screenX = view.panX + point.x * view.zoom;
  const screenY = view.panY + point.y * view.zoom;
  const radius = Math.max(6, (obstacle.radius / Math.max(map.resolution, 1e-6)) * view.zoom);

  context.fillStyle = "rgba(239, 68, 68, 0.18)";
  context.strokeStyle = "rgba(220, 38, 38, 0.88)";
  context.lineWidth = 2;
  context.beginPath();
  context.arc(screenX, screenY, radius, 0, Math.PI * 2);
  context.fill();
  context.stroke();

  context.fillStyle = "#991b1b";
  context.font = "600 11px 'Segoe UI', sans-serif";
  context.fillText(obstacle.label || obstacle.obstacle_id, screenX + radius + 6, screenY - 4);
}
