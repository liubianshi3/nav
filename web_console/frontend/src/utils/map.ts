import type { MapSnapshot } from "../types";

export interface Point2D {
  x: number;
  y: number;
}

export function clamp(value: number, min: number, max: number): number {
  return Math.min(max, Math.max(min, value));
}

export function worldToMapPixel(map: MapSnapshot, world: Point2D): Point2D {
  const dx = world.x - map.origin.x;
  const dy = world.y - map.origin.y;
  const cos = Math.cos(-map.origin.yaw);
  const sin = Math.sin(-map.origin.yaw);
  const localX = cos * dx - sin * dy;
  const localY = sin * dx + cos * dy;
  return {
    x: localX / map.resolution,
    y: map.height - localY / map.resolution,
  };
}

export function mapPixelToWorld(map: MapSnapshot, pixel: Point2D): Point2D {
  const localX = pixel.x * map.resolution;
  const localY = (map.height - pixel.y) * map.resolution;
  const cos = Math.cos(map.origin.yaw);
  const sin = Math.sin(map.origin.yaw);
  return {
    x: map.origin.x + cos * localX - sin * localY,
    y: map.origin.y + sin * localX + cos * localY,
  };
}
