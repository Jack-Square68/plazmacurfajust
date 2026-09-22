import { boundsOfPolylines } from "./geometry";
import type { Point, Polyline } from "./types";

export type Camera = {
  scale: number;
  ox: number;
  oy: number;
};

export function worldToScreen(camera: Camera, p: Point): Point {
  return {
    x: p.x * camera.scale + camera.ox,
    y: -p.y * camera.scale + camera.oy,
  };
}

export function screenToWorld(camera: Camera, p: Point): Point {
  return {
    x: (p.x - camera.ox) / camera.scale,
    y: -(p.y - camera.oy) / camera.scale,
  };
}

export function zoomAt(camera: Camera, screen: Point, factor: number): Camera {
  const world = screenToWorld(camera, screen);
  const scale = Math.min(80, Math.max(0.2, camera.scale * factor));
  return {
    scale,
    ox: screen.x - world.x * scale,
    oy: screen.y + world.y * scale,
  };
}

export function pan(camera: Camera, dx: number, dy: number): Camera {
  return { ...camera, ox: camera.ox + dx, oy: camera.oy + dy };
}

export function fitCurves(
  curves: Polyline[],
  width: number,
  height: number,
  padding = 48,
): Camera {
  const b = boundsOfPolylines(curves);
  if (!b || width < 10 || height < 10) {
    return { scale: 2.4, ox: 40, oy: height - 40 };
  }
  const bw = Math.max(1, b.maxX - b.minX);
  const bh = Math.max(1, b.maxY - b.minY);
  const scale = Math.min((width - padding * 2) / bw, (height - padding * 2) / bh);
  const ox = (width - bw * scale) / 2 - b.minX * scale;
  const oy = (height + bh * scale) / 2 + b.minY * scale;
  return { scale, ox, oy };
}
