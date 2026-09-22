import type { Point, Polyline } from "./types";

export function dist(a: Point, b: Point): number {
  return Math.hypot(a.x - b.x, a.y - b.y);
}

export function sub(a: Point, b: Point): Point {
  return { x: a.x - b.x, y: a.y - b.y };
}

export function add(a: Point, b: Point): Point {
  return { x: a.x + b.x, y: a.y + b.y };
}

export function mul(a: Point, s: number): Point {
  return { x: a.x * s, y: a.y * s };
}

export function midpoint(a: Point, b: Point): Point {
  return { x: (a.x + b.x) / 2, y: (a.y + b.y) / 2 };
}

export function unit(a: Point): Point {
  const L = Math.hypot(a.x, a.y);
  if (L < 1e-12) return { x: 0, y: 0 };
  return { x: a.x / L, y: a.y / L };
}

export function cross(a: Point, b: Point): number {
  return a.x * b.y - a.y * b.x;
}

export function rotateLeft(tangent: Point): Point {
  return { x: -tangent.y, y: tangent.x };
}

/** Distance along a ray `origin + t * dir` to segment ab, or null if no hit. */
export function raySegmentT(origin: Point, dir: Point, a: Point, b: Point): number | null {
  const seg = sub(b, a);
  const det = cross(dir, seg);
  if (Math.abs(det) < 1e-12) return null;
  const ao = sub(a, origin);
  const t = cross(ao, seg) / det;
  const u = cross(ao, dir) / det;
  if (t > 1e-4 && u >= -1e-6 && u <= 1 + 1e-6) return t;
  return null;
}

export function cleanPoints(points: Point[], closed: boolean, eps = 0.001): Point[] {
  if (points.length === 0) return [];
  const out: Point[] = [points[0]];
  for (let i = 1; i < points.length; i++) {
    if (dist(points[i], out[out.length - 1]) > eps) out.push(points[i]);
  }
  if (closed && out.length > 2 && dist(out[0], out[out.length - 1]) <= eps) {
    out.pop();
  }
  return out;
}

export function polylineLength(points: Point[], closed: boolean): number {
  if (points.length < 2) return 0;
  let L = 0;
  for (let i = 1; i < points.length; i++) L += dist(points[i - 1], points[i]);
  if (closed && points.length > 2) L += dist(points[points.length - 1], points[0]);
  return L;
}

export function boundsOf(points: Point[]): {
  minX: number;
  minY: number;
  maxX: number;
  maxY: number;
} | null {
  if (points.length === 0) return null;
  let minX = points[0].x;
  let minY = points[0].y;
  let maxX = points[0].x;
  let maxY = points[0].y;
  for (const p of points) {
    minX = Math.min(minX, p.x);
    minY = Math.min(minY, p.y);
    maxX = Math.max(maxX, p.x);
    maxY = Math.max(maxY, p.y);
  }
  return { minX, minY, maxX, maxY };
}

export function boundsOfPolylines(curves: Polyline[]): {
  minX: number;
  minY: number;
  maxX: number;
  maxY: number;
} | null {
  const pts = curves.flatMap((c) => c.points);
  return boundsOf(pts);
}

export function distToSegment(p: Point, a: Point, b: Point): number {
  const ab = sub(b, a);
  const ap = sub(p, a);
  const ab2 = ab.x * ab.x + ab.y * ab.y;
  if (ab2 < 1e-18) return dist(p, a);
  const t = Math.max(0, Math.min(1, (ap.x * ab.x + ap.y * ab.y) / ab2));
  return dist(p, { x: a.x + ab.x * t, y: a.y + ab.y * t });
}

export function distToPolyline(p: Point, curve: Polyline): number {
  const pts = curve.points;
  if (pts.length === 0) return Infinity;
  if (pts.length === 1) return dist(p, pts[0]);
  let best = Infinity;
  const last = curve.closed ? pts.length : pts.length - 1;
  for (let i = 0; i < last; i++) {
    const a = pts[i];
    const b = pts[(i + 1) % pts.length];
    best = Math.min(best, distToSegment(p, a, b));
  }
  return best;
}

export function pointInPolygon(p: Point, pts: Point[]): boolean {
  if (pts.length < 3) return false;
  let inside = false;
  for (let i = 0, j = pts.length - 1; i < pts.length; j = i++) {
    const a = pts[i];
    const b = pts[j];
    const intersect =
      a.y > p.y !== b.y > p.y &&
      p.x < ((b.x - a.x) * (p.y - a.y)) / ((b.y - a.y) || Number.EPSILON) + a.x;
    if (intersect) inside = !inside;
  }
  return inside;
}

export function centroid(points: Point[]): Point {
  if (points.length === 0) return { x: 0, y: 0 };
  if (points.length < 3) {
    return {
      x: points.reduce((s, p) => s + p.x, 0) / points.length,
      y: points.reduce((s, p) => s + p.y, 0) / points.length,
    };
  }
  let x = 0;
  let y = 0;
  let area = 0;
  for (let i = 0; i < points.length; i++) {
    const a = points[i];
    const b = points[(i + 1) % points.length];
    const cross = a.x * b.y - b.x * a.y;
    area += cross;
    x += (a.x + b.x) * cross;
    y += (a.y + b.y) * cross;
  }
  area *= 0.5;
  if (Math.abs(area) < 1e-9) {
    return {
      x: points.reduce((s, p) => s + p.x, 0) / points.length,
      y: points.reduce((s, p) => s + p.y, 0) / points.length,
    };
  }
  return { x: x / (6 * area), y: y / (6 * area) };
}

export function hitTestCurves(
  p: Point,
  curves: Polyline[],
  threshold: number,
): string | null {
  let bestId: string | null = null;
  let best = threshold;
  for (const curve of curves) {
    const d = distToPolyline(p, curve);
    if (d < best) {
      best = d;
      bestId = curve.id;
    }
  }
  if (bestId) return bestId;

  let bestArea = Infinity;
  for (const curve of curves) {
    if (!curve.closed || curve.points.length < 3) continue;
    if (!pointInPolygon(p, curve.points)) continue;
    const area = Math.abs(signedArea(curve.points));
    if (area < bestArea) {
      bestArea = area;
      bestId = curve.id;
    }
  }
  return bestId;
}

export function hitTestVertex(
  p: Point,
  curve: Polyline,
  threshold: number,
): number | null {
  let best = threshold;
  let idx: number | null = null;
  curve.points.forEach((pt, i) => {
    const d = dist(p, pt);
    if (d < best) {
      best = d;
      idx = i;
    }
  });
  return idx;
}

export function translateCurve(curve: Polyline, dx: number, dy: number): Polyline {
  return {
    ...curve,
    points: curve.points.map((p) => ({ x: p.x + dx, y: p.y + dy })),
  };
}

export function moveVertex(
  curve: Polyline,
  index: number,
  point: Point,
): Polyline {
  const points = curve.points.map((p, i) => (i === index ? point : p));
  return { ...curve, points };
}

export function signedArea(points: Point[]): number {
  let a = 0;
  for (let i = 0; i < points.length; i++) {
    const p = points[i];
    const q = points[(i + 1) % points.length];
    a += p.x * q.y - q.x * p.y;
  }
  return a / 2;
}

export function ensureCcw(points: Point[]): Point[] {
  return signedArea(points) < 0 ? [...points].reverse() : points;
}

export function sampleAt(points: Point[], closed: boolean, t: number): {
  point: Point;
  tangent: Point;
} | null {
  const L = polylineLength(points, closed);
  if (L < 1e-9 || points.length < 2) return null;
  let remain = ((t % 1) + 1) % 1 * L;
  const count = closed ? points.length : points.length - 1;
  for (let i = 0; i < count; i++) {
    const a = points[i];
    const b = points[(i + 1) % points.length];
    const seg = dist(a, b);
    if (remain <= seg || i === count - 1) {
      const u = seg < 1e-12 ? 0 : remain / seg;
      return {
        point: { x: a.x + (b.x - a.x) * u, y: a.y + (b.y - a.y) * u },
        tangent: unit(sub(b, a)),
      };
    }
    remain -= seg;
  }
  return null;
}

export function uid(prefix: string): string {
  return `${prefix}-${Math.random().toString(36).slice(2, 9)}`;
}

export function formatMm(n: number): string {
  const rounded = Math.round(n * 100) / 100;
  return Number.isInteger(rounded) ? `${rounded}` : rounded.toFixed(2);
}

/** Moving-average a closed or open polyline so offsets follow a fair curve. */
export function smoothPolyline(points: Point[], closed: boolean, passes = 2): Point[] {
  if (points.length < 3) return points;
  let curr = points;
  for (let pass = 0; pass < passes; pass++) {
    const next: Point[] = [];
    for (let i = 0; i < curr.length; i++) {
      if (!closed && (i === 0 || i === curr.length - 1)) {
        next.push(curr[i]);
        continue;
      }
      const prev = curr[(i - 1 + curr.length) % curr.length];
      const mid = curr[i];
      const nxt = curr[(i + 1) % curr.length];
      next.push({
        x: (prev.x + mid.x * 2 + nxt.x) / 4,
        y: (prev.y + mid.y * 2 + nxt.y) / 4,
      });
    }
    curr = next;
  }
  return curr;
}

