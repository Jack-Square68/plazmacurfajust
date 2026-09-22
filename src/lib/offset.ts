import * as ClipperNS from "clipper-lib";

import {
  add,
  cleanPoints,
  closestOnSegment,
  cross,
  dist,
  ensureCcw,
  midpoint,
  mul,
  pointInPolygon,
  raySegmentT,
  rotateLeft,
  signedArea,
  smoothClosedValues,
  sub,
  unit,
} from "./geometry";
import type { CapStyle, Compensated, JoinStyle, KerfParams, Point, Polyline } from "./types";

type IntPt = { X: number; Y: number };

type ClipperModule = {
  JoinType: { jtSquare: number; jtRound: number; jtMiter: number };
  EndType: {
    etOpenSquare: number;
    etOpenRound: number;
    etOpenButt: number;
    etClosedLine: number;
    etClosedPolygon: number;
  };
  PolyType: { ptSubject: number; ptClip: number };
  ClipType: { ctUnion: number };
  PolyFillType: { pftNonZero: number; pftEvenOdd: number };
  ClipperOffset: new (
    miterLimit?: number,
    arcTolerance?: number,
  ) => {
    AddPath(path: IntPt[], joinType: number, endType: number): void;
    Execute(solution: IntPt[][], delta: number): void;
  };
  Clipper: new (initOptions?: number) => {
    AddPath(path: IntPt[], polyType: number, closed: boolean): boolean;
    AddPaths(paths: IntPt[][], polyType: number, closed: boolean): boolean;
    Execute(
      clipType: number,
      solution: IntPt[][],
      subjFillType?: number,
      clipFillType?: number,
    ): boolean;
  };
};

const ClipperLib = ((ClipperNS as { default?: ClipperModule }).default ??
  (ClipperNS as unknown as ClipperModule)) as ClipperModule;

const SCALE = 10000;

function toPath(points: Point[]): IntPt[] {
  return points.map((p) => ({
    X: Math.round(p.x * SCALE),
    Y: Math.round(p.y * SCALE),
  }));
}

function fromPath(path: IntPt[]): Point[] {
  const pts = path.map((p) => ({ x: p.X / SCALE, y: p.Y / SCALE }));
  if (pts.length > 1) {
    const a = pts[0];
    const b = pts[pts.length - 1];
    if (Math.hypot(a.x - b.x, a.y - b.y) < 1 / SCALE) pts.pop();
  }
  return pts;
}

function joinType(join: JoinStyle): number {
  if (join === "round") return ClipperLib.JoinType.jtRound;
  if (join === "square") return ClipperLib.JoinType.jtSquare;
  return ClipperLib.JoinType.jtMiter;
}

function endType(args: {
  closed: boolean;
  closedAsLine: boolean;
  cap: CapStyle;
}): number {
  if (args.closed) {
    return args.closedAsLine
      ? ClipperLib.EndType.etClosedLine
      : ClipperLib.EndType.etClosedPolygon;
  }
  if (args.cap === "square") return ClipperLib.EndType.etOpenSquare;
  if (args.cap === "butt") return ClipperLib.EndType.etOpenButt;
  return ClipperLib.EndType.etOpenRound;
}

export function offsetPath(
  points: Point[],
  delta: number,
  options: {
    closed: boolean;
    closedAsLine: boolean;
    join: JoinStyle;
    cap: CapStyle;
  },
): Point[][] {
  const cleaned = cleanPoints(points, options.closed);
  const minCount = options.closed ? 3 : 2;
  if (cleaned.length < minCount || Math.abs(delta) < 1e-9) return [];

  const path = options.closed ? ensureCcw(cleaned) : cleaned;
  const co = new ClipperLib.ClipperOffset(2, 0.012 * SCALE);
  co.AddPath(
    toPath(path),
    joinType(options.join),
    endType({
      closed: options.closed,
      closedAsLine: options.closedAsLine,
      cap: options.cap,
    }),
  );
  const solution: IntPt[][] = [];
  co.Execute(solution, delta * SCALE);
  return solution.map(fromPath).filter((p) => p.length >= 2);
}

export function unionPaths(polygons: Point[][]): Point[][] {
  const closed = polygons
    .map((pts) => ensureCcw(cleanPoints(pts, true)))
    .filter((pts) => pts.length >= 3);
  if (closed.length === 0) return [];

  const clipper = new ClipperLib.Clipper(2);
  clipper.AddPaths(
    closed.map(toPath),
    ClipperLib.PolyType.ptSubject,
    true,
  );
  const solution: IntPt[][] = [];
  clipper.Execute(
    ClipperLib.ClipType.ctUnion,
    solution,
    ClipperLib.PolyFillType.pftNonZero,
    ClipperLib.PolyFillType.pftNonZero,
  );
  const out = solution.map(fromPath).filter((p) => p.length >= 3);
  return out.length ? out : closed;
}

type WidthSample = {
  point: Point;
  inward: Point;
  width: number;
  edge: number;
};

function sampleBoundary(points: Point[], spacing: number): Omit<WidthSample, "width">[] {
  const ring = ensureCcw(points);
  const samples: Omit<WidthSample, "width">[] = [];
  for (let i = 0; i < ring.length; i++) {
    const a = ring[i];
    const b = ring[(i + 1) % ring.length];
    const length = dist(a, b);
    if (length < 1e-9) continue;
    const tangent = unit({ x: b.x - a.x, y: b.y - a.y });
    const inward = rotateLeft(tangent);
    const steps = Math.max(1, Math.ceil(length / spacing));
    for (let k = 0; k < steps; k++) {
      const t = k / steps;
      samples.push({
        point: { x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t },
        inward,
        edge: i,
      });
    }
  }
  return samples;
}

function localWidth(origin: Point, inward: Point, ring: Point[], skipEdge: number): number {
  const n = ring.length;
  let best = Infinity;
  for (let i = 0; i < n; i++) {
    const wrap = Math.min(Math.abs(i - skipEdge), n - Math.abs(i - skipEdge));
    if (wrap <= 1) continue;
    const hit = raySegmentT(origin, inward, ring[i], ring[(i + 1) % n]);
    if (hit !== null && hit < best) best = hit;
    const close = closestOnSegment(origin, ring[i], ring[(i + 1) % n]);
    const gap = dist(origin, close);
    if (gap >= best || gap < 1e-4) continue;
    const mid = midpoint(origin, close);
    if (pointInPolygon(mid, ring) && gap < best) best = gap;
  }
  return best;
}

function thinRuns(flags: boolean[]): number {
  if (!flags.some(Boolean)) return 0;
  let runs = 0;
  let inRun = false;
  for (const flag of flags) {
    if (flag && !inRun) {
      runs += 1;
      inRun = true;
    } else if (!flag) {
      inRun = false;
    }
  }
  if (flags[0] && flags[flags.length - 1] && runs >= 2) runs -= 1;
  return runs;
}

function exteriorArc(center: Point, from: Point, to: Point, radius: number, ring: Point[]): Point[] {
  const a0 = Math.atan2(from.y - center.y, from.x - center.x);
  const a1 = Math.atan2(to.y - center.y, to.x - center.x);
  let da = a1 - a0;
  while (da <= -Math.PI) da += Math.PI * 2;
  while (da > Math.PI) da -= Math.PI * 2;
  const midA = a0 + da / 2;
  const mid = { x: center.x + Math.cos(midA) * radius, y: center.y + Math.sin(midA) * radius };
  if (pointInPolygon(mid, ring)) da = da > 0 ? da - Math.PI * 2 : da + Math.PI * 2;
  const steps = Math.max(6, Math.ceil((Math.abs(da) * radius) / 0.12));
  const pts: Point[] = [];
  for (let i = 1; i < steps; i++) {
    const a = a0 + (da * i) / steps;
    pts.push({ x: center.x + Math.cos(a) * radius, y: center.y + Math.sin(a) * radius });
  }
  return pts;
}

function offsetPoint(sample: WidthSample, delta: number, ring: Point[]): Point {
  if (delta <= 1e-6) return sample.point;
  let outward = mul(sample.inward, -1);
  const probe = add(sample.point, mul(outward, Math.min(0.2, delta)));
  if (pointInPolygon(probe, ring)) outward = mul(outward, -1);
  return add(sample.point, mul(outward, delta));
}

/**
 * Parallel-offset the original closed curve, but only where local width is
 * under minWidth. Wide walls stay on the input; thin stretches move out
 * along the original normals with a smoothed distance so the result is a
 * fair offset of the same curve, not a new capsule.
 */
export function ensureMinWidth(
  points: Point[],
  minWidth: number,
  _join: JoinStyle = "round",
): { outline: Point[][]; pinches: number; centerlines: Point[][] } {
  const ring = ensureCcw(cleanPoints(points, true));
  if (ring.length < 3 || minWidth <= 0) {
    return { outline: ring.length >= 3 ? [ring] : [], pinches: 0, centerlines: [] };
  }

  const spacing = Math.min(0.4, Math.max(0.16, minWidth / 24));
  const samples: WidthSample[] = sampleBoundary(ring, spacing).map((s) => ({
    ...s,
    width: localWidth(s.point, s.inward, ring, s.edge),
  }));
  if (samples.length < 3) return { outline: [ring], pinches: 0, centerlines: [] };

  const rawDelta = samples.map((s) =>
    Number.isFinite(s.width) ? Math.max(0, (minWidth - s.width) / 2) : 0,
  );
  if (!rawDelta.some((d) => d > 1e-4)) {
    return { outline: [ring], pinches: 0, centerlines: [] };
  }

  const sigma = Math.max(1.2, (minWidth * 0.55) / spacing);
  const deltas = smoothClosedValues(rawDelta, sigma);
  const pinches = thinRuns(deltas.map((d) => d > 0.04));

  const moved: Point[] = [];
  for (let i = 0; i < samples.length; i++) {
    const prev = samples[(i - 1 + samples.length) % samples.length];
    const curr = samples[i];
    const next = samples[(i + 1) % samples.length];
    const t1 = unit(sub(curr.point, prev.point));
    const t2 = unit(sub(next.point, curr.point));
    const turn = Math.atan2(cross(t1, t2), t1.x * t2.x + t1.y * t2.y);
    const delta = deltas[i];
    const pOff = offsetPoint(curr, delta, ring);

    const isCorner = Math.abs(turn) > 0.35 && delta > 0.05;
    if (isCorner && moved.length) {
      const from = offsetPoint({ ...curr, inward: prev.inward }, delta, ring);
      const to = offsetPoint(curr, delta, ring);
      if (dist(from, to) > 0.08) {
        moved.push(from);
        moved.push(...exteriorArc(curr.point, from, to, delta, ring));
      }
    }
    moved.push(pOff);
  }

  const outline = unionPaths([cleanPoints(moved, true, 0.02)]);
  const centerlines: Point[][] = [];
  const thin = samples
    .map((s, i) => (deltas[i] > 0.04 ? add(s.point, mul(s.inward, s.width * 0.5)) : null))
    .filter((p): p is Point => p !== null);
  if (thin.length >= 2) centerlines.push(cleanPoints(thin, false, 0.08));

  return {
    outline: outline.length ? outline : [ring],
    pinches,
    centerlines,
  };
}

function insetPolygons(
  polygons: Point[][],
  delta: number,
  join: JoinStyle,
  cap: CapStyle,
): Point[][] {
  if (delta <= 1e-9) return polygons;
  const out: Point[][] = [];
  for (const poly of polygons) {
    out.push(
      ...offsetPath(poly, -delta, {
        closed: true,
        closedAsLine: false,
        join,
        cap,
      }),
    );
  }
  return out;
}

function compensateClosedSlot(cleaned: Point[], params: KerfParams): Compensated {
  const minWidth = params.minWidth;
  if (minWidth <= 0) {
    return { outline: [], toolpath: [], singlePass: false, error: "Minimum width must be greater than 0." };
  }

  const area = Math.abs(signedArea(cleaned));
  const tooThinToBeARegion = area < minWidth * minWidth * 0.2;
  if (tooThinToBeARegion) {
    return compensateCenterline(cleaned, true, params);
  }

  const grown = ensureMinWidth(cleaned, minWidth, params.join);
  const outline = grown.outline;
  if (outline.length === 0) {
    return {
      outline: [],
      toolpath: [],
      singlePass: false,
      error: "Offset failed. Try a smaller width or simplify the curve.",
      pinches: grown.pinches,
    };
  }

  const halfKerf = params.kerf / 2;
  if (halfKerf <= 0.02) {
    return { outline, toolpath: outline, singlePass: true, pinches: grown.pinches };
  }

  const inset = insetPolygons(outline, halfKerf, params.join, params.cap);
  if (minWidth > params.kerf + 0.04) {
    if (inset.length === 0) {
      return { outline, toolpath: outline, singlePass: true, pinches: grown.pinches };
    }
    return { outline, toolpath: inset, singlePass: false, pinches: grown.pinches };
  }

  const centerlines = grown.centerlines.filter((l) => l.length >= 2);
  const toolpath = [...inset, ...centerlines];
  if (toolpath.length === 0) {
    return { outline, toolpath: outline, singlePass: true, pinches: grown.pinches };
  }
  return {
    outline,
    toolpath,
    singlePass: inset.length === 0,
    pinches: grown.pinches,
  };
}

function compensateCenterline(
  cleaned: Point[],
  closed: boolean,
  params: KerfParams,
): Compensated {
  const outlineDelta = params.minWidth / 2;
  const toolDelta = (params.minWidth - params.kerf) / 2;
  const common = {
    closed,
    closedAsLine: closed,
    join: params.join,
    cap: params.cap,
  };

  if (outlineDelta <= 0) {
    return { outline: [], toolpath: [], singlePass: false, error: "Minimum width must be greater than 0." };
  }

  const outline = offsetPath(cleaned, outlineDelta, common);
  if (outline.length === 0) {
    return {
      outline: [],
      toolpath: [],
      singlePass: false,
      error: "Offset failed. Try a smaller width or simplify the curve.",
    };
  }

  if (toolDelta > 0.02) {
    return {
      outline,
      toolpath: offsetPath(cleaned, toolDelta, common),
      singlePass: false,
    };
  }

  return {
    outline,
    toolpath: [cleaned],
    singlePass: true,
  };
}

export function compensateCurve(curve: Polyline, params: KerfParams): Compensated {
  const cleaned = cleanPoints(curve.points, curve.closed);
  if (cleaned.length < 2) {
    return { outline: [], toolpath: [], singlePass: false, error: "Curve needs at least two points." };
  }

  if (params.mode === "slot") {
    if (curve.closed && cleaned.length >= 3) {
      return compensateClosedSlot(cleaned, params);
    }
    return compensateCenterline(cleaned, curve.closed, params);
  }

  if (!curve.closed) {
    return {
      outline: [],
      toolpath: [],
      singlePass: false,
      error: "Part and hole compensation need a closed curve.",
    };
  }

  const halfKerf = params.kerf / 2;
  if (halfKerf <= 0) {
    return {
      outline: [cleaned],
      toolpath: [cleaned],
      singlePass: true,
    };
  }

  const polyOpts = {
    closed: true,
    closedAsLine: false,
    join: params.join,
    cap: params.cap,
  };

  const delta = params.mode === "part" ? halfKerf : -halfKerf;
  const toolpath = offsetPath(cleaned, delta, polyOpts);
  if (toolpath.length === 0) {
    return {
      outline: [cleaned],
      toolpath: [],
      singlePass: false,
      error:
        params.mode === "hole"
          ? "Hole is smaller than the kerf. Enlarge the opening or lower kerf width."
          : "Could not offset this profile. Check that it is a simple closed curve.",
    };
  }

  return {
    outline: [cleaned],
    toolpath,
    singlePass: false,
  };
}
