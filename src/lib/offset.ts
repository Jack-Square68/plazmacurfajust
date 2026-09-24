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
  polylineLength,
  raySegmentT,
  rotateLeft,
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

function edgePrefix(ring: Point[]): number[] {
  const pref = [0];
  for (let i = 0; i < ring.length; i++) {
    pref.push(pref[i] + dist(ring[i], ring[(i + 1) % ring.length]));
  }
  return pref;
}

function directedArc(prefix: number[], i: number, j: number): number {
  const n = prefix.length - 1;
  if (n <= 0 || i === j) return 0;
  return i < j ? prefix[j] - prefix[i] : prefix[n] - prefix[i] + prefix[j];
}

function alongFromOrigin(
  ring: Point[],
  prefix: number[],
  origin: Point,
  skipEdge: number,
  hitEdge: number,
): number {
  const n = ring.length;
  if (n <= 0) return 0;
  const edgeLen = prefix[skipEdge + 1] - prefix[skipEdge];
  let t = dist(ring[skipEdge], origin);
  if (t > edgeLen) t = edgeLen;
  const rem = edgeLen - t;
  if (skipEdge === hitEdge) return Math.min(t, rem);
  const fwd = rem + directedArc(prefix, (skipEdge + 1) % n, hitEdge);
  const back = t + directedArc(prefix, (hitEdge + 1) % n, skipEdge);
  return Math.min(fwd, back);
}

function localWidth(
  origin: Point,
  inward: Point,
  ring: Point[],
  skipEdge: number,
  minWidth = 0,
  prefix?: number[],
): number {
  const n = ring.length;
  const pref = prefix ?? edgePrefix(ring);
  const tangent = { x: inward.y, y: -inward.x };
  const skipAlong = Math.max(minWidth * 2, 8);
  let best = Infinity;
  for (let i = 0; i < n; i++) {
    const wrap = Math.min(Math.abs(i - skipEdge), n - Math.abs(i - skipEdge));
    if (wrap <= 1) continue;
    const hitTan = unit(sub(ring[(i + 1) % n], ring[i]));
    const parallel = Math.abs(hitTan.x * tangent.x + hitTan.y * tangent.y) >= 0.82;
    const nearby = alongFromOrigin(ring, pref, origin, skipEdge, i) < skipAlong;
    if (nearby && !parallel) continue;
    const hit = raySegmentT(origin, inward, ring[i], ring[(i + 1) % n]);
    if (hit !== null && hit < best) best = hit;
    const close = closestOnSegment(origin, ring[i], ring[(i + 1) % n]);
    const gap = dist(origin, close);
    if (gap >= best || gap < 1e-4) continue;
    const toClose = sub(close, origin);
    if (Math.abs(toClose.x * tangent.x + toClose.y * tangent.y) > 0.85 * gap) continue;
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

function wrapIndex(i: number, n: number): number {
  return ((i % n) + n) % n;
}

function turnAt(a: Point, b: Point, c: Point): number {
  const t1 = unit(sub(b, a));
  const t2 = unit(sub(c, b));
  return Math.atan2(cross(t1, t2), t1.x * t2.x + t1.y * t2.y);
}

function nearestIndex(points: Point[], target: Point): number {
  let best = 0;
  let bestD = Infinity;
  for (let i = 0; i < points.length; i++) {
    const d = dist(points[i], target);
    if (d < bestD) {
      bestD = d;
      best = i;
    }
  }
  return best;
}

function sampleWidthNear(point: Point, samples: WidthSample[], fallback = 1e9): number {
  if (!samples.length) return fallback;
  const i = nearestIndex(
    samples.map((s) => s.point),
    point,
  );
  const width = samples[i]?.width;
  return Number.isFinite(width) ? width : fallback;
}

function finiteWidthNear(point: Point, samples: WidthSample[], fallback = 1e9, radius = 3): number {
  let best = fallback;
  for (const sample of samples) {
    if (!Number.isFinite(sample.width) || sample.width >= 1e8) continue;
    if (dist(sample.point, point) <= radius && sample.width < best) best = sample.width;
  }
  return best < fallback ? best : sampleWidthNear(point, samples, fallback);
}

function isTaperedEnd(center: Point, samples: WidthSample[], minWidth = 6): boolean {
  const w0 = finiteWidthNear(center, samples);
  if (w0 >= 1e8 || w0 > minWidth * 0.85) return false;
  const near: number[] = [];
  const far: number[] = [];
  for (const sample of samples) {
    if (!Number.isFinite(sample.width) || sample.width >= 1e8) continue;
    const d = dist(sample.point, center);
    if (d >= 3.5 && d <= 8) near.push(sample.width);
    else if (d >= 10 && d <= 18) far.push(sample.width);
  }
  if (!near.length || !far.length) return false;
  near.sort((a, b) => a - b);
  far.sort((a, b) => a - b);
  return far[Math.floor(far.length / 2)] > near[Math.floor(near.length / 2)] * 1.2 + 0.25;
}

function landingGrow(center: Point, samples: WidthSample[], deltas: number[]): number {
  const vals: number[] = [];
  for (let i = 0; i < samples.length; i++) {
    const d = dist(samples[i].point, center);
    if (d < 3 || d > 8) continue;
    if (i < deltas.length) vals.push(deltas[i]);
  }
  if (!vals.length) {
    const i = nearestIndex(
      samples.map((s) => s.point),
      center,
    );
    return i < deltas.length ? Math.max(deltas[i], 0) : 0;
  }
  vals.sort((a, b) => a - b);
  return Math.max(vals[Math.floor(vals.length / 2)], 0);
}

function findTaperedTips(ring: Point[], samples: WidthSample[], minWidth: number): Point[] {
  const tips: Point[] = [];
  const n = ring.length;
  if (n < 6 || !samples.length) return tips;
  const addTip = (p: Point) => {
    if (tips.some((t) => dist(t, p) < 5)) return;
    tips.push(p);
  };
  for (let i = 0; i < n; i++) {
    const a = ring[wrapIndex(i - 1, n)];
    const b = ring[i];
    const c = ring[wrapIndex(i + 1, n)];
    const t1 = unit(sub(b, a));
    const t2 = unit(sub(c, b));
    const turn = Math.abs(turnAt(a, b, c));
    const opposite = t1.x * t2.x + t1.y * t2.y < -0.85;
    if (opposite && turn > 2.2 && isTaperedEnd(b, samples, minWidth) && sampleWidthNear(b, samples) < minWidth * 0.9) {
      addTip(b);
    }
    const j = wrapIndex(i + 1, n);
    const edge = dist(b, ring[j]);
    if (edge < 0.04 || edge > minWidth * 0.55) continue;
    const turnB = Math.abs(turnAt(a, b, ring[j]));
    const turnJ = Math.abs(turnAt(b, ring[j], ring[wrapIndex(j + 1, n)]));
    if (turnB < 0.7 || turnJ < 0.7) continue;
    const mid = midpoint(b, ring[j]);
    const width = Math.min(
      finiteWidthNear(mid, samples),
      finiteWidthNear(b, samples),
      finiteWidthNear(ring[j], samples),
    );
    if (width > minWidth * 0.9) continue;
    if (
      !isTaperedEnd(mid, samples, minWidth)
      && !isTaperedEnd(b, samples, minWidth)
      && !isTaperedEnd(ring[j], samples, minWidth)
    ) {
      continue;
    }
    addTip(mid);
  }
  return tips;
}

function wallOutward(a: Point, b: Point, ring: Point[]): Point {
  const tangent = unit(sub(b, a));
  let inward = rotateLeft(tangent);
  const mid = midpoint(a, b);
  if (!pointInPolygon(add(mid, mul(inward, 0.35)), ring)) inward = mul(inward, -1);
  return mul(inward, -1);
}

function vertexRoundJoin(a: Point, b: Point, c: Point, radius: number, ring: Point[]): Point[] {
  if (radius < 0.04) return [b];
  const p0 = add(b, mul(wallOutward(a, b, ring), radius));
  const p1 = add(b, mul(wallOutward(b, c, ring), radius));
  if (dist(p0, p1) < 0.04) return [p0];
  return [p0, ...exteriorArc(b, p0, p1, radius, ring), p1];
}

function parallelTipChain(
  center: Point,
  ring: Point[],
  samples: WidthSample[],
  deltas: number[],
  _moved: Point[],
): Point[] {
  const n = ring.length;
  if (n < 6) return [];
  const i0 = nearestIndex(ring, center);
  const g = landingGrow(ring[i0], samples, deltas);
  if (g < 0.04) return [];
  let ia = -1;
  let ib = -1;
  let best = Infinity;
  for (let k = -10; k <= 10; k++) {
    const i = wrapIndex(i0 + k, n);
    const j = wrapIndex(i + 1, n);
    const edge = dist(ring[i], ring[j]);
    if (edge < 0.04 || edge > 4) continue;
    const turnI = Math.abs(turnAt(ring[wrapIndex(i - 1, n)], ring[i], ring[j]));
    const turnJ = Math.abs(turnAt(ring[i], ring[j], ring[wrapIndex(j + 1, n)]));
    if (turnI < 0.6 || turnJ < 0.6) continue;
    if (edge < best) {
      best = edge;
      ia = i;
      ib = j;
    }
  }
  if (ia < 0) return [];
  const left = vertexRoundJoin(ring[wrapIndex(ia - 1, n)], ring[ia], ring[ib], g, ring);
  const right = vertexRoundJoin(ring[ia], ring[ib], ring[wrapIndex(ib + 1, n)], g, ring);
  if (left.length < 2 || right.length < 2) return [];
  return cleanPoints([...left, ...right.slice(1)], false, 0.02);
}

function replaceSpan(moved: Point[], start: number, end: number, chain: Point[]): Point[] {
  const n = moved.length;
  const before = moved[(start - 1 + n) % n];
  const after = moved[(end + 1) % n];
  const fwd = dist(before, chain[0]) + dist(chain[chain.length - 1], after);
  const rev = dist(before, chain[chain.length - 1]) + dist(chain[0], after);
  const used = rev < fwd ? [...chain].reverse() : chain;
  if (start <= end) return [...moved.slice(0, start), ...used, ...moved.slice(end + 1)];
  return [...used, ...moved.slice(end + 1, start)];
}

function longestFlagRun(flags: boolean[]): [number, number] | null {
  const n = flags.length;
  if (!n || !flags.some(Boolean)) return null;
  if (flags.every(Boolean)) return [0, n - 1];
  let bestStart = 0;
  let bestEnd = 0;
  let bestLen = 0;
  for (let i = 0; i < n; i++) {
    if (!flags[i] || flags[(i - 1 + n) % n]) continue;
    let len = 0;
    let j = i;
    do {
      len += 1;
      j = (j + 1) % n;
    } while (j !== i && flags[j]);
    if (len > bestLen) {
      bestLen = len;
      bestStart = i;
      bestEnd = (j - 1 + n) % n;
    }
  }
  return bestLen > 0 ? [bestStart, bestEnd] : null;
}

function spliceNearVertex(moved: Point[], vertex: Point, chain: Point[], radius: number): Point[] {
  const n = moved.length;
  if (n < 4 || chain.length < 2) return moved;
  const thresh = Math.max(radius * 1.8, 3.2);
  const flags = moved.map((p) => dist(p, vertex) < thresh);
  const run = longestFlagRun(flags);
  if (!run) return moved;
  const [start, end] = run;
  const runLen = start <= end ? end - start + 1 : n - start + end + 1;
  if (runLen < 1 || runLen > Math.max(8, Math.floor(n * 0.35))) return moved;
  return replaceSpan(moved, start, end, chain);
}

function fixTaperedTips(
  moved: Point[],
  ring: Point[],
  samples: WidthSample[],
  deltas: number[],
  minWidth: number,
): Point[] {
  const tips = findTaperedTips(ring, samples, minWidth);
  let next = moved;
  for (const tip of tips) {
    const chain = parallelTipChain(tip, ring, samples, deltas, next);
    if (chain.length < 2) continue;
    const g = Math.max(landingGrow(tip, samples, deltas), 2.4);
    if (Math.min(...chain.map((p) => dist(p, tip))) > g + 2.2) continue;
    const spliced = spliceNearVertex(next, tip, chain, Math.max(g, 2.8));
    if (spliced.length >= 3) next = spliced;
  }
  return next;
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

  const perimeter = polylineLength(ring, true);
  const target = Math.min(0.4, Math.max(0.16, minWidth / 24));
  const spacing = perimeter > 720 * target ? perimeter / 720 : target;
  const prefix = edgePrefix(ring);
  const samples: WidthSample[] = sampleBoundary(ring, spacing).map((s) => ({
    ...s,
    width: localWidth(s.point, s.inward, ring, s.edge, minWidth, prefix),
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

  const taperTips = findTaperedTips(ring, samples, minWidth);
  for (const tip of taperTips) {
    const g = landingGrow(tip, samples, deltas);
    if (g < 0.04) continue;
    for (let i = 0; i < deltas.length; i++) {
      if (dist(samples[i].point, tip) < 5) deltas[i] = Math.min(deltas[i], g);
    }
  }
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

    const tinyEdge =
      dist(prev.point, curr.point) < minWidth * 0.4 || dist(curr.point, next.point) < minWidth * 0.4;
    const nearTaper = taperTips.some((tip) => dist(curr.point, tip) < 8);
    const isCorner =
      !nearTaper && !tinyEdge && Math.abs(turn) > 0.35 && Math.abs(turn) < 2.2 && delta > 0.05;
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

  const repaired = fixTaperedTips(cleanPoints(moved, true, 0.02), ring, samples, deltas, minWidth);
  const outline = unionPaths([repaired]);
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
    if (curve.centerline) {
      return compensateCenterline(cleaned, curve.closed, params);
    }
    return { outline: [], toolpath: [], singlePass: false };
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
