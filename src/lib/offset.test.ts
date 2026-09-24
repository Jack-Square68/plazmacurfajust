import { boundsOf, dist, distToPolyline } from "./geometry";
import { compensateCurve, ensureMinWidth } from "./offset";
import { createDemoCurves } from "./presets";
import { DEFAULT_PARAMS, type Point, type Polyline } from "./types";

function assert(cond: boolean, message: string): void {
  if (!cond) throw new Error(message);
}

function closed(name: string, points: Point[]): Polyline {
  return { id: name, name, points, closed: true };
}

function turningAngles(pts: Point[]): number[] {
  const angles: number[] = [];
  const n = pts.length;
  for (let i = 0; i < n; i++) {
    const a = pts[(i - 1 + n) % n];
    const b = pts[i];
    const c = pts[(i + 1) % n];
    const abx = b.x - a.x;
    const aby = b.y - a.y;
    const bcx = c.x - b.x;
    const bcy = c.y - b.y;
    const lab = Math.hypot(abx, aby);
    const lbc = Math.hypot(bcx, bcy);
    if (lab < 1e-9 || lbc < 1e-9) continue;
    const dot = (abx * bcx + aby * bcy) / (lab * lbc);
    const ang = Math.acos(Math.max(-1, Math.min(1, dot)));
    angles.push(ang);
  }
  return angles;
}

function maxTurn(pts: Point[]): number {
  return Math.max(0, ...turningAngles(pts));
}

function maxTurnWhere(pts: Point[], keep: (p: Point) => boolean): number {
  const angles: number[] = [];
  const n = pts.length;
  for (let i = 0; i < n; i++) {
    const a = pts[(i - 1 + n) % n];
    const b = pts[i];
    const c = pts[(i + 1) % n];
    if (!keep(b)) continue;
    const abx = b.x - a.x;
    const aby = b.y - a.y;
    const bcx = c.x - b.x;
    const bcy = c.y - b.y;
    const lab = Math.hypot(abx, aby);
    const lbc = Math.hypot(bcx, bcy);
    if (lab < 1e-9 || lbc < 1e-9) continue;
    const dot = (abx * bcx + aby * bcy) / (lab * lbc);
    angles.push(Math.acos(Math.max(-1, Math.min(1, dot))));
  }
  return Math.max(0, ...angles);
}

// Re-export helper if presets doesn't export the koru builder — build one here.
function makeKoru(): Point[] {
  try {
    const demo = createDemoCurves().find((c) => c.name === "Tapered koru");
    if (demo) return demo.points;
  } catch {
    /* fall through */
  }
  return [];
}

function localSpan(pts: Point[], at: Point, window = 8): number {
  const nearby = pts.filter((p) => dist(p, at) < window);
  if (nearby.length < 2) return 0;
  let best = 0;
  for (let i = 0; i < nearby.length; i++) {
    for (let j = i + 1; j < nearby.length; j++) {
      best = Math.max(best, dist(nearby[i], nearby[j]));
    }
  }
  return best;
}

const minWidth = 6;
const params = { ...DEFAULT_PARAMS, minWidth, kerf: 1.5, mode: "slot" as const };

// Large closed opening — already wider than min width.
{
  const square = closed("square", [
    { x: 0, y: 0 },
    { x: 80, y: 0 },
    { x: 80, y: 70 },
    { x: 0, y: 70 },
  ]);
  const result = compensateCurve(square, params);
  const b = boundsOf(result.outline.flat());
  assert(result.pinches === 0, `large square should not pinch, got ${result.pinches}`);
  assert(result.outline.length === 1, `large square should stay one outline, got ${result.outline.length}`);
  assert(!!b && Math.abs(b.maxX - b.minX - 80) < 0.6, "large square width should stay ~80");
  assert(!!b && Math.abs(b.maxY - b.minY - 70) < 0.6, "large square height should stay ~70");
}

// Thin closed slot 80 x 3 — only the 3 mm direction should grow to 6.
{
  const slot = closed("thin", [
    { x: 0, y: 0 },
    { x: 80, y: 0 },
    { x: 80, y: 3 },
    { x: 0, y: 3 },
  ]);
  const grown = ensureMinWidth(slot.points, minWidth);
  const b = boundsOf(grown.outline.flat());
  assert(grown.pinches > 0, "thin slot should report pinches");
  assert(!!b && b.maxY - b.minY > 5.4 && b.maxY - b.minY < 7.2, `thin slot height should be ~6, got ${b ? b.maxY - b.minY : "?"}`);
  assert(!!b && b.maxX - b.minX > 79, "thin slot should not shrink in length");
}

// Hourglass pinch — concave neck is thinner than min width.
{
  const pinch = closed("pinch", [
    { x: 20, y: 8 },
    { x: 58, y: 8 },
    { x: 72, y: 22 },
    { x: 86, y: 8 },
    { x: 124, y: 8 },
    { x: 124, y: 32 },
    { x: 86, y: 32 },
    { x: 72, y: 18 },
    { x: 58, y: 32 },
    { x: 20, y: 32 },
  ]);
  const grown = ensureMinWidth(pinch.points, minWidth);
  assert(grown.pinches > 0, `hourglass neck should be under min width, pinches=${grown.pinches}`);
  const neck = localSpan(grown.outline.flat(), { x: 72, y: 20 }, 10);
  assert(neck > 5.2, `hourglass neck should grow toward 6 mm, got ${neck}`);
}

// Open centerline still thickens the whole path.
{
  const line: Polyline = {
    id: "line",
    name: "line",
    closed: false,
    centerline: true,
    points: [
      { x: 0, y: 0 },
      { x: 40, y: 0 },
    ],
  };
  const result = compensateCurve(line, params);
  const b = boundsOf(result.outline.flat());
  assert(result.outline.length >= 1, "open line should outline");
  assert(!!b && b.maxY - b.minY > 5.4 && b.maxY - b.minY < 6.6, `open stadium height ~6, got ${b ? b.maxY - b.minY : "?"}`);
}

// Tapered koru: wide belly stays, tip is a thicker parallel — not a MinWidth bulb.
{
  const koruPts = makeKoru();
  assert(koruPts.length > 20, "koru preset should exist");
  const koru = closed("koru", koruPts);
  const original = boundsOf(koruPts)!;
  const grown = ensureMinWidth(koruPts, minWidth);
  const outline = grown.outline[0];
  assert(!!outline && outline.length > 40, "koru outline should be dense");
  const next = boundsOf(outline)!;
  // Wide end is toward +x of the belly; overall size should not explode.
  assert(next.maxX - next.minX < original.maxX - original.minX + minWidth + 4, "koru should not balloon");
  assert(grown.pinches > 0, "tapered koru tip should be under min width");

  let tipEdge = Infinity;
  let tip = koruPts[0];
  for (let i = 0; i < koruPts.length; i++) {
    const a = koruPts[i];
    const b = koruPts[(i + 1) % koruPts.length];
    const edge = dist(a, b);
    if (edge < tipEdge) {
      tipEdge = edge;
      tip = { x: (a.x + b.x) / 2, y: (a.y + b.y) / 2 };
    }
  }
  const tipSpan = localSpan(outline, tip, 10);
  assert(tipSpan > 2.0, `tapered tip should thicken, got ${tipSpan}`);
  const closest = Math.min(...outline.map((p) => dist(p, tip)));
  assert(closest < 2.8, `koru tip should hug the original, d=${closest}`);
  const tipTurn = maxTurnWhere(outline, (p) => dist(p, tip) < 12);
  assert(tipTurn < 0.8, `koru tip must not keep a house knuckle, turn=${tipTurn}`);

  const inward = koruPts.reduce((best, p) => {
    const d = dist(p, tip);
    return d > 6 && d < 12 && d < dist(best, tip) ? p : best;
  }, koruPts[0]);
  const spine = { x: tip.x - inward.x, y: tip.y - inward.y };
  const sl = Math.hypot(spine.x, spine.y) || 1;
  const sx = spine.x / sl;
  const sy = spine.y / sl;
  let capW = 0;
  let neckW = 0;
  for (const p of outline) {
    if (dist(p, tip) > 14) continue;
    const along = (p.x - tip.x) * sx + (p.y - tip.y) * sy;
    const across = Math.abs((p.x - tip.x) * sy - (p.y - tip.y) * sx);
    if (along >= 0 && along < 4) capW = Math.max(capW, across * 2);
    if (along < -5 && along > -10) neckW = Math.max(neckW, across * 2);
  }
  assert(neckW > 0.8, `koru neck should be measurable, neck=${neckW}`);
  assert(capW > 0.8, `koru cap should be measurable, cap=${capW}`);
  assert(capW < neckW * 1.35 + 1, `koru tip must not be a grafted bulb (cap=${capW} neck=${neckW})`);

  const midX = (original.minX + original.maxX) * 0.5;
  const sharp = maxTurnWhere(outline, (p) => p.x <= midX);
  assert(sharp < 1.2, `koru scroll should stay a smooth offset (max turn ${sharp.toFixed(2)} rad)`);

  const wide = koruPts.reduce((best, p) => (p.x > best.x ? p : best), koruPts[0]);
  const keep = distToPolyline(wide, { id: "o", name: "o", points: outline, closed: true });
  assert(keep < 0.6, `wide koru belly should stay on the original curve, drift=${keep}`);
}

// Tiny closed opening used to fall back to a full-loop stadium ribbon.
{
  const tiny = closed("tiny", [
    { x: 0, y: 0 },
    { x: 5, y: 0 },
    { x: 5, y: 1 },
    { x: 0, y: 1 },
  ]);
  const result = compensateCurve(tiny, params);
  const b = boundsOf(result.outline.flat());
  assert(result.pinches !== undefined && result.pinches > 0, "tiny closed slot should widen locally");
  assert(!!b && b.maxY - b.minY > 5.2 && b.maxY - b.minY < 7.4, `tiny slot height ~6, got ${b ? b.maxY - b.minY : "?"}`);
  assert(!!b && b.maxX - b.minX < 12, `tiny slot should not become a stadium around the loop, width=${b ? b.maxX - b.minX : "?"}`);
}

// Tessellated V of a wide opening: stay pointed, do not grow into a square bar.
{
  const chevron: Point[] = [];
  for (let i = 0; i < 50; i++) {
    const t = i / 49;
    chevron.push({ x: 50 * t, y: 80 - 80 * t });
  }
  for (let i = 1; i < 50; i++) {
    const t = i / 49;
    chevron.push({ x: 50 + 50 * t, y: 80 * t });
  }
  chevron.push({ x: 0, y: 80 });
  const grown = ensureMinWidth(chevron, minWidth);
  const outline = grown.outline[0] ?? [];
  const minY = Math.min(...outline.map((p) => p.y));
  const tip = outline.filter((p) => p.y < 4);
  const tipSpan = tip.length
    ? Math.max(...tip.map((p) => p.x)) - Math.min(...tip.map((p) => p.x))
    : 0;
  assert(grown.pinches === 0, `wide V opening should not count as a pinch, got ${grown.pinches}`);
  assert(minY < 1.2, `V tip should stay pointed, minY=${minY}`);
  assert(tipSpan < 8, `V tip should not become a square bar (span=${tipSpan})`);
}

// Imported open strokes (no centerline flag) stay as drawn.
{
  const stroke: Polyline = {
    id: "stroke",
    name: "stroke",
    closed: false,
    points: [
      { x: 0, y: 0 },
      { x: 40, y: 0 },
    ],
  };
  const result = compensateCurve(stroke, params);
  assert(result.outline.length === 0, "imported open stroke should not stadium-offset");
}

console.log("offset tests passed");
