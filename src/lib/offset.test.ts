import { boundsOf, dist } from "./geometry";
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

// Open centerline still thickens the whole path.
{
  const line: Polyline = {
    id: "line",
    name: "line",
    closed: false,
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

// Tapered koru: wide belly stays, tip rounds to min width, curve stays fair.
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

  const tip = koruPts.reduce((best, p) => (p.x < best.x ? p : best), koruPts[0]);
  const tipSpan = localSpan(outline, { x: tip.x - 1, y: tip.y }, 10);
  assert(tipSpan > minWidth * 0.7, `rounded tip should be near min width, got ${tipSpan}`);

  const sharp = maxTurn(outline);
  assert(sharp < 2.6, `koru outline should stay smooth (max turn ${sharp.toFixed(2)} rad)`);
}

console.log("offset tests passed");
