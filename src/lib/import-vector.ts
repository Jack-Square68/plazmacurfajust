import {
  boundsOf,
  cleanPoints,
  dist,
  polylineLength,
  uid,
} from "./geometry";
import type { Point, Polyline } from "./types";

export type ImportResult = {
  curves: Polyline[];
  warnings: string[];
};

type Mat = [number, number, number, number, number, number];

const IDENTITY: Mat = [1, 0, 0, 1, 0, 0];

function mulMat(a: Mat, b: Mat): Mat {
  return [
    a[0] * b[0] + a[2] * b[1],
    a[1] * b[0] + a[3] * b[1],
    a[0] * b[2] + a[2] * b[3],
    a[1] * b[2] + a[3] * b[3],
    a[0] * b[4] + a[2] * b[5] + a[4],
    a[1] * b[4] + a[3] * b[5] + a[5],
  ];
}

function applyMat(m: Mat, p: Point): Point {
  return { x: m[0] * p.x + m[2] * p.y + m[4], y: m[1] * p.x + m[3] * p.y + m[5] };
}

function translate(tx: number, ty: number): Mat {
  return [1, 0, 0, 1, tx, ty];
}

function scaleMat(sx: number, sy: number): Mat {
  return [sx, 0, 0, sy, 0, 0];
}

function rotateMat(deg: number, cx = 0, cy = 0): Mat {
  const r = (deg * Math.PI) / 180;
  const c = Math.cos(r);
  const s = Math.sin(r);
  return mulMat(mulMat(translate(cx, cy), [c, s, -s, c, 0, 0]), translate(-cx, -cy));
}

function parseNumbers(text: string): number[] {
  const out: number[] = [];
  const re = /[-+]?(?:\d*\.\d+|\d+)(?:[eE][-+]?\d+)?/g;
  let match: RegExpExecArray | null;
  while ((match = re.exec(text))) out.push(Number(match[0]));
  return out.filter((n) => Number.isFinite(n));
}

function parseTransform(attr: string | null): Mat {
  if (!attr) return IDENTITY;
  let m: Mat = IDENTITY;
  const re = /(matrix|translate|scale|rotate|skewX|skewY)\s*\(([^)]*)\)/gi;
  let match: RegExpExecArray | null;
  while ((match = re.exec(attr))) {
    const kind = match[1].toLowerCase();
    const n = parseNumbers(match[2]);
    if (kind === "matrix" && n.length >= 6) {
      m = mulMat(m, [n[0], n[1], n[2], n[3], n[4], n[5]]);
    } else if (kind === "translate") {
      m = mulMat(m, translate(n[0] ?? 0, n[1] ?? 0));
    } else if (kind === "scale") {
      m = mulMat(m, scaleMat(n[0] ?? 1, n[1] ?? n[0] ?? 1));
    } else if (kind === "rotate") {
      m = mulMat(m, rotateMat(n[0] ?? 0, n[1] ?? 0, n[2] ?? 0));
    }
  }
  return m;
}

function sampleCubic(p0: Point, p1: Point, p2: Point, p3: Point): Point[] {
  const len =
    dist(p0, p1) + dist(p1, p2) + dist(p2, p3);
  const steps = Math.max(6, Math.min(48, Math.ceil(len / 0.7)));
  const pts: Point[] = [];
  for (let i = 1; i <= steps; i++) {
    const t = i / steps;
    const u = 1 - t;
    pts.push({
      x:
        u * u * u * p0.x +
        3 * u * u * t * p1.x +
        3 * u * t * t * p2.x +
        t * t * t * p3.x,
      y:
        u * u * u * p0.y +
        3 * u * u * t * p1.y +
        3 * u * t * t * p2.y +
        t * t * t * p3.y,
    });
  }
  return pts;
}

function sampleQuad(p0: Point, p1: Point, p2: Point): Point[] {
  const len = dist(p0, p1) + dist(p1, p2);
  const steps = Math.max(5, Math.min(36, Math.ceil(len / 0.7)));
  const pts: Point[] = [];
  for (let i = 1; i <= steps; i++) {
    const t = i / steps;
    const u = 1 - t;
    pts.push({
      x: u * u * p0.x + 2 * u * t * p1.x + t * t * p2.x,
      y: u * u * p0.y + 2 * u * t * p1.y + t * t * p2.y,
    });
  }
  return pts;
}

function reflect(current: Point, control: Point): Point {
  return { x: 2 * current.x - control.x, y: 2 * current.y - control.y };
}

function svgArcToPoints(p0: Point, rx: number, ry: number, phiDeg: number, large: boolean, sweep: boolean, p1: Point): Point[] {
  if (dist(p0, p1) < 1e-9) return [];
  rx = Math.abs(rx);
  ry = Math.abs(ry);
  if (rx < 1e-9 || ry < 1e-9) return [p1];

  const phi = (phiDeg * Math.PI) / 180;
  const cos = Math.cos(phi);
  const sin = Math.sin(phi);
  const dx = (p0.x - p1.x) / 2;
  const dy = (p0.y - p1.y) / 2;
  let x1 = cos * dx + sin * dy;
  let y1 = -sin * dx + cos * dy;
  const lambda = (x1 * x1) / (rx * rx) + (y1 * y1) / (ry * ry);
  if (lambda > 1) {
    const s = Math.sqrt(lambda);
    rx *= s;
    ry *= s;
  }
  const rx2 = rx * rx;
  const ry2 = ry * ry;
  const x12 = x1 * x1;
  const y12 = y1 * y1;
  let radicand = (rx2 * ry2 - rx2 * y12 - ry2 * x12) / (rx2 * y12 + ry2 * x12);
  radicand = Math.max(0, radicand);
  const sign = large === sweep ? -1 : 1;
  const coef = sign * Math.sqrt(radicand);
  const cx1 = coef * ((rx * y1) / ry);
  const cy1 = coef * (-(ry * x1) / rx);
  const cx = cos * cx1 - sin * cy1 + (p0.x + p1.x) / 2;
  const cy = sin * cx1 + cos * cy1 + (p0.y + p1.y) / 2;

  const angle = (ux: number, uy: number, vx: number, vy: number) => {
    const n = Math.hypot(ux, uy) * Math.hypot(vx, vy);
    if (n < 1e-12) return 0;
    const d = Math.max(-1, Math.min(1, (ux * vx + uy * vy) / n));
    const a = Math.acos(d);
    return ux * vy - uy * vx < 0 ? -a : a;
  };
  const ux = (x1 - cx1) / rx;
  const uy = (y1 - cy1) / ry;
  const vx = (-x1 - cx1) / rx;
  const vy = (-y1 - cy1) / ry;
  let start = angle(1, 0, ux, uy);
  let delta = angle(ux, uy, vx, vy);
  if (!sweep && delta > 0) delta -= Math.PI * 2;
  if (sweep && delta < 0) delta += Math.PI * 2;

  const radius = (rx + ry) / 2;
  const steps = Math.max(8, Math.min(64, Math.ceil((Math.abs(delta) * radius) / 0.55)));
  const pts: Point[] = [];
  for (let i = 1; i <= steps; i++) {
    const t = start + (delta * i) / steps;
    const x = cx + rx * Math.cos(t) * cos - ry * Math.sin(t) * sin;
    const y = cy + rx * Math.cos(t) * sin + ry * Math.sin(t) * cos;
    pts.push({ x, y });
  }
  return pts;
}

type PathSeg = { points: Point[]; closed: boolean };

function parseSvgPath(d: string, mat: Mat): PathSeg[] {
  const segs: PathSeg[] = [];
  let current: Point[] = [];
  let start: Point = { x: 0, y: 0 };
  let cursor: Point = { x: 0, y: 0 };
  let lastCubic: Point | null = null;
  let lastQuad: Point | null = null;
  let closed = false;

  const flush = () => {
    const pts = cleanPoints(current.map((p) => applyMat(mat, p)), closed, 0.02);
    if (pts.length >= 2) segs.push({ points: pts, closed });
    current = [];
    closed = false;
  };

  const push = (p: Point) => {
    cursor = p;
    current.push(p);
    lastCubic = null;
    lastQuad = null;
  };

  const tokens = d.match(/[MmLlHhVvCcSsQqTtAaZz]|[-+]?(?:\d*\.\d+|\d+)(?:[eE][-+]?\d+)?/g) ?? [];
  let i = 0;
  let cmd = "";
  while (i < tokens.length) {
    const tok = tokens[i];
    if (/^[A-Za-z]$/.test(tok)) {
      cmd = tok;
      i += 1;
      if (cmd === "Z" || cmd === "z") {
        if (current.length >= 2) {
          closed = true;
          cursor = start;
        }
        flush();
        current = [start];
        continue;
      }
    }
    if (!cmd) {
      i += 1;
      continue;
    }

    const rel = cmd === cmd.toLowerCase();
    const take = (n: number) => {
      const vals: number[] = [];
      for (let k = 0; k < n && i < tokens.length; k++, i++) {
        const v = Number(tokens[i]);
        if (!Number.isFinite(v)) break;
        vals.push(v);
      }
      return vals;
    };

    const abs = (x: number, y: number): Point =>
      rel ? { x: cursor.x + x, y: cursor.y + y } : { x, y };

    switch (cmd.toUpperCase()) {
      case "M": {
        const n = take(2);
        if (n.length < 2) break;
        if (current.length) flush();
        const p = abs(n[0], n[1]);
        start = p;
        current = [p];
        cursor = p;
        lastCubic = null;
        lastQuad = null;
        cmd = rel ? "l" : "L";
        break;
      }
      case "L": {
        const n = take(2);
        if (n.length < 2) break;
        push(abs(n[0], n[1]));
        break;
      }
      case "H": {
        const n = take(1);
        if (!n.length) break;
        push({ x: rel ? cursor.x + n[0] : n[0], y: cursor.y });
        break;
      }
      case "V": {
        const n = take(1);
        if (!n.length) break;
        push({ x: cursor.x, y: rel ? cursor.y + n[0] : n[0] });
        break;
      }
      case "C": {
        const n = take(6);
        if (n.length < 6) break;
        const c1 = abs(n[0], n[1]);
        const c2 = abs(n[2], n[3]);
        const p = abs(n[4], n[5]);
        current.push(...sampleCubic(cursor, c1, c2, p));
        cursor = p;
        lastCubic = c2;
        lastQuad = null;
        break;
      }
      case "S": {
        const n = take(4);
        if (n.length < 4) break;
        const c1 = lastCubic ? reflect(cursor, lastCubic) : cursor;
        const c2 = abs(n[0], n[1]);
        const p = abs(n[2], n[3]);
        current.push(...sampleCubic(cursor, c1, c2, p));
        cursor = p;
        lastCubic = c2;
        lastQuad = null;
        break;
      }
      case "Q": {
        const n = take(4);
        if (n.length < 4) break;
        const c = abs(n[0], n[1]);
        const p = abs(n[2], n[3]);
        current.push(...sampleQuad(cursor, c, p));
        cursor = p;
        lastQuad = c;
        lastCubic = null;
        break;
      }
      case "T": {
        const n = take(2);
        if (n.length < 2) break;
        const c: Point = lastQuad ? reflect(cursor, lastQuad) : { ...cursor };
        const p = abs(n[0], n[1]);
        current.push(...sampleQuad(cursor, c, p));
        cursor = p;
        lastQuad = c;
        lastCubic = null;
        break;
      }
      case "A": {
        const n = take(7);
        if (n.length < 7) break;
        const p = abs(n[5], n[6]);
        current.push(
          ...svgArcToPoints(cursor, n[0], n[1], n[2], Boolean(n[3]), Boolean(n[4]), p),
        );
        cursor = p;
        lastCubic = null;
        lastQuad = null;
        break;
      }
      default:
        i += 1;
        break;
    }
  }
  flush();
  return segs;
}

function parsePointsAttr(raw: string | null, mat: Mat): Point[] {
  if (!raw) return [];
  const n = parseNumbers(raw);
  const pts: Point[] = [];
  for (let i = 0; i + 1 < n.length; i += 2) {
    pts.push(applyMat(mat, { x: n[i], y: n[i + 1] }));
  }
  return pts;
}

function sampleCircle(cx: number, cy: number, rx: number, ry: number, mat: Mat): Point[] {
  const steps = Math.max(24, Math.min(96, Math.ceil(((rx + ry) * Math.PI) / 0.7)));
  const pts: Point[] = [];
  for (let i = 0; i < steps; i++) {
    const a = (Math.PI * 2 * i) / steps;
    pts.push(applyMat(mat, { x: cx + Math.cos(a) * rx, y: cy + Math.sin(a) * ry }));
  }
  return pts;
}

function parseAttrs(raw: string): Record<string, string> {
  const attrs: Record<string, string> = {};
  const re = /([:@A-Za-z_][\w:.-]*)\s*=\s*(?:"([^"]*)"|'([^']*)')/g;
  let match: RegExpExecArray | null;
  while ((match = re.exec(raw))) {
    attrs[match[1].toLowerCase()] = match[2] ?? match[3] ?? "";
  }
  return attrs;
}

function numAttr(attrs: Record<string, string>, name: string, fallback = 0): number {
  const raw = attrs[name];
  if (raw == null || raw === "") return fallback;
  const n = Number(raw);
  return Number.isFinite(n) ? n : fallback;
}

function emitSvgShape(tag: string, attrs: Record<string, string>, mat: Mat, out: PathSeg[]): void {
  if (tag === "path") {
    const d = attrs.d;
    if (d) out.push(...parseSvgPath(d, mat));
    return;
  }
  if (tag === "polygon") {
    const pts = cleanPoints(parsePointsAttr(attrs.points ?? null, mat), true, 0.02);
    if (pts.length >= 3) out.push({ points: pts, closed: true });
    return;
  }
  if (tag === "polyline") {
    const pts = cleanPoints(parsePointsAttr(attrs.points ?? null, mat), false, 0.02);
    if (pts.length >= 2) out.push({ points: pts, closed: false });
    return;
  }
  if (tag === "rect") {
    const x = numAttr(attrs, "x");
    const y = numAttr(attrs, "y");
    const w = numAttr(attrs, "width");
    const h = numAttr(attrs, "height");
    if (w > 0 && h > 0) {
      out.push({
        points: [
          applyMat(mat, { x, y }),
          applyMat(mat, { x: x + w, y }),
          applyMat(mat, { x: x + w, y: y + h }),
          applyMat(mat, { x, y: y + h }),
        ],
        closed: true,
      });
    }
    return;
  }
  if (tag === "circle") {
    const r = numAttr(attrs, "r");
    const pts = sampleCircle(numAttr(attrs, "cx"), numAttr(attrs, "cy"), r, r, mat);
    if (pts.length >= 3) out.push({ points: pts, closed: true });
    return;
  }
  if (tag === "ellipse") {
    const pts = sampleCircle(
      numAttr(attrs, "cx"),
      numAttr(attrs, "cy"),
      numAttr(attrs, "rx"),
      numAttr(attrs, "ry"),
      mat,
    );
    if (pts.length >= 3) out.push({ points: pts, closed: true });
  }
  if (tag === "line") {
    const a = applyMat(mat, { x: numAttr(attrs, "x1"), y: numAttr(attrs, "y1") });
    const b = applyMat(mat, { x: numAttr(attrs, "x2"), y: numAttr(attrs, "y2") });
    if (dist(a, b) > 0.02) out.push({ points: [a, b], closed: false });
  }
}

function parseSvgShapes(text: string): PathSeg[] {
  const segs: PathSeg[] = [];
  const stack: Mat[] = [IDENTITY];
  let skipDepth = 0;
  const stripped = text.replace(/<!--[\s\S]*?-->/g, "");
  const tagRe = /<(\/)?([A-Za-z:][\w:.-]*)([^>]*)(\/)?>/g;
  let match: RegExpExecArray | null;
  while ((match = tagRe.exec(stripped))) {
    const closing = Boolean(match[1]);
    const selfClose = Boolean(match[4]) || /\/\s*$/.test(match[3]);
    const tag = match[2].toLowerCase().replace(/^.*:/, "");
    if (closing) {
      if (tag === "defs" || tag === "clippath" || tag === "mask" || tag === "symbol") {
        skipDepth = Math.max(0, skipDepth - 1);
      } else if (skipDepth === 0 && (tag === "g" || tag === "svg")) {
        stack.pop();
      }
      continue;
    }
    const skip =
      tag === "defs" || tag === "clippath" || tag === "mask" || tag === "symbol";
    if (skip) {
      if (!selfClose) skipDepth += 1;
      continue;
    }
    if (skipDepth > 0) continue;
    const attrs = parseAttrs(match[3] ?? "");
    const parent = stack[stack.length - 1] ?? IDENTITY;
    const mat = mulMat(parent, parseTransform(attrs.transform ?? null));
    if (tag === "g" || tag === "svg") {
      stack.push(mat);
      if (selfClose) stack.pop();
      continue;
    }
    emitSvgShape(tag, attrs, mat, segs);
  }
  return segs;
}

export function autoClosePath(points: Point[], closed: boolean): { points: Point[]; closed: boolean } {
  const cleaned = cleanPoints(points, closed, 0.02);
  if (closed && cleaned.length >= 3) return { points: cleaned, closed: true };
  if (cleaned.length < 3) return { points: cleaned, closed: false };
  const gap = dist(cleaned[0], cleaned[cleaned.length - 1]);
  const len = polylineLength(cleaned, false);
  const thresh = Math.max(0.8, Math.min(8, len * 0.02));
  if (gap <= thresh) {
    const pts = gap < 0.05 ? cleaned.slice(0, -1) : cleaned;
    return { points: pts, closed: pts.length >= 3 };
  }
  return { points: cleaned, closed: false };
}

function nameStem(filename: string): string {
  return filename.replace(/\.[^.]+$/, "").trim() || "import";
}

function toCurves(segs: PathSeg[], filename: string, flipY: boolean): Polyline[] {
  const closedUp = segs.map((s) => autoClosePath(s.points, s.closed));
  const usable = closedUp.filter((s) => s.points.length >= 2);
  if (usable.length === 0) return [];

  let minY = Infinity;
  let maxY = -Infinity;
  if (flipY) {
    for (const s of usable) {
      for (const p of s.points) {
        minY = Math.min(minY, p.y);
        maxY = Math.max(maxY, p.y);
      }
    }
  }

  const stem = nameStem(filename);
  return usable.map((s, i) => {
    const points = flipY
      ? s.points.map((p) => ({ x: p.x, y: minY + maxY - p.y }))
      : s.points;
    const closed = s.closed;
    const name = usable.length === 1 ? stem : `${stem} ${i + 1}`;
    return {
      id: uid("imp"),
      name,
      points,
      closed,
    };
  });
}

function importSvg(text: string, filename: string): ImportResult {
  const warnings: string[] = [];
  const segs = parseSvgShapes(text);
  const curves = toCurves(segs, filename, true);
  if (curves.length === 0) warnings.push("No paths, polygons, or shapes found in this SVG.");
  return { curves, warnings };
}

type DxfGroup = { code: number; value: string };

function parseDxfGroups(text: string): DxfGroup[] {
  const lines = text.replace(/\r\n/g, "\n").replace(/\r/g, "\n").split("\n");
  const groups: DxfGroup[] = [];
  for (let i = 0; i + 1 < lines.length; i += 2) {
    const code = Number(lines[i].trim());
    if (!Number.isFinite(code)) {
      i -= 1;
      continue;
    }
    groups.push({ code, value: lines[i + 1] ?? "" });
  }
  return groups;
}

function num(value: string): number {
  const n = Number(value.trim());
  return Number.isFinite(n) ? n : 0;
}

function bulgePoints(a: Point, b: Point, bulge: number): Point[] {
  if (Math.abs(bulge) < 1e-8) return [];
  const chord = dist(a, b);
  if (chord < 1e-9) return [];
  const angle = 4 * Math.atan(bulge);
  const radius = Math.abs(chord / (2 * Math.sin(angle / 2)));
  const mx = (a.x + b.x) / 2;
  const my = (a.y + b.y) / 2;
  const ux = (b.x - a.x) / chord;
  const uy = (b.y - a.y) / chord;
  let h = Math.sqrt(Math.max(0, radius * radius - (chord * 0.5) ** 2));
  if (Math.abs(angle) > Math.PI) h = -h;
  const sign = bulge >= 0 ? 1 : -1;
  const cx = mx - uy * h * sign;
  const cy = my + ux * h * sign;
  const a0 = Math.atan2(a.y - cy, a.x - cx);
  const steps = Math.max(6, Math.min(64, Math.ceil((Math.abs(angle) * radius) / 0.45)));
  const pts: Point[] = [];
  for (let i = 1; i < steps; i++) {
    const t = a0 + (angle * i) / steps;
    pts.push({ x: cx + Math.cos(t) * radius, y: cy + Math.sin(t) * radius });
  }
  return pts;
}

function expandBulge(verts: { p: Point; bulge: number }[], closed: boolean): Point[] {
  const pts: Point[] = [];
  const n = verts.length;
  const last = closed ? n : n - 1;
  for (let i = 0; i < last; i++) {
    const a = verts[i];
    const b = verts[(i + 1) % n];
    pts.push(a.p);
    pts.push(...bulgePoints(a.p, b.p, a.bulge));
  }
  if (!closed && n) pts.push(verts[n - 1].p);
  return pts;
}

function sliceEntities(groups: DxfGroup[]): DxfGroup[] {
  let start = -1;
  let end = groups.length;
  for (let i = 0; i < groups.length; i++) {
    if (groups[i].code === 0 && groups[i].value.trim() === "SECTION") {
      const name = groups[i + 1];
      if (name && name.code === 2 && name.value.trim().toUpperCase() === "ENTITIES") {
        start = i + 2;
      }
    }
    if (start >= 0 && groups[i].code === 0 && groups[i].value.trim() === "ENDSEC") {
      end = i;
      break;
    }
  }
  return start >= 0 ? groups.slice(start, end) : groups;
}

function importDxf(text: string, filename: string): ImportResult {
  const warnings: string[] = [];
  const groups = sliceEntities(parseDxfGroups(text));
  const segs: PathSeg[] = [];

  const entityAt = (start: number): { type: string; end: number; body: DxfGroup[] } => {
    const type = groups[start].value.trim().toUpperCase();
    let end = start + 1;
    while (end < groups.length && groups[end].code !== 0) end += 1;
    return { type, end, body: groups.slice(start + 1, end) };
  };

  let i = 0;
  while (i < groups.length) {
    if (groups[i].code !== 0) {
      i += 1;
      continue;
    }
    const { type, end, body } = entityAt(i);
    if (type === "LWPOLYLINE") {
      const verts: { p: Point; bulge: number }[] = [];
      let closed = false;
      let pending: { x?: number; y?: number; bulge: number } = { bulge: 0 };
      const flushVert = () => {
        if (pending.x != null && pending.y != null) {
          verts.push({ p: { x: pending.x, y: pending.y }, bulge: pending.bulge });
        }
        pending = { bulge: 0 };
      };
      for (const g of body) {
        if (g.code === 70) closed = (num(g.value) & 1) === 1;
        if (g.code === 10) {
          flushVert();
          pending.x = num(g.value);
        }
        if (g.code === 20) pending.y = num(g.value);
        if (g.code === 42) pending.bulge = num(g.value);
      }
      flushVert();
      const pts = cleanPoints(expandBulge(verts, closed), closed, 0.02);
      if (pts.length >= 2) segs.push({ points: pts, closed });
    } else if (type === "POLYLINE") {
      let closed = false;
      for (const g of body) {
        if (g.code === 70) closed = (num(g.value) & 1) === 1;
      }
      const verts: { p: Point; bulge: number }[] = [];
      let j = end;
      while (j < groups.length && groups[j].code === 0) {
        const nxt = entityAt(j);
        if (nxt.type === "SEQEND") {
          j = nxt.end;
          break;
        }
        if (nxt.type !== "VERTEX") break;
        let x = 0;
        let y = 0;
        let bulge = 0;
        for (const g of nxt.body) {
          if (g.code === 10) x = num(g.value);
          if (g.code === 20) y = num(g.value);
          if (g.code === 42) bulge = num(g.value);
        }
        verts.push({ p: { x, y }, bulge });
        j = nxt.end;
      }
      i = j;
      const pts = cleanPoints(expandBulge(verts, closed), closed, 0.02);
      if (pts.length >= 2) segs.push({ points: pts, closed });
      continue;
    } else if (type === "LINE") {
      let x1 = 0;
      let y1 = 0;
      let x2 = 0;
      let y2 = 0;
      for (const g of body) {
        if (g.code === 10) x1 = num(g.value);
        if (g.code === 20) y1 = num(g.value);
        if (g.code === 11) x2 = num(g.value);
        if (g.code === 21) y2 = num(g.value);
      }
      if (Math.hypot(x2 - x1, y2 - y1) > 0.02) {
        segs.push({ points: [{ x: x1, y: y1 }, { x: x2, y: y2 }], closed: false });
      }
    } else if (type === "CIRCLE") {
      let cx = 0;
      let cy = 0;
      let r = 0;
      for (const g of body) {
        if (g.code === 10) cx = num(g.value);
        if (g.code === 20) cy = num(g.value);
        if (g.code === 40) r = num(g.value);
      }
      if (r > 0.02) segs.push({ points: sampleCircle(cx, cy, r, r, IDENTITY), closed: true });
    } else if (type === "ARC") {
      let cx = 0;
      let cy = 0;
      let r = 0;
      let a0 = 0;
      let a1 = 0;
      for (const g of body) {
        if (g.code === 10) cx = num(g.value);
        if (g.code === 20) cy = num(g.value);
        if (g.code === 40) r = num(g.value);
        if (g.code === 50) a0 = num(g.value);
        if (g.code === 51) a1 = num(g.value);
      }
      if (r > 0.02) {
        let start = (a0 * Math.PI) / 180;
        let end = (a1 * Math.PI) / 180;
        if (end <= start) end += Math.PI * 2;
        const steps = Math.max(8, Math.min(64, Math.ceil(((end - start) * r) / 0.45)));
        const pts: Point[] = [];
        for (let k = 0; k <= steps; k++) {
          const a = start + ((end - start) * k) / steps;
          pts.push({ x: cx + Math.cos(a) * r, y: cy + Math.sin(a) * r });
        }
        segs.push({ points: pts, closed: false });
      }
    }
    i = end;
  }

  const curves = toCurves(segs, filename, false);
  if (curves.length === 0) warnings.push("No polylines, lines, or circles found in this DXF.");
  return { curves, warnings };
}

export function importVectorText(text: string, filename: string): ImportResult {
  const lower = filename.toLowerCase();
  if (lower.endsWith(".svg") || text.trimStart().startsWith("<")) {
    return importSvg(text, filename);
  }
  return importDxf(text, filename);
}

export function importVectorFile(file: File): Promise<ImportResult> {
  return file.text().then((text) => importVectorText(text, file.name));
}

export function pickDefaultImport(curves: Polyline[]): Polyline | null {
  return curves.find((c) => c.closed) ?? curves[0] ?? null;
}

export function boundsHint(curves: Polyline[]): string {
  const b = boundsOf(curves.flatMap((c) => c.points));
  if (!b) return "";
  const w = (b.maxX - b.minX).toFixed(1);
  const h = (b.maxY - b.minY).toFixed(1);
  return `${curves.length} curve${curves.length === 1 ? "" : "s"} · ${w} × ${h} mm`;
}
