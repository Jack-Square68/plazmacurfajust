import { boundsOf } from "./geometry";
import type { Point, Polyline } from "./types";

function dxfHeader(): string {
  return [
    "0",
    "SECTION",
    "2",
    "HEADER",
    "9",
    "$INSUNITS",
    "70",
    "4",
    "9",
    "$MEASUREMENT",
    "70",
    "1",
    "0",
    "ENDSEC",
    "0",
    "SECTION",
    "2",
    "TABLES",
    "0",
    "TABLE",
    "2",
    "LAYER",
    "70",
    "3",
    ...layer("ORIGINAL", 1),
    ...layer("KERF_OUTLINE", 7),
    ...layer("KERF_TOOLPATH", 30),
    "0",
    "ENDTAB",
    "0",
    "ENDSEC",
    "0",
    "SECTION",
    "2",
    "ENTITIES",
  ].join("\n");
}

function layer(name: string, color: number): string[] {
  return ["0", "LAYER", "2", name, "70", "0", "62", String(color), "6", "CONTINUOUS"];
}

function lwpolyline(points: Point[], closed: boolean, layerName: string): string {
  const n = points.length;
  if (n < 2) return "";
  const flags = closed ? 1 : 0;
  const verts = points.flatMap((p) => ["10", fmt(p.x), "20", fmt(p.y)]);
  return [
    "0",
    "LWPOLYLINE",
    "8",
    layerName,
    "90",
    String(n),
    "70",
    String(flags),
    ...verts,
  ].join("\n");
}

function fmt(n: number): string {
  return (Math.round(n * 10000) / 10000).toString();
}

export function curvesToDxf(
  originals: Polyline[],
  outlines: Point[][],
  toolpaths: Point[][],
): string {
  const entities: string[] = [];
  for (const curve of originals) {
    entities.push(lwpolyline(curve.points, curve.closed, "ORIGINAL"));
  }
  for (const path of outlines) {
    entities.push(lwpolyline(path, true, "KERF_OUTLINE"));
  }
  for (const path of toolpaths) {
    const closed = distClose(path);
    entities.push(lwpolyline(path, closed, "KERF_TOOLPATH"));
  }
  return [dxfHeader(), ...entities.filter(Boolean), "0", "ENDSEC", "0", "EOF", ""].join("\n");
}

function distClose(path: Point[]): boolean {
  if (path.length < 3) return false;
  const a = path[0];
  const b = path[path.length - 1];
  return Math.hypot(a.x - b.x, a.y - b.y) < 0.05;
}

export function downloadText(filename: string, text: string, mime: string): void {
  const blob = new Blob([text], { type: mime });
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  a.href = url;
  a.download = filename;
  a.click();
  URL.revokeObjectURL(url);
}

export function curvesToSvg(
  originals: Polyline[],
  outlines: Point[][],
  toolpaths: Point[][],
): string {
  const all = [
    ...originals.flatMap((c) => c.points),
    ...outlines.flatMap((p) => p),
    ...toolpaths.flatMap((p) => p),
  ];
  const b = boundsOf(all) ?? { minX: 0, minY: 0, maxX: 100, maxY: 100 };
  const pad = 12;
  const w = b.maxX - b.minX + pad * 2;
  const h = b.maxY - b.minY + pad * 2;
  const tx = (x: number) => x - b.minX + pad;
  const ty = (y: number) => b.maxY + pad - y;

  const poly = (pts: Point[], closed: boolean) =>
    pts.map((p, i) => `${i === 0 ? "M" : "L"} ${tx(p.x).toFixed(3)} ${ty(p.y).toFixed(3)}`).join(" ") +
    (closed ? " Z" : "");

  const originalPaths = originals
    .map(
      (c) =>
        `<path d="${poly(c.points, c.closed)}" fill="none" stroke="#c40d0d" stroke-width="0.7" />`,
    )
    .join("\n");
  const outlinePaths = outlines
    .map(
      (p) =>
        `<path d="${poly(p, true)}" fill="none" stroke="#111" stroke-width="0.85" />`,
    )
    .join("\n");
  const toolPaths = toolpaths
    .map(
      (p) =>
        `<path d="${poly(p, distClose(p))}" fill="none" stroke="#d97706" stroke-width="0.55" stroke-dasharray="2 1.4" />`,
    )
    .join("\n");

  return `<?xml version="1.0" encoding="UTF-8"?>
<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 ${w.toFixed(3)} ${h.toFixed(3)}" width="${w.toFixed(1)}mm" height="${h.toFixed(1)}mm">
  <rect width="100%" height="100%" fill="#d8d8d4"/>
  ${outlinePaths}
  ${toolPaths}
  ${originalPaths}
</svg>
`;
}
