import { autoClosePath, importVectorText } from "./import-vector";
import { boundsOf, dist } from "./geometry";

function assert(cond: boolean, message: string): void {
  if (!cond) throw new Error(message);
}

{
  const svg = `<?xml version="1.0"?>
<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 120 40">
  <g transform="translate(10 4)">
    <polygon points="0,0 80,0 80,8 0,8"/>
    <path d="M 0 20 L 38 20 L 40 24 L 42 20 L 80 20 L 80 32 L 42 32 L 40 28 L 38 32 L 0 32 Z"/>
  </g>
</svg>`;
  const { curves, warnings } = importVectorText(svg, "ornament.svg");
  assert(warnings.length === 0, `svg warnings: ${warnings.join("; ")}`);
  assert(curves.length === 2, `expected 2 svg curves, got ${curves.length}`);
  assert(curves.every((c) => c.closed), "svg openings should be closed");
  const wide = curves.find((c) => {
    const b = boundsOf(c.points);
    return b && b.maxY - b.minY > 10;
  });
  assert(!!wide, "hourglass path should import");
}

{
  const svg = `<svg><polyline points="0,0 40,0 40,0.4"/></svg>`;
  const { curves } = importVectorText(svg, "gap.svg");
  assert(curves.length === 1, "polyline should import");
  assert(!curves[0].closed, "large-gap polyline stays open");
}

{
  const closed = autoClosePath(
    [
      { x: 0, y: 0 },
      { x: 20, y: 0 },
      { x: 20, y: 8 },
      { x: 0.4, y: 0.2 },
    ],
    false,
  );
  assert(closed.closed, "near-closed path should auto-close");
  assert(dist(closed.points[0], closed.points[closed.points.length - 1]) > 0.05, "keep last vertex if gap is real");
}

{
  const dxf = [
    "0", "SECTION", "2", "ENTITIES",
    "0", "LWPOLYLINE", "90", "4", "70", "1",
    "10", "0", "20", "0",
    "10", "80", "20", "0",
    "10", "80", "20", "3",
    "10", "0", "20", "3",
    "0", "LINE",
    "10", "0", "20", "20",
    "11", "12", "21", "20",
    "0", "CIRCLE",
    "10", "10", "20", "40", "40", "6",
    "0", "ENDSEC", "0", "EOF",
  ].join("\n");
  const { curves } = importVectorText(dxf, "slots.dxf");
  assert(curves.length === 3, `expected 3 dxf curves, got ${curves.length}`);
  const slot = curves.find((c) => c.closed && c.points.length === 4);
  assert(!!slot, "closed lwpolyline should import");
  const circle = curves.find((c) => c.closed && c.points.length > 10);
  assert(!!circle, "circle should become a closed polyline");
  const line = curves.find((c) => !c.closed);
  assert(!!line && !line.centerline, "open dxf line is not a slot centerline");
}

console.log("import tests passed");
