import { uid } from "./geometry";
import type { Polyline } from "./types";

function curve(name: string, points: { x: number; y: number }[], closed = false): Polyline {
  return { id: uid("crv"), name, points, closed };
}

function arc(
  cx: number,
  cy: number,
  r: number,
  a0: number,
  a1: number,
  steps = 16,
): { x: number; y: number }[] {
  const pts: { x: number; y: number }[] = [];
  for (let i = 0; i <= steps; i++) {
    const t = a0 + ((a1 - a0) * i) / steps;
    pts.push({ x: cx + Math.cos(t) * r, y: cy + Math.sin(t) * r });
  }
  return pts;
}

export function createDemoCurves(): Polyline[] {
  const hook = [
    { x: 36, y: 48 },
    { x: 118, y: 48 },
    ...arc(118, 78, 30, -Math.PI / 2, Math.PI / 2, 14),
    { x: 86, y: 108 },
  ];

  return [
    curve("Straight slot", [
      { x: 40, y: 210 },
      { x: 220, y: 210 },
    ]),
    curve("Dogleg slot", [
      { x: 40, y: 150 },
      { x: 120, y: 150 },
      { x: 120, y: 92 },
      { x: 210, y: 92 },
    ]),
    curve("J-hook slot", hook),
    curve(
      "Square hole",
      [
        { x: 258, y: 78 },
        { x: 338, y: 78 },
        { x: 338, y: 148 },
        { x: 258, y: 148 },
      ],
      true,
    ),
    curve(
      "Part outline",
      [
        { x: 258, y: 176 },
        { x: 348, y: 176 },
        { x: 348, y: 246 },
        { x: 300, y: 246 },
        { x: 300, y: 214 },
        { x: 258, y: 214 },
      ],
      true,
    ),
  ];
}

export const PRESET_HINTS: Record<string, string> = {
  "Straight slot": "Classic red centerline → black stadium. Drag Min width to fatten it.",
  "Dogleg slot": "Inside corners pinch; round joins keep the plasma path machinable.",
  "J-hook slot": "Works on polylines with arcs, not just straight segments.",
  "Square hole": "Switch mode to Hole so the torch sits inside by half the kerf.",
  "Part outline": "Switch mode to Part so the torch rides outside the finished profile.",
};
