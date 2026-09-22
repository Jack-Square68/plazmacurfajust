import * as ClipperNS from "clipper-lib";

import { cleanPoints, ensureCcw } from "./geometry";
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
  ClipperOffset: new (
    miterLimit?: number,
    arcTolerance?: number,
  ) => {
    AddPath(path: IntPt[], joinType: number, endType: number): void;
    Execute(solution: IntPt[][], delta: number): void;
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
  const co = new ClipperLib.ClipperOffset(3, 0.04 * SCALE);
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

export function compensateCurve(curve: Polyline, params: KerfParams): Compensated {
  const cleaned = cleanPoints(curve.points, curve.closed);
  if (cleaned.length < 2) {
    return { outline: [], toolpath: [], singlePass: false, error: "Curve needs at least two points." };
  }

  if (params.mode === "slot") {
    const outlineDelta = params.minWidth / 2;
    const toolDelta = (params.minWidth - params.kerf) / 2;
    const common = {
      closed: curve.closed,
      closedAsLine: curve.closed,
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
