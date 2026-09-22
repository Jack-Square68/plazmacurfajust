"use client";

import { useCallback, useEffect, useRef } from "react";

import { pan, screenToWorld, worldToScreen, zoomAt, type Camera } from "@/lib/camera";
import { centroid, formatMm, hitTestCurves, hitTestVertex, sampleAt } from "@/lib/geometry";
import type { Compensated, KerfParams, Point, Polyline, Tool } from "@/lib/types";

type Draft = { points: Point[]; closed: boolean };

type Props = {
  curves: Polyline[];
  selectedId: string | null;
  params: KerfParams;
  compensated: Compensated | null;
  camera: Camera;
  tool: Tool;
  spacePan: boolean;
  draft: Draft | null;
  hoverWorld: Point | null;
  onCamera: (camera: Camera) => void;
  onSelect: (id: string | null) => void;
  onCurves: (curves: Polyline[]) => void;
  onDraftPoint: (point: Point) => void;
  onFinishDraft: (closed: boolean) => void;
  onHover: (point: Point | null) => void;
};

type Drag =
  | { kind: "pan"; last: Point }
  | { kind: "vertex"; id: string; index: number }
  | { kind: "move"; id: string; last: Point }
  | null;

export function Viewport({
  curves,
  selectedId,
  params,
  compensated,
  camera,
  tool,
  spacePan,
  draft,
  hoverWorld,
  onCamera,
  onSelect,
  onCurves,
  onDraftPoint,
  onFinishDraft,
  onHover,
}: Props) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const wrapRef = useRef<HTMLDivElement>(null);
  const dragRef = useRef<Drag>(null);
  const movedRef = useRef(false);
  const cameraRef = useRef(camera);
  const curvesRef = useRef(curves);
  const selectedRef = useRef(selectedId);
  const toolRef = useRef(tool);
  const spaceRef = useRef(spacePan);

  useEffect(() => {
    cameraRef.current = camera;
    curvesRef.current = curves;
    selectedRef.current = selectedId;
    toolRef.current = tool;
    spaceRef.current = spacePan;
  }, [camera, curves, selectedId, tool, spacePan]);

  const paint = useCallback(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;
    const dpr = window.devicePixelRatio || 1;
    const width = canvas.width / dpr;
    const height = canvas.height / dpr;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    drawScene(ctx, {
      width,
      height,
      camera: cameraRef.current,
      curves: curvesRef.current,
      selectedId: selectedRef.current,
      compensated,
      params,
      draft,
      hoverWorld,
      tool: toolRef.current,
    });
  }, [compensated, params, draft, hoverWorld]);

  useEffect(() => {
    const wrap = wrapRef.current;
    const canvas = canvasRef.current;
    if (!wrap || !canvas) return;

    const resize = () => {
      const rect = wrap.getBoundingClientRect();
      const dpr = window.devicePixelRatio || 1;
      canvas.width = Math.max(1, Math.floor(rect.width * dpr));
      canvas.height = Math.max(1, Math.floor(rect.height * dpr));
      canvas.style.width = `${rect.width}px`;
      canvas.style.height = `${rect.height}px`;
      paint();
    };

    resize();
    const obs = new ResizeObserver(resize);
    obs.observe(wrap);
    return () => obs.disconnect();
  }, [paint]);

  useEffect(() => {
    paint();
  }, [paint, camera, curves, selectedId]);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const handler = (e: WheelEvent) => {
      e.preventDefault();
      const rect = canvas.getBoundingClientRect();
      const screen = { x: e.clientX - rect.left, y: e.clientY - rect.top };
      const factor = e.deltaY < 0 ? 1.12 : 1 / 1.12;
      onCamera(zoomAt(cameraRef.current, screen, factor));
    };
    canvas.addEventListener("wheel", handler, { passive: false });
    return () => canvas.removeEventListener("wheel", handler);
  }, [onCamera]);

  const eventPoint = (e: { clientX: number; clientY: number }): Point => {
    const canvas = canvasRef.current!;
    const rect = canvas.getBoundingClientRect();
    return { x: e.clientX - rect.left, y: e.clientY - rect.top };
  };

  const onPointerDown = (e: React.PointerEvent<HTMLCanvasElement>) => {
    if (e.button === 1 || e.button === 2) {
      dragRef.current = { kind: "pan", last: eventPoint(e) };
      e.currentTarget.setPointerCapture(e.pointerId);
      return;
    }
    if (e.button !== 0) return;
    const screen = eventPoint(e);
    const world = screenToWorld(cameraRef.current, screen);
    movedRef.current = false;

    if (spaceRef.current || toolRef.current === "pan") {
      dragRef.current = { kind: "pan", last: screen };
      e.currentTarget.setPointerCapture(e.pointerId);
      return;
    }

    if (toolRef.current === "draw") {
      onDraftPoint(world);
      return;
    }

    const pxThresh = Math.max(2.5, 16 / cameraRef.current.scale);
    const selected = curvesRef.current.find((c) => c.id === selectedRef.current);
    if (selected) {
      const vertex = hitTestVertex(world, selected, pxThresh);
      if (vertex !== null) {
        dragRef.current = { kind: "vertex", id: selected.id, index: vertex };
        e.currentTarget.setPointerCapture(e.pointerId);
        return;
      }
    }

    const hit = hitTestCurves(world, curvesRef.current, pxThresh);
    if (hit) {
      onSelect(hit);
      dragRef.current = { kind: "move", id: hit, last: world };
      e.currentTarget.setPointerCapture(e.pointerId);
      return;
    }

    onSelect(null);
    dragRef.current = { kind: "pan", last: screen };
    e.currentTarget.setPointerCapture(e.pointerId);
  };

  const onPointerMove = (e: React.PointerEvent<HTMLCanvasElement>) => {
    const screen = eventPoint(e);
    const world = screenToWorld(cameraRef.current, screen);
    onHover(world);

    const drag = dragRef.current;
    if (!drag) return;
    movedRef.current = true;

    if (drag.kind === "pan") {
      onCamera(pan(cameraRef.current, screen.x - drag.last.x, screen.y - drag.last.y));
      dragRef.current = { kind: "pan", last: screen };
      return;
    }

    if (drag.kind === "vertex") {
      onCurves(
        curvesRef.current.map((c) =>
          c.id === drag.id
            ? { ...c, points: c.points.map((p, i) => (i === drag.index ? world : p)) }
            : c,
        ),
      );
      return;
    }

    if (drag.kind === "move") {
      const dx = world.x - drag.last.x;
      const dy = world.y - drag.last.y;
      onCurves(
        curvesRef.current.map((c) =>
          c.id === drag.id
            ? { ...c, points: c.points.map((p) => ({ x: p.x + dx, y: p.y + dy })) }
            : c,
        ),
      );
      dragRef.current = { kind: "move", id: drag.id, last: world };
    }
  };

  const onPointerUp = (e: React.PointerEvent<HTMLCanvasElement>) => {
    const drag = dragRef.current;
    dragRef.current = null;
    if (drag?.kind === "move" && !movedRef.current) {
      onSelect(drag.id);
    }
    try {
      e.currentTarget.releasePointerCapture(e.pointerId);
    } catch {
      /* already released */
    }
  };

  const onContextMenu = (e: React.MouseEvent) => e.preventDefault();

  return (
    <div ref={wrapRef} className="relative min-h-[280px] flex-1 overflow-hidden">
      <canvas
        ref={canvasRef}
        className="absolute inset-0 h-full w-full touch-none"
        style={{
          cursor:
            spacePan || tool === "pan"
              ? "grab"
              : tool === "draw"
                ? "crosshair"
                : "default",
        }}
        onPointerDown={onPointerDown}
        onPointerMove={onPointerMove}
        onPointerUp={onPointerUp}
        onPointerLeave={() => onHover(null)}
        onDoubleClick={() => onFinishDraft(false)}
        onContextMenu={onContextMenu}
      />
    </div>
  );
}

type Scene = {
  width: number;
  height: number;
  camera: Camera;
  curves: Polyline[];
  selectedId: string | null;
  compensated: Compensated | null;
  params: KerfParams;
  draft: Draft | null;
  hoverWorld: Point | null;
  tool: Tool;
};

function drawScene(ctx: CanvasRenderingContext2D, s: Scene) {
  ctx.fillStyle = "#c9c9c4";
  ctx.fillRect(0, 0, s.width, s.height);
  drawGrid(ctx, s);
  drawAxes(ctx, s);

  if (s.compensated) {
    if (s.params.output !== "toolpath") {
      drawFilled(ctx, s.camera, s.compensated.outline, "rgba(17,17,17,0.10)");
      strokePaths(ctx, s.camera, s.compensated.outline, "#111111", 2.4, true, []);
    }
    if (s.params.output !== "outline") {
      const closed = s.compensated.toolpath.map((p) => p.length > 2 && !s.compensated!.singlePass);
      s.compensated.toolpath.forEach((path, i) => {
        strokePaths(ctx, s.camera, [path], "#d97706", 1.6, closed[i], [7, 5]);
      });
    }
  }

  for (const curve of s.curves) {
    const selected = curve.id === s.selectedId;
    if (curve.closed) {
      drawFilled(
        ctx,
        s.camera,
        [curve.points],
        selected ? "rgba(234,179,8,0.16)" : "rgba(196,13,13,0.10)",
      );
    }
    strokePaths(
      ctx,
      s.camera,
      [curve.points],
      selected ? "#eab308" : "#c40d0d",
      selected ? 2.6 : 2,
      curve.closed,
      [],
    );
    drawCurveLabel(ctx, s.camera, curve, selected);
    if (selected) {
      drawVertices(ctx, s.camera, curve.points);
      if (s.params.mode === "slot" && !curve.closed) {
        drawWidthDim(ctx, s.camera, curve, s.params.minWidth);
      }
    }
  }

  if (s.draft && s.draft.points.length) {
    const pts = s.hoverWorld ? [...s.draft.points, s.hoverWorld] : s.draft.points;
    strokePaths(ctx, s.camera, [pts], "#c40d0d", 1.8, s.draft.closed, [4, 4]);
    drawVertices(ctx, s.camera, s.draft.points);
  }

  ctx.fillStyle = "#3f3f46";
  ctx.font = "11px ui-monospace, SFMono-Regular, Menlo, monospace";
  ctx.fillText("mm  ·  scroll zoom  ·  drag empty to pan", 12, s.height - 12);
}

function drawGrid(ctx: CanvasRenderingContext2D, s: Scene) {
  const a = screenToWorld(s.camera, { x: 0, y: 0 });
  const b = screenToWorld(s.camera, { x: s.width, y: s.height });
  const minX = Math.min(a.x, b.x);
  const maxX = Math.max(a.x, b.x);
  const minY = Math.min(a.y, b.y);
  const maxY = Math.max(a.y, b.y);
  const minor = s.camera.scale > 4 ? 10 : 50;
  const major = minor * 5;
  const startX = Math.floor(minX / minor) * minor;
  const startY = Math.floor(minY / minor) * minor;

  for (let x = startX; x <= maxX; x += minor) {
    const p0 = worldToScreen(s.camera, { x, y: minY });
    const p1 = worldToScreen(s.camera, { x, y: maxY });
    ctx.beginPath();
    ctx.moveTo(p0.x, p0.y);
    ctx.lineTo(p1.x, p1.y);
    ctx.strokeStyle = Math.round(x) % major === 0 ? "rgba(0,0,0,0.16)" : "rgba(0,0,0,0.07)";
    ctx.lineWidth = 1;
    ctx.stroke();
  }
  for (let y = startY; y <= maxY; y += minor) {
    const p0 = worldToScreen(s.camera, { x: minX, y });
    const p1 = worldToScreen(s.camera, { x: maxX, y });
    ctx.beginPath();
    ctx.moveTo(p0.x, p0.y);
    ctx.lineTo(p1.x, p1.y);
    ctx.strokeStyle = Math.round(y) % major === 0 ? "rgba(0,0,0,0.16)" : "rgba(0,0,0,0.07)";
    ctx.lineWidth = 1;
    ctx.stroke();
  }
}

function drawAxes(ctx: CanvasRenderingContext2D, s: Scene) {
  const o = worldToScreen(s.camera, { x: 0, y: 0 });
  ctx.strokeStyle = "rgba(180, 35, 35, 0.55)";
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(0, o.y);
  ctx.lineTo(s.width, o.y);
  ctx.stroke();
  ctx.strokeStyle = "rgba(30, 90, 40, 0.5)";
  ctx.beginPath();
  ctx.moveTo(o.x, 0);
  ctx.lineTo(o.x, s.height);
  ctx.stroke();
}

function strokePaths(
  ctx: CanvasRenderingContext2D,
  camera: Camera,
  paths: Point[][],
  color: string,
  width: number,
  closed: boolean,
  dash: number[],
) {
  ctx.save();
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.lineJoin = "round";
  ctx.lineCap = "round";
  ctx.setLineDash(dash);
  for (const path of paths) {
    if (path.length < 2) continue;
    ctx.beginPath();
    path.forEach((p, i) => {
      const s = worldToScreen(camera, p);
      if (i === 0) ctx.moveTo(s.x, s.y);
      else ctx.lineTo(s.x, s.y);
    });
    if (closed) ctx.closePath();
    ctx.stroke();
  }
  ctx.restore();
}

function drawFilled(
  ctx: CanvasRenderingContext2D,
  camera: Camera,
  paths: Point[][],
  fill: string,
) {
  ctx.save();
  ctx.fillStyle = fill;
  ctx.beginPath();
  for (const path of paths) {
    if (path.length < 3) continue;
    path.forEach((p, i) => {
      const s = worldToScreen(camera, p);
      if (i === 0) ctx.moveTo(s.x, s.y);
      else ctx.lineTo(s.x, s.y);
    });
    ctx.closePath();
  }
  ctx.fill("evenodd");
  ctx.restore();
}

function drawVertices(ctx: CanvasRenderingContext2D, camera: Camera, points: Point[]) {
  ctx.fillStyle = "#fff";
  ctx.strokeStyle = "#111";
  ctx.lineWidth = 1;
  for (const p of points) {
    const s = worldToScreen(camera, p);
    ctx.fillRect(s.x - 3.5, s.y - 3.5, 7, 7);
    ctx.strokeRect(s.x - 3.5, s.y - 3.5, 7, 7);
  }
}

function drawWidthDim(
  ctx: CanvasRenderingContext2D,
  camera: Camera,
  curve: Polyline,
  width: number,
) {
  const sample = sampleAt(curve.points, curve.closed, 0.5);
  if (!sample || width <= 0) return;
  const n = { x: -sample.tangent.y, y: sample.tangent.x };
  const a = { x: sample.point.x - n.x * (width / 2), y: sample.point.y - n.y * (width / 2) };
  const b = { x: sample.point.x + n.x * (width / 2), y: sample.point.y + n.y * (width / 2) };
  const sa = worldToScreen(camera, a);
  const sb = worldToScreen(camera, b);
  ctx.save();
  ctx.strokeStyle = "#111";
  ctx.fillStyle = "#111";
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(sa.x, sa.y);
  ctx.lineTo(sb.x, sb.y);
  ctx.stroke();
  ctx.font = "11px ui-sans-serif, system-ui";
  const label = `${formatMm(width)} mm`;
  const mx = (sa.x + sb.x) / 2;
  const my = (sa.y + sb.y) / 2;
  ctx.fillStyle = "rgba(201,201,196,0.9)";
  const tw = ctx.measureText(label).width;
  ctx.fillRect(mx - tw / 2 - 4, my - 16, tw + 8, 14);
  ctx.fillStyle = "#111";
  ctx.fillText(label, mx - tw / 2, my - 5);
  ctx.restore();
}

function drawCurveLabel(
  ctx: CanvasRenderingContext2D,
  camera: Camera,
  curve: Polyline,
  selected: boolean,
) {
  if (curve.points.length === 0) return;
  const origin = curve.closed ? centroid(curve.points) : curve.points[0];
  const s = worldToScreen(camera, origin);
  ctx.save();
  ctx.font = "11px ui-sans-serif, system-ui";
  const tw = ctx.measureText(curve.name).width;
  const x = curve.closed ? s.x - tw / 2 : s.x + 8;
  const y = curve.closed ? s.y + 4 : s.y - 10;
  ctx.fillStyle = selected ? "rgba(253,224,71,0.92)" : "rgba(255,255,255,0.88)";
  ctx.fillRect(x - 4, y - 11, tw + 8, 16);
  ctx.strokeStyle = selected ? "#a16207" : "rgba(196,13,13,0.35)";
  ctx.strokeRect(x - 4, y - 11, tw + 8, 16);
  ctx.fillStyle = selected ? "#111" : "#7f1d1d";
  ctx.fillText(curve.name, x, y);
  ctx.restore();
}

