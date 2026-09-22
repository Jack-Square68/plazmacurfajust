"use client";

import { useCallback, useEffect, useMemo, useRef, useState } from "react";

import { KerfPanel } from "@/components/kerf-panel";
import { Toolbar } from "@/components/toolbar";
import { Viewport } from "@/components/viewport";
import { TooltipProvider } from "@/components/ui/tooltip";
import { fitCurves } from "@/lib/camera";
import type { Camera } from "@/lib/camera";
import { curvesToDxf, curvesToSvg, downloadText } from "@/lib/export";
import { polylineLength, uid } from "@/lib/geometry";
import { compensateCurve } from "@/lib/offset";
import { createDemoCurves } from "@/lib/presets";
import {
  DEFAULT_PARAMS,
  type KerfParams,
  type Point,
  type Polyline,
  type Tool,
} from "@/lib/types";

type Draft = { points: Point[]; closed: boolean };

const bootstrapCurves = createDemoCurves();

export function KerfApp() {
  const [curves, setCurves] = useState<Polyline[]>(bootstrapCurves);
  const [selectedId, setSelectedId] = useState<string | null>(bootstrapCurves[0]?.id ?? null);
  const [params, setParams] = useState<KerfParams>(DEFAULT_PARAMS);
  const [tool, setTool] = useState<Tool>("select");
  const [spacePan, setSpacePan] = useState(false);
  const [draft, setDraft] = useState<Draft | null>(null);
  const [hoverWorld, setHoverWorld] = useState<Point | null>(null);
  const [camera, setCamera] = useState<Camera>({ scale: 2.2, ox: 40, oy: 420 });
  const stageRef = useRef<HTMLDivElement>(null);
  const didFit = useRef(false);

  const chooseTool = useCallback((next: Tool) => {
    setTool(next);
    if (next === "draw") setDraft({ points: [], closed: false });
    else setDraft((current) => (current && current.points.length === 0 ? null : current));
  }, []);

  useEffect(() => {
    const el = stageRef.current;
    if (!el) return;
    const run = () => {
      if (didFit.current) return;
      const rect = el.getBoundingClientRect();
      if (rect.width < 20) return;
      didFit.current = true;
      setCamera(fitCurves(curves, rect.width, rect.height));
    };
    const obs = new ResizeObserver(run);
    obs.observe(el);
    return () => obs.disconnect();
  }, [curves]);

  const selectCurve = useCallback((id: string | null, list: Polyline[] = curves) => {
    setSelectedId(id);
    if (!id) return;
    const curve = list.find((item) => item.id === id);
    if (!curve) return;
    if (!curve.closed) {
      setParams((current) => (current.mode === "slot" ? current : { ...current, mode: "slot" }));
      return;
    }
    if (curve.name === "Square hole") {
      setParams((current) => ({ ...current, mode: "hole" }));
    } else if (curve.name === "Part outline") {
      setParams((current) => ({ ...current, mode: "part" }));
    }
  }, [curves]);

  const selected = curves.find((c) => c.id === selectedId) ?? null;

  const compensated = useMemo(() => {
    if (!selected) return null;
    return compensateCurve(selected, params);
  }, [selected, params]);

  const fit = useCallback(() => {
    const el = stageRef.current;
    if (!el) return;
    const rect = el.getBoundingClientRect();
    setCamera(fitCurves(curves, rect.width, rect.height));
  }, [curves]);

  const reset = () => {
    const next = createDemoCurves();
    setCurves(next);
    setSelectedId(next[0]?.id ?? null);
    setDraft(null);
    didFit.current = false;
    setParams(DEFAULT_PARAMS);
    setTool("select");
  };

  const finishDraft = useCallback(
    (closed: boolean) => {
      if (!draft || draft.points.length < 2) {
        setDraft(null);
        return;
      }
      const curve: Polyline = {
        id: uid("crv"),
        name: closed ? "Drawn loop" : "Drawn curve",
        points: draft.points,
        closed,
      };
      setCurves((prev) => [...prev, curve]);
      setSelectedId(curve.id);
      setDraft(null);
      setTool("select");
    },
    [draft],
  );

  const onDraftPoint = (point: Point) => {
    setDraft((prev) => {
      if (!prev) return { points: [point], closed: false };
      const first = prev.points[0];
      if (
        prev.points.length >= 3 &&
        Math.hypot(point.x - first.x, point.y - first.y) < 4 / camera.scale
      ) {
        queueMicrotask(() => finishDraft(true));
        return prev;
      }
      return { ...prev, points: [...prev.points, point] };
    });
  };

  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      const tag = (e.target as HTMLElement | null)?.tagName;
      if (tag === "INPUT" || tag === "TEXTAREA") return;

      if (e.code === "Space") {
        if (!e.repeat) setSpacePan(true);
        e.preventDefault();
      }
      if (e.key === "v" || e.key === "V") chooseTool("select");
      if (e.key === "d" || e.key === "D") chooseTool("draw");
      if (e.key === "h" || e.key === "H") chooseTool("pan");
      if (e.key === "Escape") {
        setDraft(null);
        setTool("select");
        setSelectedId(null);
      }
      if (e.key === "Enter" && draft) {
        finishDraft(false);
      }
      if ((e.key === "c" || e.key === "C") && draft) {
        finishDraft(true);
      }
      if ((e.key === "Backspace" || e.key === "Delete") && draft) {
        setDraft((prev) =>
          prev && prev.points.length
            ? { ...prev, points: prev.points.slice(0, -1) }
            : prev,
        );
        e.preventDefault();
        return;
      }
      if ((e.key === "Backspace" || e.key === "Delete") && selectedId && !draft) {
        const next = curves.filter((c) => c.id !== selectedId);
        setCurves(next);
        selectCurve(next[0]?.id ?? null, next);
      }
    };
    const onUp = (e: KeyboardEvent) => {
      if (e.code === "Space") setSpacePan(false);
    };
    window.addEventListener("keydown", onKey);
    window.addEventListener("keyup", onUp);
    return () => {
      window.removeEventListener("keydown", onKey);
      window.removeEventListener("keyup", onUp);
    };
  }, [chooseTool, curves, draft, finishDraft, selectCurve, selectedId]);

  const exportAll = (kind: "dxf" | "svg") => {
    const originals = curves;
    const outlines: Point[][] = [];
    const toolpaths: Point[][] = [];
    for (const curve of originals) {
      const result = compensateCurve(curve, paramsFor(curve, params));
      if (params.output !== "toolpath") outlines.push(...result.outline);
      if (params.output !== "outline") toolpaths.push(...result.toolpath);
    }
    if (kind === "dxf") {
      downloadText(
        "plasma-kerf.dxf",
        curvesToDxf(originals, outlines, toolpaths),
        "application/dxf",
      );
    } else {
      downloadText(
        "plasma-kerf.svg",
        curvesToSvg(originals, outlines, toolpaths),
        "image/svg+xml",
      );
    }
  };

  const prompt = commandPrompt(tool, draft, selected, params, compensated?.error);

  return (
    <TooltipProvider>
      <div className="flex min-h-full flex-1 flex-col bg-background">
        <header className="flex flex-wrap items-center gap-3 border-b px-4 py-3">
          <div className="min-w-0">
            <p className="text-[11px] font-medium tracking-[0.22em] text-muted-foreground uppercase">
              Rhino companion
            </p>
            <h1 className="font-heading text-xl leading-tight tracking-tight">Kerf</h1>
          </div>
          <p className="hidden max-w-xl text-sm text-muted-foreground sm:block">
            Widen only the parts of an opening that fall under the minimum width, then
            account for torch kerf. Open centerlines still thicken the whole path.
          </p>
        </header>

        <div className="flex min-h-0 flex-1 flex-col lg:flex-row">
          <div className="flex min-h-0 min-w-0 flex-1 flex-col">
            <div className="border-b px-3 py-2">
              <Toolbar
                tool={tool}
                onTool={chooseTool}
                onFit={fit}
                onReset={reset}
                onDelete={() => {
                  if (!selectedId) return;
                  const next = curves.filter((c) => c.id !== selectedId);
                  setCurves(next);
                  selectCurve(next[0]?.id ?? null, next);
                }}
                onExportDxf={() => exportAll("dxf")}
                onExportSvg={() => exportAll("svg")}
                canDelete={Boolean(selectedId)}
              />
            </div>
            <div ref={stageRef} className="flex min-h-0 flex-1 flex-col">
              <Viewport
                curves={curves}
                selectedId={selectedId}
                params={params}
                compensated={compensated}
                camera={camera}
                tool={spacePan ? "pan" : tool}
                spacePan={spacePan}
                draft={draft}
                hoverWorld={hoverWorld}
                onCamera={setCamera}
                onSelect={selectCurve}
                onCurves={setCurves}
                onDraftPoint={onDraftPoint}
                onFinishDraft={finishDraft}
                onHover={setHoverWorld}
              />
            </div>
            <div className="flex items-center gap-3 border-t bg-zinc-900 px-3 py-1.5 font-mono text-[12px] text-zinc-100">
              <span className="text-amber-400">PlasmaKerf</span>
              <span className="min-w-0 truncate">{prompt}</span>
              {selected && (
                <span className="ml-auto hidden shrink-0 text-zinc-400 sm:inline">
                  L={polylineLength(selected.points, selected.closed).toFixed(1)} mm
                </span>
              )}
            </div>
          </div>
          <KerfPanel
            params={params}
            onChange={setParams}
            curves={curves}
            selected={selected}
            onSelect={(id) => selectCurve(id)}
            error={compensated?.error}
            singlePass={Boolean(compensated?.singlePass)}
            pinches={compensated?.pinches}
            selectedClosed={Boolean(selected?.closed)}
          />
        </div>
      </div>
    </TooltipProvider>
  );
}

function paramsFor(curve: Polyline, params: KerfParams): KerfParams {
  if (params.mode !== "slot" && !curve.closed) return { ...params, mode: "slot" };
  return params;
}

function commandPrompt(
  tool: Tool,
  draft: Draft | null,
  selected: Polyline | null,
  params: KerfParams,
  error?: string,
): string {
  if (error) return error;
  if (tool === "draw" || draft) {
    return "Click to add points · Enter finish · C close · Esc cancel";
  }
  if (tool === "pan") return "Drag to pan · scroll to zoom";
  if (!selected) return "Select a curve to compensate";
  if (params.mode === "slot") {
    return `${selected.name}  MinWidth=${params.minWidth}  Kerf=${params.kerf}  Caps=${capLabel(params.cap)}`;
  }
  return `${selected.name}  ${params.mode}  Kerf=${params.kerf}`;
}

function capLabel(cap: KerfParams["cap"]): string {
  if (cap === "butt") return "Flat";
  return cap[0].toUpperCase() + cap.slice(1);
}
