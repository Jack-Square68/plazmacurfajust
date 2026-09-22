"use client";

import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Separator } from "@/components/ui/separator";
import { Slider } from "@/components/ui/slider";
import { ToggleGroup, ToggleGroupItem } from "@/components/ui/toggle-group";
import { SectionDiagram } from "@/components/section-diagram";
import { formatMm } from "@/lib/geometry";
import { PRESET_HINTS } from "@/lib/presets";
import type { CapStyle, CutMode, JoinStyle, KerfParams, OutputMode, Polyline } from "@/lib/types";

type Props = {
  params: KerfParams;
  onChange: (next: KerfParams) => void;
  curves: Polyline[];
  selected: Polyline | null;
  onSelect: (id: string) => void;
  error?: string;
  singlePass: boolean;
  pinches?: number;
  selectedClosed?: boolean;
  importNote?: string | null;
};

export function KerfPanel({
  params,
  onChange,
  curves,
  selected,
  onSelect,
  error,
  singlePass,
  pinches,
  selectedClosed,
  importNote,
}: Props) {
  const slot = params.mode === "slot";

  return (
    <aside className="flex h-full min-h-0 w-full flex-col gap-4 overflow-y-auto bg-card p-4 lg:w-[320px] lg:shrink-0 lg:border-l">
      <div>
        <p className="text-[11px] font-medium tracking-[0.18em] text-muted-foreground uppercase">
          Parameters
        </p>
        <h2 className="mt-1 font-heading text-lg">Plasma kerf</h2>
        <p className="mt-1 text-sm text-muted-foreground">
          Upload SVG or DXF, or click a red curve. Closed openings only widen where
          they are under the minimum; the black outline is the finished cut.
        </p>
      </div>

      <div className="space-y-1.5">
        <Label>Cut type</Label>
        <ToggleGroup
          type="single"
          value={params.mode}
          onValueChange={(value) => {
            if (value) onChange({ ...params, mode: value as CutMode });
          }}
          variant="outline"
          size="sm"
          spacing={0}
          className="w-full"
        >
          <ToggleGroupItem value="slot" className="flex-1">
            Slot
          </ToggleGroupItem>
          <ToggleGroupItem value="part" className="flex-1">
            Part
          </ToggleGroupItem>
          <ToggleGroupItem value="hole" className="flex-1">
            Hole
          </ToggleGroupItem>
        </ToggleGroup>
        <p className="text-xs text-muted-foreground">
          {params.mode === "slot" &&
            "Open centerline: thicken the whole path. Closed opening: a smooth parallel offset of the original, only where it is under the minimum."}
          {params.mode === "part" &&
            "Keep the red profile as the finished part. Torch path sits outside by kerf/2."}
          {params.mode === "hole" &&
            "Keep the red profile as the finished hole. Torch path sits inside by kerf/2."}
        </p>
      </div>

      {slot && (
        <NumberParam
          label="Minimum width"
          unit="mm"
          value={params.minWidth}
          min={0.5}
          max={40}
          step={0.1}
          onChange={(minWidth) => onChange({ ...params, minWidth })}
        />
      )}

      <NumberParam
        label="Kerf width"
        unit="mm"
        value={params.kerf}
        min={0}
        max={12}
        step={0.05}
        onChange={(kerf) => onChange({ ...params, kerf })}
      />

      <SectionDiagram params={params} />

      {slot && selectedClosed && pinches === 0 && (
        <p className="rounded-lg bg-zinc-100 px-3 py-2 text-xs text-zinc-800 ring-1 ring-zinc-200">
          This opening is already at least {formatMm(params.minWidth)} mm. Wide areas
          stay as drawn; only kerf is applied.
        </p>
      )}

      {slot && selectedClosed && (pinches ?? 0) > 0 && (
        <p className="rounded-lg bg-emerald-50 px-3 py-2 text-xs text-emerald-950 ring-1 ring-emerald-200">
          Widened {pinches} narrow stretch{pinches === 1 ? "" : "es"} to{" "}
          {formatMm(params.minWidth)} mm. The rest of the opening is unchanged.
        </p>
      )}

      {singlePass && slot && (
        <p className="rounded-lg bg-amber-50 px-3 py-2 text-xs text-amber-950 ring-1 ring-amber-200">
          Minimum width is at or below kerf ({formatMm(params.kerf)} mm). The torch
          follows the red centerline; the opening will be about kerf-wide.
        </p>
      )}

      <Separator />

      <div className="space-y-1.5">
        <Label>End caps</Label>
        <ToggleGroup
          type="single"
          value={params.cap}
          onValueChange={(value) => {
            if (value) onChange({ ...params, cap: value as CapStyle });
          }}
          variant="outline"
          size="sm"
          spacing={0}
          className="w-full"
          disabled={!slot}
        >
          <ToggleGroupItem value="round" className="flex-1">
            Round
          </ToggleGroupItem>
          <ToggleGroupItem value="square" className="flex-1">
            Square
          </ToggleGroupItem>
          <ToggleGroupItem value="butt" className="flex-1">
            Flat
          </ToggleGroupItem>
        </ToggleGroup>
      </div>

      <div className="space-y-1.5">
        <Label>Corners</Label>
        <ToggleGroup
          type="single"
          value={params.join}
          onValueChange={(value) => {
            if (value) onChange({ ...params, join: value as JoinStyle });
          }}
          variant="outline"
          size="sm"
          spacing={0}
          className="w-full"
        >
          <ToggleGroupItem value="round" className="flex-1">
            Round
          </ToggleGroupItem>
          <ToggleGroupItem value="miter" className="flex-1">
            Sharp
          </ToggleGroupItem>
          <ToggleGroupItem value="square" className="flex-1">
            Chamfer
          </ToggleGroupItem>
        </ToggleGroup>
      </div>

      <div className="space-y-1.5">
        <Label>Show</Label>
        <ToggleGroup
          type="single"
          value={params.output}
          onValueChange={(value) => {
            if (value) onChange({ ...params, output: value as OutputMode });
          }}
          variant="outline"
          size="sm"
          spacing={0}
          className="w-full"
        >
          <ToggleGroupItem value="outline" className="flex-1">
            Black
          </ToggleGroupItem>
          <ToggleGroupItem value="toolpath" className="flex-1">
            Torch
          </ToggleGroupItem>
          <ToggleGroupItem value="both" className="flex-1">
            Both
          </ToggleGroupItem>
        </ToggleGroup>
      </div>

      <Separator />

      <div className="rounded-lg border border-border bg-muted/40 px-3 py-3">
        <p className="text-[11px] font-medium tracking-[0.16em] text-muted-foreground uppercase">
          Curves
        </p>
        <div className="mt-2 flex flex-col gap-1">
          {curves.map((curve) => {
            const active = curve.id === selected?.id;
            return (
              <button
                key={curve.id}
                type="button"
                onClick={() => onSelect(curve.id)}
                className={`flex items-center justify-between rounded-md px-2 py-1.5 text-left text-sm ${
                  active ? "bg-amber-100 text-amber-950" : "hover:bg-background"
                }`}
              >
                <span className="truncate font-medium">{curve.name}</span>
                <span className="ml-2 shrink-0 text-[10px] tracking-wide text-muted-foreground uppercase">
                  {curve.closed ? "closed" : "open"}
                </span>
              </button>
            );
          })}
        </div>
        {importNote && (
          <p className="mt-2 text-xs text-muted-foreground">{importNote}</p>
        )}
        {selected && PRESET_HINTS[selected.name] && (
          <p className="mt-2 text-xs text-muted-foreground">{PRESET_HINTS[selected.name]}</p>
        )}
        {!selected && (
          <p className="mt-2 text-xs text-muted-foreground">
            Click a red curve, click inside a closed profile, or draw one with the pen.
          </p>
        )}
        {error && <p className="mt-2 text-xs text-destructive">{error}</p>}
      </div>
    </aside>
  );
}

function NumberParam({
  label,
  unit,
  value,
  min,
  max,
  step,
  onChange,
}: {
  label: string;
  unit: string;
  value: number;
  min: number;
  max: number;
  step: number;
  onChange: (value: number) => void;
}) {
  return (
    <div className="space-y-2">
      <div className="flex items-center justify-between gap-2">
        <Label>{label}</Label>
        <div className="flex items-center gap-1">
          <Input
            type="number"
            className="h-7 w-[4.5rem] text-right tabular-nums"
            value={Number(value.toFixed(2))}
            min={min}
            max={max}
            step={step}
            onChange={(e) => {
              const n = Number(e.target.value);
              if (Number.isFinite(n)) onChange(Math.min(max, Math.max(min, n)));
            }}
          />
          <span className="text-xs text-muted-foreground">{unit}</span>
        </div>
      </div>
      <Slider
        min={min}
        max={max}
        step={step}
        value={[value]}
        onValueChange={(vals) => onChange(vals[0] ?? value)}
      />
    </div>
  );
}
