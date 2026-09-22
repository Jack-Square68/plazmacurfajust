"use client";

import { formatMm } from "@/lib/geometry";
import type { CutMode, KerfParams } from "@/lib/types";

type Props = {
  params: KerfParams;
};

export function SectionDiagram({ params }: Props) {
  const w = 220;
  const h = 92;
  const minW = Math.max(params.minWidth, 0.1);
  const kerf = Math.max(params.kerf, 0);
  const scale = 140 / Math.max(minW, kerf, 4);
  const slotW = minW * scale;
  const kerfW = Math.min(kerf * scale, slotW);
  const cx = w / 2;
  const cy = 44;
  const slotLeft = cx - slotW / 2;
  const slotRight = cx + slotW / 2;
  const torch = kerfW;
  const single = params.mode === "slot" && minW <= kerf + 0.02;

  return (
    <div className="rounded-lg border border-border bg-zinc-100 px-2 py-2">
      <svg viewBox={`0 0 ${w} ${h}`} className="h-[92px] w-full" aria-hidden>
        <text x="8" y="14" fill="#52525b" fontSize="10" fontFamily="ui-sans-serif">
          {caption(params.mode, single)}
        </text>
        {params.mode === "slot" ? (
          <>
            <rect
              x={slotLeft}
              y={cy - 16}
              width={slotW}
              height={32}
              rx="16"
              fill="#111827"
              fillOpacity="0.08"
              stroke="#111"
              strokeWidth="1.6"
            />
            <line
              x1={slotLeft + 8}
              y1={cy}
              x2={slotRight - 8}
              y2={cy}
              stroke="#c40d0d"
              strokeWidth="2"
            />
            {!single && (
              <>
                <rect
                  x={cx - torch / 2}
                  y={cy - 7}
                  width={torch}
                  height={14}
                  rx="7"
                  fill="none"
                  stroke="#d97706"
                  strokeWidth="1.3"
                  strokeDasharray="3 2"
                />
                <Dim
                  x1={cx - torch / 2}
                  x2={cx + torch / 2}
                  y={cy + 28}
                  label={`${formatMm(kerf)} kerf`}
                  color="#b45309"
                />
              </>
            )}
            <Dim
              x1={slotLeft}
              x2={slotRight}
              y={single ? cy + 28 : cy - 28}
              label={`${formatMm(minW)} min`}
              color="#111"
            />
          </>
        ) : (
          <>
            <rect
              x={cx - 54}
              y={cy - 18}
              width={108}
              height={36}
              fill="none"
              stroke={params.mode === "part" ? "#c40d0d" : "#111"}
              strokeWidth="1.6"
            />
            <rect
              x={cx - 54 - (params.mode === "part" ? kerfW / 2 : -kerfW / 2)}
              y={cy - 18 - (params.mode === "part" ? kerfW / 2 : -kerfW / 2)}
              width={108 + (params.mode === "part" ? kerfW : -kerfW)}
              height={36 + (params.mode === "part" ? kerfW : -kerfW)}
              fill="none"
              stroke="#d97706"
              strokeWidth="1.3"
              strokeDasharray="3 2"
            />
            <text x={cx} y={cy + 4} textAnchor="middle" fill="#c40d0d" fontSize="10">
              CAD
            </text>
            <text x={cx} y={h - 8} textAnchor="middle" fill="#b45309" fontSize="10">
              torch offset {formatMm(kerf / 2)} mm each side
            </text>
          </>
        )}
      </svg>
    </div>
  );
}

function Dim({
  x1,
  x2,
  y,
  label,
  color,
}: {
  x1: number;
  x2: number;
  y: number;
  label: string;
  color: string;
}) {
  const mid = (x1 + x2) / 2;
  return (
    <g stroke={color} fill={color}>
      <line x1={x1} y1={y - 3} x2={x1} y2={y + 3} strokeWidth="1" />
      <line x1={x2} y1={y - 3} x2={x2} y2={y + 3} strokeWidth="1" />
      <line x1={x1} y1={y} x2={x2} y2={y} strokeWidth="1" />
      <text
        x={mid}
        y={y - 5}
        textAnchor="middle"
        fontSize="9"
        fontFamily="ui-sans-serif"
        stroke="none"
      >
        {label}
      </text>
    </g>
  );
}

function caption(mode: CutMode, single: boolean): string {
  if (mode === "part") return "Finished part (red) · torch outside (orange)";
  if (mode === "hole") return "Finished hole (black) · torch inside (orange)";
  if (single) return "Single pass — slot will be kerf-wide";
  return "Black outline stays put except where width is under min";
}
