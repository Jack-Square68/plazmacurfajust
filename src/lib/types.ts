export type Point = { x: number; y: number };

export type JoinStyle = "round" | "miter" | "square";
export type CapStyle = "round" | "square" | "butt";
export type CutMode = "slot" | "part" | "hole";
export type OutputMode = "outline" | "toolpath" | "both";
export type Tool = "select" | "draw" | "pan";

export type Polyline = {
  id: string;
  name: string;
  points: Point[];
  closed: boolean;
};

export type KerfParams = {
  minWidth: number;
  kerf: number;
  join: JoinStyle;
  cap: CapStyle;
  mode: CutMode;
  output: OutputMode;
};

export type Compensated = {
  outline: Point[][];
  toolpath: Point[][];
  singlePass: boolean;
  error?: string;
  /** Narrow stretches that were widened to min width. 0 = already wide enough. */
  pinches?: number;
};

export const DEFAULT_PARAMS: KerfParams = {
  minWidth: 6,
  kerf: 1.5,
  join: "round",
  cap: "round",
  mode: "slot",
  output: "both",
};
