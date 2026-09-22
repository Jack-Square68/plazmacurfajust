"use client";

import { Button } from "@/components/ui/button";
import {
  Tooltip,
  TooltipContent,
  TooltipTrigger,
} from "@/components/ui/tooltip";
import { cn } from "@/lib/utils";
import type { Tool } from "@/lib/types";
import {
  Download,
  FileCode2,
  Hand,
  MousePointer2,
  PenLine,
  RotateCcw,
  Trash2,
  Upload,
} from "lucide-react";

type Props = {
  tool: Tool;
  onTool: (tool: Tool) => void;
  onFit: () => void;
  onReset: () => void;
  onDelete: () => void;
  onExportDxf: () => void;
  onExportSvg: () => void;
  onUpload: (files: FileList | File[]) => void;
  canDelete: boolean;
};

function Tip({
  label,
  children,
}: {
  label: string;
  children: React.ReactElement;
}) {
  return (
    <Tooltip>
      <TooltipTrigger asChild>{children}</TooltipTrigger>
      <TooltipContent>{label}</TooltipContent>
    </Tooltip>
  );
}

export function Toolbar({
  tool,
  onTool,
  onFit,
  onReset,
  onDelete,
  onExportDxf,
  onExportSvg,
  onUpload,
  canDelete,
}: Props) {
  return (
    <div className="flex flex-wrap items-center gap-1.5">
      <div className="flex items-center rounded-lg border border-border bg-card p-0.5">
        <ToolBtn
          active={tool === "select"}
          label="Select (V)"
          onClick={() => onTool("select")}
        >
          <MousePointer2 />
        </ToolBtn>
        <ToolBtn
          active={tool === "draw"}
          label="Draw polyline (D)"
          onClick={() => onTool("draw")}
        >
          <PenLine />
        </ToolBtn>
        <ToolBtn
          active={tool === "pan"}
          label="Pan (H or hold Space)"
          onClick={() => onTool("pan")}
        >
          <Hand />
        </ToolBtn>
      </div>
      <Tip label="Fit all curves">
        <Button variant="outline" size="sm" onClick={onFit}>
          Fit
        </Button>
      </Tip>
      <Tip label="Reload demo curves">
        <Button variant="outline" size="icon-sm" onClick={onReset}>
          <RotateCcw />
        </Button>
      </Tip>
      <Tip label="Delete selected">
        <Button
          variant="outline"
          size="icon-sm"
          onClick={onDelete}
          disabled={!canDelete}
        >
          <Trash2 />
        </Button>
      </Tip>
      <div className="ml-auto flex items-center gap-1.5">
        <Tip label="Upload SVG or DXF openings">
          <Button variant="outline" size="sm" asChild>
            <label className="cursor-pointer">
              <Upload />
              Upload
              <input
                type="file"
                accept=".svg,.dxf,image/svg+xml"
                className="hidden"
                multiple
                onChange={(e) => {
                  if (e.target.files?.length) onUpload(e.target.files);
                  e.target.value = "";
                }}
              />
            </label>
          </Button>
        </Tip>
        <Button variant="outline" size="sm" onClick={onExportSvg}>
          <Download />
          SVG
        </Button>
        <Button size="sm" onClick={onExportDxf}>
          <FileCode2 />
          DXF for Rhino
        </Button>
      </div>
    </div>
  );
}

function ToolBtn({
  active,
  label,
  onClick,
  children,
}: {
  active: boolean;
  label: string;
  onClick: () => void;
  children: React.ReactNode;
}) {
  return (
    <Tip label={label}>
      <Button
        variant={active ? "secondary" : "ghost"}
        size="icon-sm"
        onClick={onClick}
        className={cn(active && "bg-muted")}
        aria-pressed={active}
      >
        {children}
      </Button>
    </Tip>
  );
}
