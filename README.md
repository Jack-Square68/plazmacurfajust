# Kerf — plasma curve compensation for Rhino

Click a curve. Get a black cut outline that only grows where the opening is under the **minimum width**, then inset it by **plasma kerf** so the finished opening matches what you meant.

This repo is two pieces of the same tool:

1. **`rhino/PlasmaKerf.py`** — the command you run inside Rhino 7 / 8
2. **This web app** — same geometry, so you can try the red → black offset before dropping the script into Rhino

## What it does

Plasma burns a slot about as wide as the torch kerf. A skinny red centerline on the drawing is not a finished opening.

| You draw (red) | Tool builds (black) |
| --- | --- |
| Open centerline | Stadium / capsule of **Min width**, round or square caps (the whole path is under min width) |
| Closed opening | Only stretches **narrower than Min width** are widened. Already-wide curves stay as drawn; thin tapers get a **smooth** min-width capsule with round ends |
| Part profile | Torch path **outside** by kerf / 2 |
| Hole profile | Torch path **inside** by kerf / 2 |

The orange dashed curve is the torch centerline. If min width ≤ kerf, it just follows the red line (single pass).

## Run the web preview

```bash
npm install
npm run dev
```

Open [http://127.0.0.1:43147](http://127.0.0.1:43147).

- Click a red curve
- Drag **Minimum width** and **Kerf width**
- Draw your own polyline with the pen (`D`), Enter to finish, `C` to close
- Export **DXF for Rhino** (layers `ORIGINAL`, `KERF_OUTLINE`, `KERF_TOOLPATH`, units mm)

## Install in Rhino

1. Copy `rhino/PlasmaKerf.py` somewhere stable.
2. In Rhino: **Tools → Options → Aliases → New**
   - Alias: `PlasmaKerf`
   - Command: `-_RunPythonScript "C:\full\path\to\PlasmaKerf.py"`
3. Or drag the `.py` file onto the Rhino viewport.

Rhino 8 also works from **ScriptEditor**: open the file and Run.

### Command

```
PlasmaKerf
Select curves to compensate for plasma kerf
MinWidth=6  Kerf=1.5  Caps=Round  Corners=Round  Mode=Slot  Output=Both
```

Enter bakes:

- Layer **Kerf Outline** (black) — finished cut
- Layer **Kerf Toolpath** (orange) — torch path

Pre-select curves, then run the alias, same as any Rhino command.

### Typical plasma starting points

| Material / process | Kerf to try |
| --- | --- |
| Thin sheet, fine nozzle | 1.0–1.5 mm |
| Mid steel, 45 A | 1.5–2.0 mm |
| Thick plate, high current | 2.5–4.0 mm |

Measure a scrap coupon. Set **Min width** to the smallest slot you still want after the torch has eaten the kerf.

## Units

Millimetres in the web app and in the DXF (`$INSUNITS = 4`). In Rhino the script uses the document unit system — if the file is in inches, type inches.
