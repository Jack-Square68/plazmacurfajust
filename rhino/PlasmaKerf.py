"""
PlasmaKerf — offset curves for plasma kerf and minimum slot width.

Click a curve (or pre-select one), then adjust MinWidth / Kerf in the
command line. A live black preview is the finished cut; orange is the
torch centerline. Enter bakes the result onto layers.

Rhino 7 / 8
-----------
Drag this file onto the Rhino window, or:

    _-RunPythonScript "<path>/PlasmaKerf.py"

Optional alias:
    Options > Aliases > New
    Alias:   PlasmaKerf
    Command: -_RunPythonScript "<path>/PlasmaKerf.py"

Modes
-----
Slot  Open centerline: thicken the whole path to MinWidth with end caps.
      Closed opening: widen only stretches narrower than MinWidth; already
      wide areas stay as drawn. Torch path is inset by Kerf/2.
Part  Closed profile kept as the finished part. Torch offset outside.
Hole  Closed profile kept as the finished hole. Torch offset inside.
"""

from __future__ import print_function

import math

import Rhino
import Rhino.DocObjects as rd
import Rhino.Geometry as rg
import Rhino.Input.Custom as ric
import scriptcontext as sc
import System
import System.Drawing
from System.Drawing import Color


CMD_NAME = "PlasmaKerf"
LAYER_OUTLINE = "Kerf Outline"
LAYER_TOOLPATH = "Kerf Toolpath"

CAP_NAMES = ["Round", "Flat", "Square"]
CORNER_NAMES = ["Round", "Sharp", "Chamfer", "Smooth"]
MODE_NAMES = ["Slot", "Part", "Hole"]
OUTPUT_NAMES = ["Both", "Outline", "Toolpath"]


def _as_list(values):
    if values is None:
        return []
    try:
        return [v for v in values]
    except TypeError:
        return [values]


def _tol():
    try:
        return max(sc.doc.ModelAbsoluteTolerance, 1e-4)
    except Exception:
        return 0.001


def _unitize(vec):
    v = rg.Vector3d(vec)
    if not v.Unitize():
        return None
    return v


def _ensure_layer(name, color):
    for i in range(sc.doc.Layers.Count):
        layer = sc.doc.Layers[i]
        if layer.Name == name and not layer.IsDeleted:
            return i
    layer = rd.Layer()
    layer.Name = name
    layer.Color = color
    return sc.doc.Layers.Add(layer)


def _attrs(layer_name, color):
    attrs = rd.ObjectAttributes()
    attrs.LayerIndex = _ensure_layer(layer_name, color)
    attrs.ObjectColor = color
    attrs.ColorSource = rd.ObjectColorSource.ColorFromLayer
    return attrs


def _curve_plane(curve):
    ok, plane = curve.TryGetPlane(_tol() * 10)
    if ok:
        return plane
    view = sc.doc.Views.ActiveView
    if view:
        return view.ActiveViewport.ConstructionPlane()
    return rg.Plane.WorldXY


def _corner_style(name):
    table = {
        "Round": rg.CurveOffsetCornerStyle.Round,
        "Sharp": rg.CurveOffsetCornerStyle.Sharp,
        "Chamfer": rg.CurveOffsetCornerStyle.Chamfer,
        "Smooth": rg.CurveOffsetCornerStyle.Smooth,
    }
    return table.get(name, rg.CurveOffsetCornerStyle.Round)


def _offset(curve, plane, distance, corners):
    if abs(distance) < 1e-9:
        dup = curve.DuplicateCurve()
        return [dup] if dup else []
    try:
        result = curve.Offset(plane, distance, _tol(), _corner_style(corners))
    except Exception:
        result = None
    return [c for c in _as_list(result) if c is not None]


def _join(curves):
    if not curves:
        return []
    joined = rg.Curve.JoinCurves(curves, _tol() * 2, False)
    got = _as_list(joined)
    return got if got else list(curves)


def _round_cap(center, a, b, outward):
    radius = center.DistanceTo(a)
    if radius < _tol():
        return rg.LineCurve(a, b)
    direction = _unitize(outward)
    if direction is None:
        return rg.LineCurve(a, b)
    mid = center + direction * radius
    try:
        arc = rg.Arc(a, mid, b)
        if arc.IsValid and arc.Length > _tol():
            return rg.ArcCurve(arc)
    except Exception:
        pass
    return rg.LineCurve(a, b)


def _flat_cap(a, b):
    return rg.LineCurve(a, b)


def _square_cap(center, a, b, outward):
    direction = _unitize(outward)
    if direction is None:
        return _flat_cap(a, b)
    radius = center.DistanceTo(a)
    aa = a + direction * radius
    bb = b + direction * radius
    return _join([rg.LineCurve(a, aa), rg.LineCurve(aa, bb), rg.LineCurve(bb, b)])


def _orient_from_start(curve, start_pt):
    if curve.PointAtStart.DistanceTo(start_pt) > curve.PointAtEnd.DistanceTo(start_pt):
        curve.Reverse()
    return curve


def _make_cap(style, center, a, b, outward):
    if style == "Flat":
        return [_flat_cap(a, b)]
    if style == "Square":
        piece = _square_cap(center, a, b, outward)
        return piece if isinstance(piece, list) else [piece]
    return [_round_cap(center, a, b, outward)]


def _thicken_open(curve, half, corners, caps):
    plane = _curve_plane(curve)
    lefts = _offset(curve, plane, half, corners)
    rights = _offset(curve, plane, -half, corners)
    if not lefts or not rights:
        return []
    left = _orient_from_start(lefts[0].DuplicateCurve(), curve.PointAtStart)
    right = _orient_from_start(rights[0].DuplicateCurve(), curve.PointAtStart)
    right.Reverse()
    t0 = curve.Domain.Min
    t1 = curve.Domain.Max
    tan0 = _unitize(curve.TangentAt(t0))
    tan1 = _unitize(curve.TangentAt(t1))
    if tan0 is None or tan1 is None:
        return []
    cap_end = _make_cap(caps, curve.PointAtEnd, left.PointAtEnd, right.PointAtStart, tan1)
    cap_start = _make_cap(caps, curve.PointAtStart, right.PointAtEnd, left.PointAtStart, -tan0)
    pieces = [left] + cap_end + [right] + cap_start
    return _join(pieces)


def _thicken_closed(curve, half, corners):
    plane = _curve_plane(curve)
    a = _offset(curve, plane, half, corners)
    b = _offset(curve, plane, -half, corners)
    return a + b


def _signed_offset(curve, distance, corners, outward):
    plane = _curve_plane(curve)
    dist = abs(distance)
    if dist < 1e-9:
        dup = curve.DuplicateCurve()
        return [dup] if dup else []
    orientation = curve.ClosedCurveOrientation(plane)
    positive_is_out = orientation != rg.CurveOrientation.Clockwise
    if outward != positive_is_out:
        dist = -dist
    else:
        dist = dist
    return _offset(curve, plane, dist, corners)


def _signed_area_approx(curve):
    try:
        amp = curve.ToPolyline(_tol(), _tol(), 0.0, 0.0)
        if amp is None:
            return 0.0
        poly = amp.ToPolyline() if hasattr(amp, "ToPolyline") else amp
        if poly is None or poly.Count < 3:
            return 0.0
        area = 0.0
        count = poly.Count - 1 if poly.Count > 1 and poly[0].DistanceTo(poly[poly.Count - 1]) < _tol() else poly.Count
        for i in range(count):
            a = poly[i]
            b = poly[(i + 1) % count]
            area += a.X * b.Y - b.X * a.Y
        return area * 0.5
    except Exception:
        try:
            return abs(curve.GetBoundingBox(False).Area) * 0.25
        except Exception:
            return 0.0


def _inward_at(curve, t, plane):
    tan = _unitize(curve.TangentAt(t))
    if tan is None:
        return None
    inward = rg.Vector3d.CrossProduct(plane.Normal, tan)
    if not inward.Unitize():
        return None
    try:
        orientation = curve.ClosedCurveOrientation(plane)
    except Exception:
        orientation = rg.CurveOrientation.Undefined
    if orientation == rg.CurveOrientation.Clockwise:
        inward.Reverse()
    return inward


def _ray_width(curve, origin, direction):
    if direction is None:
        return None
    try:
        box = curve.GetBoundingBox(False)
        span = box.Diagonal.Length
    except Exception:
        span = 1000.0
    span = max(span, 1.0)
    start = origin + direction * (_tol() * 8)
    far = origin + direction * (span * 2.0 + 1.0)
    probe = rg.LineCurve(start, far)
    try:
        events = rg.Intersect.Intersection.CurveCurve(curve, probe, _tol(), _tol())
    except Exception:
        events = None
    if events is None or events.Count == 0:
        return None
    best = None
    for i in range(events.Count):
        try:
            pt = events[i].PointA
        except Exception:
            continue
        d = origin.DistanceTo(pt)
        if d < _tol() * 12:
            continue
        if best is None or d < best:
            best = d
    return best


def _inside_clearance(curve, origin, skip_t, plane):
    """Nearest other wall whose connecting segment stays inside the opening."""
    try:
        length = curve.GetLength()
    except Exception:
        return None
    if length < _tol() * 4:
        return None
    window = max(length * 0.04, min(length * 0.12, 4.0))
    steps = max(48, int(math.ceil(length / 0.45)))
    best = None
    for i in range(steps):
        t = curve.Domain.ParameterAt(i / float(steps)) if hasattr(curve.Domain, "ParameterAt") else (
            curve.Domain.Min + (curve.Domain.Max - curve.Domain.Min) * (i / float(steps))
        )
        try:
            along = abs(curve.GetLength(curve.Domain.Min, t) - curve.GetLength(curve.Domain.Min, skip_t))
            along = min(along, length - along)
        except Exception:
            along = abs(t - skip_t)
        if along < window:
            continue
        pt = curve.PointAt(t)
        d = origin.DistanceTo(pt)
        if d < _tol() * 8:
            continue
        if best is not None and d >= best:
            continue
        mid = rg.Point3d(
            (origin.X + pt.X) * 0.5,
            (origin.Y + pt.Y) * 0.5,
            (origin.Z + pt.Z) * 0.5,
        )
        try:
            contain = curve.Contains(mid, plane, _tol())
        except Exception:
            contain = None
        if contain == rg.PointContainment.Inside:
            best = d
    return best


def _union_closed(curves):
    usable = [c for c in curves if c is not None]
    if not usable:
        return []
    if len(usable) == 1:
        return usable
    try:
        joined = rg.Curve.CreateBooleanUnion(usable, _tol())
        got = [c for c in _as_list(joined) if c is not None]
        if got:
            return got
    except Exception:
        pass
    return usable


def _fair_closed_curve(pts, plane):
    cleaned = []
    for pt in pts:
        if not cleaned or cleaned[-1].DistanceTo(pt) > 0.03:
            cleaned.append(pt)
    if len(cleaned) > 2 and cleaned[0].DistanceTo(cleaned[-1]) <= 0.03:
        cleaned.pop()
    if len(cleaned) < 3:
        return None
    curr = list(cleaned)
    for _ in range(2):
        nxt = []
        n = len(curr)
        for i in range(n):
            a = curr[(i - 1) % n]
            b = curr[i]
            c = curr[(i + 1) % n]
            nxt.append(rg.Point3d(
                (a.X + b.X * 2.0 + c.X) / 4.0,
                (a.Y + b.Y * 2.0 + c.Y) / 4.0,
                (a.Z + b.Z * 2.0 + c.Z) / 4.0,
            ))
        curr = nxt
    closed_pts = list(curr) + [curr[0]]
    try:
        fair = rg.Curve.CreateInterpolatedCurve(closed_pts, 3)
        if fair is not None and fair.IsValid:
            return fair
    except Exception:
        pass
    try:
        return rg.PolylineCurve(closed_pts)
    except Exception:
        return None


def _smooth_closed_values(values, sigma):
    if not values or sigma < 0.35:
        return list(values)
    radius = max(1, int(math.ceil(sigma * 3)))
    kernel = []
    total = 0.0
    for i in range(-radius, radius + 1):
        k = math.exp(-(i * i) / (2.0 * sigma * sigma))
        kernel.append(k)
        total += k
    n = len(values)
    out = []
    for i in range(n):
        acc = 0.0
        for j in range(-radius, radius + 1):
            acc += values[(i + j) % n] * kernel[j + radius]
        out.append(acc / total)
    return out


def _ensure_min_width(curve, min_width, corners):
    """Parallel-offset the original curve only where it is thinner than min_width."""
    plane = _curve_plane(curve)
    try:
        length = curve.GetLength()
    except Exception:
        length = 0.0
    if length < _tol() * 4:
        return [curve.DuplicateCurve()], 0, []

    spacing = min(0.4, max(0.16, min_width / 24.0))
    steps = max(64, int(math.ceil(length / spacing)))
    domain = curve.Domain
    samples = []
    for i in range(steps):
        t = domain.ParameterAt(i / float(steps)) if hasattr(domain, "ParameterAt") else (
            domain.Min + (domain.Max - domain.Min) * (i / float(steps))
        )
        pt = curve.PointAt(t)
        inward = _inward_at(curve, t, plane)
        if inward is None:
            continue
        width = _ray_width(curve, pt, inward)
        inside = _inside_clearance(curve, pt, t, plane)
        if width is None and inside is None:
            width = min_width
        elif width is None:
            width = inside
        elif inside is not None:
            width = min(width, inside)
        samples.append((pt, inward, width, t))

    raw = []
    for _pt, _inward, width, _t in samples:
        if width is None:
            raw.append(0.0)
        else:
            raw.append(max(0.0, (min_width - width) * 0.5))
    if not any(d > 1e-4 for d in raw):
        return [curve.DuplicateCurve()], 0, []

    sigma = max(1.2, (min_width * 0.55) / spacing)
    deltas = _smooth_closed_values(raw, sigma)
    pinches = 0
    in_run = False
    for d in deltas:
        thin = d > 0.04
        if thin and not in_run:
            pinches += 1
            in_run = True
        elif not thin:
            in_run = False
    if deltas and deltas[0] > 0.04 and deltas[-1] > 0.04 and pinches >= 2:
        pinches -= 1

    moved = []
    n = len(samples)
    for i in range(n):
        pt, inward, _width, _t = samples[i]
        delta = deltas[i]
        outward = rg.Vector3d(-inward.X, -inward.Y, -inward.Z)
        if delta <= 1e-6:
            moved.append(pt)
            continue
        probe = pt + outward * min(0.2, delta)
        try:
            contain = curve.Contains(probe, plane, _tol())
        except Exception:
            contain = None
        if contain == rg.PointContainment.Inside:
            outward.Reverse()
        if not outward.Unitize():
            moved.append(pt)
            continue
        moved.append(pt + outward * delta)

    outline_curve = _fair_closed_curve(moved, plane)
    if outline_curve is None:
        return [curve.DuplicateCurve()], pinches, []
    if not outline_curve.IsClosed:
        try:
            outline_curve.MakeClosed(_tol() * 4)
        except Exception:
            pass
    return [outline_curve], pinches, []


def _inset_closed(curves, distance, corners):
    out = []
    for curve in curves:
        out.extend(_signed_offset(curve, distance, corners, False))
    return [c for c in out if c]


def compensate_curve(curve, min_width, kerf, caps, corners, mode):
    """Returns (outline_curves, toolpath_curves, error_or_None)."""
    if curve is None:
        return [], [], "Invalid curve."

    crv = curve.DuplicateCurve()
    if crv is None:
        return [], [], "Could not copy curve."

    if mode == "Slot":
        half = min_width * 0.5
        if half <= 0:
            return [], [], "MinWidth must be greater than 0."
        if crv.IsClosed:
            area = abs(_signed_area_approx(crv))
            if area < min_width * min_width * 0.2:
                outline = _thicken_closed(crv, half, corners)
                pinches = 0
                thin_center = []
            else:
                outline, pinches, thin_center = _ensure_min_width(crv, min_width, corners)
        else:
            outline = _thicken_open(crv, half, corners, caps)
            pinches = 0
            thin_center = []
        if not outline:
            return [], [], "Offset failed. Try Round corners or simplify the curve."
        tool_width = min_width - kerf
        if crv.IsClosed and pinches is not None:
            half_kerf = kerf * 0.5
            if half_kerf <= 0.02:
                toolpath = [c.DuplicateCurve() for c in outline]
            elif tool_width > 0.04:
                toolpath = _inset_closed(outline, half_kerf, corners)
                if not toolpath:
                    toolpath = [c.DuplicateCurve() for c in outline]
            else:
                inset = _inset_closed(outline, half_kerf, corners)
                toolpath = inset + [c for c in thin_center if c]
                if not toolpath:
                    toolpath = [crv.DuplicateCurve()]
        elif tool_width > 0.04:
            tool_half = tool_width * 0.5
            if crv.IsClosed:
                toolpath = _thicken_closed(crv, tool_half, corners)
            else:
                toolpath = _thicken_open(crv, tool_half, corners, caps)
        else:
            toolpath = [crv.DuplicateCurve()]
        return outline, [c for c in toolpath if c], None

    if not crv.IsClosed:
        return [], [], "Part and Hole modes need a closed curve."

    outline = [crv.DuplicateCurve()]
    half_kerf = kerf * 0.5
    if half_kerf <= 0:
        return outline, [crv.DuplicateCurve()], None
    outward = mode == "Part"
    toolpath = _signed_offset(crv, half_kerf, corners, outward)
    if not toolpath:
        if mode == "Hole":
            return outline, [], "Hole is smaller than the kerf."
        return outline, [], "Could not offset this profile."
    return outline, toolpath, None


class KerfConduit(Rhino.Display.DisplayConduit):
    def __init__(self):
        try:
            super(KerfConduit, self).__init__()
        except Exception:
            pass
        self.outlines = []
        self.toolpaths = []
        self.show_outline = True
        self.show_toolpath = True

    def CalculateBoundingBox(self, e):
        for curve in self.outlines + self.toolpaths:
            try:
                e.IncludeBoundingBox(curve.GetBoundingBox(False))
            except Exception:
                pass

    def _draw(self, e):
        if self.show_outline:
            for curve in self.outlines:
                e.Display.DrawCurve(curve, Color.Black, 3)
        if self.show_toolpath:
            for curve in self.toolpaths:
                e.Display.DrawCurve(curve, Color.FromArgb(217, 119, 6), 2)

    def DrawOverlay(self, e):
        self._draw(e)

    def PostDrawObjects(self, e):
        self._draw(e)


def _select_curves():
    go = ric.GetObject()
    go.SetCommandPrompt("Select curves to compensate for plasma kerf")
    go.GeometryFilter = rd.ObjectType.Curve
    go.SubObjectSelect = False
    go.GroupSelect = True
    go.EnablePreSelect(True, True)
    go.GetMultiple(1, 0)
    if go.CommandResult() != Rhino.Commands.Result.Success:
        return []
    curves = []
    for i in range(go.ObjectCount):
        obj = go.Object(i)
        geom = obj.Curve()
        if geom:
            curves.append(geom.DuplicateCurve())
    return curves


def _update_preview(conduit, curves, min_width, kerf, caps, corners, mode, output):
    outlines = []
    toolpaths = []
    errors = []
    for curve in curves:
        out_c, tool_c, err = compensate_curve(curve, min_width, kerf, caps, corners, mode)
        outlines.extend(out_c)
        toolpaths.extend(tool_c)
        if err:
            errors.append(err)
    conduit.outlines = outlines
    conduit.toolpaths = toolpaths
    conduit.show_outline = output in ("Both", "Outline")
    conduit.show_toolpath = output in ("Both", "Toolpath")
    sc.doc.Views.Redraw()
    return errors


def _bake(outlines, toolpaths, output):
    ids = []
    if output in ("Both", "Outline"):
        attrs = _attrs(LAYER_OUTLINE, Color.Black)
        for curve in outlines:
            obj_id = sc.doc.Objects.AddCurve(curve, attrs)
            if obj_id != System.Guid.Empty:
                ids.append(obj_id)
    if output in ("Both", "Toolpath"):
        attrs = _attrs(LAYER_TOOLPATH, Color.FromArgb(217, 119, 6))
        for curve in toolpaths:
            obj_id = sc.doc.Objects.AddCurve(curve, attrs)
            if obj_id != System.Guid.Empty:
                ids.append(obj_id)
    sc.doc.Objects.UnselectAll()
    for obj_id in ids:
        sc.doc.Objects.Select(obj_id)
    sc.doc.Views.Redraw()
    return len(ids)


def RunCommand():
    curves = _select_curves()
    if not curves:
        return Rhino.Commands.Result.Cancel

    min_width = ric.OptionDouble(6.0, 0.01, 100000.0)
    kerf = ric.OptionDouble(1.5, 0.0, 100000.0)
    cap_index = [0]
    corner_index = [0]
    mode_index = [0]
    output_index = [0]

    conduit = KerfConduit()
    conduit.Enabled = True

    def refresh():
        return _update_preview(
            conduit,
            curves,
            min_width.CurrentValue,
            kerf.CurrentValue,
            CAP_NAMES[cap_index[0]],
            CORNER_NAMES[corner_index[0]],
            MODE_NAMES[mode_index[0]],
            OUTPUT_NAMES[output_index[0]],
        )

    refresh()
    result = Rhino.Commands.Result.Cancel
    try:
        while True:
            getter = ric.GetOption()
            getter.SetCommandPrompt(
                "Adjust plasma kerf. Enter to bake black outline / orange torch path"
            )
            getter.AcceptNothing(True)
            getter.AddOptionDouble("MinWidth", min_width)
            getter.AddOptionDouble("Kerf", kerf)
            getter.AddOptionList("Caps", CAP_NAMES, cap_index[0])
            getter.AddOptionList("Corners", CORNER_NAMES, corner_index[0])
            getter.AddOptionList("Mode", MODE_NAMES, mode_index[0])
            getter.AddOptionList("Output", OUTPUT_NAMES, output_index[0])
            getter_result = getter.Get()

            if getter_result == Rhino.Input.GetResult.Cancel:
                result = Rhino.Commands.Result.Cancel
                break

            if getter_result == Rhino.Input.GetResult.Option:
                opt = getter.Option()
                name = opt.EnglishName if opt else ""
                if name == "Caps":
                    cap_index[0] = opt.CurrentListOptionIndex
                elif name == "Corners":
                    corner_index[0] = opt.CurrentListOptionIndex
                elif name == "Mode":
                    mode_index[0] = opt.CurrentListOptionIndex
                elif name == "Output":
                    output_index[0] = opt.CurrentListOptionIndex
                errors = refresh()
                if errors:
                    print("{0}: {1}".format(CMD_NAME, errors[0]))
                continue

            errors = refresh()
            if errors and not conduit.outlines and not conduit.toolpaths:
                print("{0}: {1}".format(CMD_NAME, errors[0]))
                result = Rhino.Commands.Result.Failure
                break

            count = _bake(
                conduit.outlines,
                conduit.toolpaths,
                OUTPUT_NAMES[output_index[0]],
            )
            print(
                "{0}: baked {1} curve(s)  MinWidth={2}  Kerf={3}  Mode={4}".format(
                    CMD_NAME,
                    count,
                    min_width.CurrentValue,
                    kerf.CurrentValue,
                    MODE_NAMES[mode_index[0]],
                )
            )
            result = Rhino.Commands.Result.Success
            break
    finally:
        conduit.Enabled = False
        sc.doc.Views.Redraw()

    return result


if __name__ == "__main__":
    RunCommand()
