"""
PlasmaKerf — offset curves for plasma kerf and minimum slot width.

Click a curve (or pre-select one), then adjust MinWidth / Kerf in the
command line. A live black preview is the finished cut; orange is the
torch centerline. Enter bakes the result onto layers.

Rhino 7 / 8
-----------
Closed openings are sampled once to a polyline (capped) for width, then
the original curve is offset along its own normals and rebuilt as a
degree-3 NURBS. Rhino is not asked to CurveCurve / GetLength / Contains
on every sample, so a koru no longer locks the UI.

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

try:
    import Rhino
    import Rhino.DocObjects as rd
    import Rhino.Geometry as rg
    import Rhino.Input.Custom as ric
    import scriptcontext as sc
    import System
    import System.Drawing
    from System.Drawing import Color
    HAS_RHINO = True
except ImportError:
    HAS_RHINO = False
    Rhino = rd = rg = ric = sc = System = None
    Color = None


# Fast 2D min-width math. Rhino used to probe every sample with CurveCurve +
# GetLength/Contains on the NURBS (O(samples * inner_steps)); that froze the UI
# on a typical koru. This is the same polyline walk as the web tool.

MAX_SAMPLES = 360


def _vsub(a, b):
    return (a[0] - b[0], a[1] - b[1])


def _vadd(a, b):
    return (a[0] + b[0], a[1] + b[1])


def _vmul(a, s):
    return (a[0] * s, a[1] * s)


def _vdist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _vunit(a):
    length = math.hypot(a[0], a[1])
    if length < 1e-12:
        return (0.0, 0.0)
    return (a[0] / length, a[1] / length)


def _vcross(a, b):
    return a[0] * b[1] - a[1] * b[0]


def _vleft(tangent):
    return (-tangent[1], tangent[0])


def _clean_ring(points, eps=0.02):
    if not points:
        return []
    out = [points[0]]
    for p in points[1:]:
        if _vdist(p, out[-1]) > eps:
            out.append(p)
    if len(out) > 2 and _vdist(out[0], out[-1]) <= eps:
        out.pop()
    return out


def _signed_area_xy(points):
    area = 0.0
    n = len(points)
    for i in range(n):
        a = points[i]
        b = points[(i + 1) % n]
        area += a[0] * b[1] - b[0] * a[1]
    return area * 0.5


def _ensure_ccw(points):
    if _signed_area_xy(points) < 0:
        return list(reversed(points))
    return list(points)


def _point_in_ring(p, ring):
    inside = False
    n = len(ring)
    j = n - 1
    for i in range(n):
        a = ring[i]
        b = ring[j]
        if (a[1] > p[1]) != (b[1] > p[1]):
            x = ((b[0] - a[0]) * (p[1] - a[1]) / ((b[1] - a[1]) or 1e-18)) + a[0]
            if p[0] < x:
                inside = not inside
        j = i
    return inside


def _closest_on_seg(p, a, b):
    ab = _vsub(b, a)
    ap = _vsub(p, a)
    den = ab[0] * ab[0] + ab[1] * ab[1]
    if den < 1e-18:
        return a
    t = (ap[0] * ab[0] + ap[1] * ab[1]) / den
    if t < 0.0:
        t = 0.0
    elif t > 1.0:
        t = 1.0
    return (a[0] + ab[0] * t, a[1] + ab[1] * t)


def _ray_seg_t(origin, direction, a, b):
    seg = _vsub(b, a)
    det = _vcross(direction, seg)
    if abs(det) < 1e-12:
        return None
    ao = _vsub(a, origin)
    t = _vcross(ao, seg) / det
    u = _vcross(ao, direction) / det
    if t > 1e-4 and -1e-6 <= u <= 1.0 + 1e-6:
        return t
    return None


def _ring_length(ring):
    total = 0.0
    n = len(ring)
    for i in range(n):
        total += _vdist(ring[i], ring[(i + 1) % n])
    return total


def _cap_ring(ring, max_count):
    if len(ring) <= max_count:
        return ring
    total = _ring_length(ring)
    if total < 1e-9:
        return ring[:max_count]
    spacing = total / float(max_count)
    out = [ring[0]]
    acc = 0.0
    n = len(ring)
    for i in range(n):
        a = ring[i]
        b = ring[(i + 1) % n]
        seg = _vdist(a, b)
        if seg < 1e-12:
            continue
        acc += seg
        while acc >= spacing and len(out) < max_count:
            t = 1.0 - (acc - spacing) / seg
            if t < 0.0:
                t = 0.0
            if t > 1.0:
                t = 1.0
            pt = (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
            if _vdist(pt, out[-1]) > 1e-6:
                out.append(pt)
            acc -= spacing
    return out if len(out) >= 3 else ring[:max_count]


def _sample_boundary(ring, spacing):
    samples = []
    n = len(ring)
    for i in range(n):
        a = ring[i]
        b = ring[(i + 1) % n]
        length = _vdist(a, b)
        if length < 1e-9:
            continue
        tangent = _vunit(_vsub(b, a))
        inward = _vleft(tangent)
        steps = max(1, int(math.ceil(length / spacing)))
        for k in range(steps):
            t = k / float(steps)
            samples.append({
                "point": (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t),
                "inward": inward,
                "edge": i,
            })
    return samples


def _local_width(origin, inward, ring, skip_edge):
    n = len(ring)
    best = float("inf")
    for i in range(n):
        wrap = min(abs(i - skip_edge), n - abs(i - skip_edge))
        if wrap <= 1:
            continue
        hit = _ray_seg_t(origin, inward, ring[i], ring[(i + 1) % n])
        if hit is not None and hit < best:
            best = hit
        close = _closest_on_seg(origin, ring[i], ring[(i + 1) % n])
        gap = _vdist(origin, close)
        if gap >= best or gap < 1e-4:
            continue
        mid = ((origin[0] + close[0]) * 0.5, (origin[1] + close[1]) * 0.5)
        if _point_in_ring(mid, ring) and gap < best:
            best = gap
    return best


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


def _thin_runs(flags):
    if not any(flags):
        return 0
    runs = 0
    in_run = False
    for flag in flags:
        if flag and not in_run:
            runs += 1
            in_run = True
        elif not flag:
            in_run = False
    if flags[0] and flags[-1] and runs >= 2:
        runs -= 1
    return runs


def _exterior_arc(center, start, end, radius, ring):
    a0 = math.atan2(start[1] - center[1], start[0] - center[0])
    a1 = math.atan2(end[1] - center[1], end[0] - center[0])
    da = a1 - a0
    while da <= -math.pi:
        da += math.pi * 2.0
    while da > math.pi:
        da -= math.pi * 2.0
    mid_a = a0 + da / 2.0
    mid = (center[0] + math.cos(mid_a) * radius, center[1] + math.sin(mid_a) * radius)
    if _point_in_ring(mid, ring):
        da = da - math.pi * 2.0 if da > 0 else da + math.pi * 2.0
    steps = max(6, int(math.ceil((abs(da) * radius) / 0.12)))
    pts = []
    for i in range(1, steps):
        a = a0 + (da * i) / float(steps)
        pts.append((center[0] + math.cos(a) * radius, center[1] + math.sin(a) * radius))
    return pts


def _offset_point(sample, delta, ring):
    if delta <= 1e-6:
        return sample["point"]
    outward = _vmul(sample["inward"], -1.0)
    probe = _vadd(sample["point"], _vmul(outward, min(0.2, delta)))
    if _point_in_ring(probe, ring):
        outward = _vmul(outward, -1.0)
    return _vadd(sample["point"], _vmul(outward, delta))


def _prepare_ring(points):
    ring = _cap_ring(_ensure_ccw(_clean_ring(points)), MAX_SAMPLES)
    if len(ring) < 3:
        return (ring, [], 0.25)
    perimeter = _ring_length(ring)
    spacing = max(0.25, perimeter / float(MAX_SAMPLES))
    samples = _sample_boundary(ring, spacing)
    for sample in samples:
        sample["width"] = _local_width(sample["point"], sample["inward"], ring, sample["edge"])
    return (ring, samples, spacing)


def _nearest_edge(origin, ring):
    n = len(ring)
    best_i = 0
    best = 1e300
    for i in range(n):
        close = _closest_on_seg(origin, ring[i], ring[(i + 1) % n])
        d = _vdist(origin, close)
        if d < best:
            best = d
            best_i = i
    return best_i


def _apply_min_width(ring, samples, spacing, min_width, round_corners=True):
    if len(ring) < 3 or min_width <= 0:
        return (ring, 0)
    if len(samples) < 3:
        return (ring, 0)
    raw = []
    for sample in samples:
        width = sample["width"]
        if width == float("inf"):
            raw.append(0.0)
        else:
            raw.append(max(0.0, (min_width - width) * 0.5))
    if not any(d > 1e-4 for d in raw):
        return (ring, 0)
    sigma = max(1.2, (min_width * 0.55) / spacing)
    deltas = _smooth_closed_values(raw, sigma)
    pinches = _thin_runs([d > 0.04 for d in deltas])
    moved = []
    n = len(samples)
    for i in range(n):
        prev = samples[(i - 1 + n) % n]
        curr = samples[i]
        nxt = samples[(i + 1) % n]
        t1 = _vunit(_vsub(curr["point"], prev["point"]))
        t2 = _vunit(_vsub(nxt["point"], curr["point"]))
        turn = math.atan2(_vcross(t1, t2), t1[0] * t2[0] + t1[1] * t2[1])
        delta = deltas[i]
        p_off = _offset_point(curr, delta, ring)
        if round_corners and abs(turn) > 0.35 and delta > 0.05 and moved:
            from_pt = _offset_point({"point": curr["point"], "inward": prev["inward"]}, delta, ring)
            to_pt = _offset_point(curr, delta, ring)
            if _vdist(from_pt, to_pt) > 0.08:
                moved.append(from_pt)
                moved.extend(_exterior_arc(curr["point"], from_pt, to_pt, delta, ring))
        moved.append(p_off)
    cleaned = _clean_ring(moved, 0.02)
    return (cleaned if len(cleaned) >= 3 else ring, pinches)


def ensure_min_width_ring(points, min_width):
    """Widen only under-min-width stretches of a closed 2D ring.

    points: list of (x, y). Returns (moved_points, pinches).
    """
    ring, samples, spacing = _prepare_ring(points)
    return _apply_min_width(ring, samples, spacing, min_width)


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


def _to_xy(plane, pt):
    vec = pt - plane.Origin
    return (vec * plane.XAxis, vec * plane.YAxis)


def _from_xy(plane, xy):
    return plane.Origin + plane.XAxis * xy[0] + plane.YAxis * xy[1]


def _curve_ring_xy(curve, plane, count):
    """Sample a closed Rhino curve once. No per-sample GetLength."""
    pts = []
    try:
        ts = curve.DivideByCount(max(3, int(count)), True)
    except Exception:
        ts = None
    if ts:
        for t in ts:
            pts.append(_to_xy(plane, curve.PointAt(t)))
    else:
        domain = curve.Domain
        steps = max(3, int(count))
        for i in range(steps):
            t = domain.Min + (domain.Max - domain.Min) * (i / float(steps))
            pts.append(_to_xy(plane, curve.PointAt(t)))
    return _ensure_ccw(_clean_ring(pts, max(_tol(), 0.02)))


def _polyline_curve(plane, ring):
    if len(ring) < 2:
        return None
    pts = [_from_xy(plane, p) for p in ring]
    pts.append(pts[0])
    try:
        return rg.PolylineCurve(pts)
    except Exception:
        return None


def _samples_from_curve(curve, plane, ring, count):
    """Walk the original curve so the offset keeps its fair normals."""
    samples = []
    ts = None
    try:
        ts = curve.DivideByCount(max(8, int(count)), True)
    except Exception:
        ts = None
    if not ts:
        domain = curve.Domain
        steps = max(8, int(count))
        ts = [domain.Min + (domain.Max - domain.Min) * (i / float(steps)) for i in range(steps)]
    for t in ts:
        pt = curve.PointAt(t)
        xy = _to_xy(plane, pt)
        tan = curve.TangentAt(t)
        tan2 = _vunit((tan * plane.XAxis, tan * plane.YAxis))
        inward = _vleft(tan2)
        probe = _vadd(xy, _vmul(inward, 0.2))
        if not _point_in_ring(probe, ring):
            inward = _vmul(inward, -1.0)
        edge = _nearest_edge(xy, ring)
        width = _local_width(xy, inward, ring, edge)
        samples.append({
            "point": xy,
            "inward": inward,
            "edge": edge,
            "width": width,
            "t": t,
        })
    return samples


def _interpolated_closed(plane, ring):
    """Degree-3 periodic NURBS through the offset points — not a polyline."""
    if len(ring) < 4:
        return _polyline_curve(plane, ring)
    pts = [_from_xy(plane, p) for p in ring]
    styles = []
    try:
        styles.append(rg.CurveKnotStyle.ChordPeriodic)
        styles.append(rg.CurveKnotStyle.UniformPeriodic)
    except Exception:
        pass
    for style in styles:
        try:
            crv = rg.Curve.CreateInterpolatedCurve(pts, 3, style)
            if crv is not None and crv.IsValid:
                if not crv.IsClosed:
                    try:
                        crv.MakeClosed(_tol() * 4)
                    except Exception:
                        pass
                return crv
        except Exception:
            pass
    try:
        closed_pts = list(pts) + [pts[0]]
        crv = rg.Curve.CreateInterpolatedCurve(closed_pts, 3)
        if crv is not None and crv.IsValid:
            return crv
    except Exception:
        pass
    return _polyline_curve(plane, ring)


_CLOSED_PREP = {}


def _clear_closed_prep():
    _CLOSED_PREP.clear()


def _ensure_min_width(curve, min_width, corners):
    """Parallel-offset the original curve only where it is thinner than min_width."""
    plane = _curve_plane(curve)
    try:
        length = curve.GetLength()
    except Exception:
        length = 0.0
    if length < _tol() * 4:
        return [curve.DuplicateCurve()], 0, []

    key = id(curve)
    prep = _CLOSED_PREP.get(key)
    if prep is None:
        count = min(MAX_SAMPLES, max(48, int(math.ceil(length / 0.35))))
        ring = _curve_ring_xy(curve, plane, count)
        ring = _cap_ring(_ensure_ccw(_clean_ring(ring, max(_tol(), 0.02))), MAX_SAMPLES)
        out_count = min(320, max(96, int(math.ceil(length / 1.0))))
        samples = _samples_from_curve(curve, plane, ring, out_count)
        spacing = length / float(max(len(samples), 1))
        prep = (plane, ring, samples, spacing)
        _CLOSED_PREP[key] = prep
    plane, ring, samples, spacing = prep
    moved, pinches = _apply_min_width(ring, samples, spacing, min_width, round_corners=False)
    if pinches == 0:
        return [curve.DuplicateCurve()], 0, []
    outline_curve = _interpolated_closed(plane, moved)
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

    crv = curve

    if mode == "Slot":
        half = min_width * 0.5
        if half <= 0:
            return [], [], "MinWidth must be greater than 0."
        if crv.IsClosed:
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


if HAS_RHINO:
    _ConduitBase = Rhino.Display.DisplayConduit
else:
    class _ConduitBase(object):
        pass


class KerfConduit(_ConduitBase):
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
    redraw_was = True
    try:
        redraw_was = sc.doc.Views.RedrawEnabled
        sc.doc.Views.RedrawEnabled = False
    except Exception:
        pass
    try:
        for curve in curves:
            out_c, tool_c, err = compensate_curve(curve, min_width, kerf, caps, corners, mode)
            outlines.extend(out_c)
            toolpaths.extend(tool_c)
            if err:
                errors.append(err)
    finally:
        try:
            sc.doc.Views.RedrawEnabled = redraw_was
        except Exception:
            pass
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
    _clear_closed_prep()

    min_width = ric.OptionDouble(6.0, 0.01, 100000.0)
    kerf = ric.OptionDouble(1.5, 0.0, 100000.0)
    cap_index = [0]
    corner_index = [0]
    mode_index = [0]
    output_index = [0]

    conduit = KerfConduit()
    conduit.Enabled = True

    last_geom = [None]

    def refresh():
        geom_key = (
            min_width.CurrentValue,
            kerf.CurrentValue,
            CAP_NAMES[cap_index[0]],
            CORNER_NAMES[corner_index[0]],
            MODE_NAMES[mode_index[0]],
        )
        output = OUTPUT_NAMES[output_index[0]]
        if last_geom[0] == geom_key and (conduit.outlines or conduit.toolpaths):
            conduit.show_outline = output in ("Both", "Outline")
            conduit.show_toolpath = output in ("Both", "Toolpath")
            sc.doc.Views.Redraw()
            return []
        errors = _update_preview(
            conduit,
            curves,
            min_width.CurrentValue,
            kerf.CurrentValue,
            CAP_NAMES[cap_index[0]],
            CORNER_NAMES[corner_index[0]],
            MODE_NAMES[mode_index[0]],
            output,
        )
        last_geom[0] = geom_key
        return errors

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


def _self_test():
    def assert_true(cond, message):
        if not cond:
            raise AssertionError(message)

    square = [(0.0, 0.0), (80.0, 0.0), (80.0, 70.0), (0.0, 70.0)]
    moved, pinches = ensure_min_width_ring(square, 6.0)
    xs = [p[0] for p in moved]
    ys = [p[1] for p in moved]
    assert_true(pinches == 0, "wide square should not pinch")
    assert_true(max(xs) - min(xs) < 81.0, "wide square should stay ~80 wide")

    thin = [(0.0, 0.0), (80.0, 0.0), (80.0, 3.0), (0.0, 3.0)]
    moved, pinches = ensure_min_width_ring(thin, 6.0)
    ys = [p[1] for p in moved]
    xs = [p[0] for p in moved]
    height = max(ys) - min(ys)
    assert_true(pinches > 0, "thin slot should pinch")
    assert_true(5.4 < height < 7.2, "thin slot height should be ~6, got {0}".format(height))
    assert_true(max(xs) - min(xs) > 79.0, "thin slot should keep its length")

    tiny = [(0.0, 0.0), (5.0, 0.0), (5.0, 1.0), (0.0, 1.0)]
    moved, pinches = ensure_min_width_ring(tiny, 6.0)
    xs = [p[0] for p in moved]
    ys = [p[1] for p in moved]
    assert_true(pinches > 0, "tiny closed slot should widen locally")
    assert_true(max(ys) - min(ys) > 5.2, "tiny slot should reach min width")
    assert_true(max(xs) - min(xs) < 12.0, "tiny slot should not become a stadium ribbon")

    print("PlasmaKerf math tests passed")


if __name__ == "__main__":
    if HAS_RHINO:
        RunCommand()
    else:
        _self_test()
