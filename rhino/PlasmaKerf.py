"""
PlasmaKerf — offset curves for plasma kerf and minimum slot width.

Click a curve (or pre-select one), then adjust MinWidth / Kerf in the
command line. A live black preview is the finished cut; orange is the
torch centerline. Enter bakes the result onto layers.

Rhino 7 / 8
-----------
Closed openings are sampled once to a polyline (capped) for width. Already-wide
walls keep the original NURBS. Only under-width pinches move out along the
original normals and are G2-blended back, so a circular band stays circular.
Pointed corners of a wide opening stay as drawn. Sharp corners on a grown
pinch get a simple wall-to-wall radius, not a bulge. Rhino is not asked to
CurveCurve / GetLength / Contains on every sample, so a koru no longer locks
the UI.

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


def _vdot(a, b):
    return a[0] * b[0] + a[1] * b[1]


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


def _edge_prefix(ring):
    n = len(ring)
    pref = [0.0] * (n + 1)
    for i in range(n):
        pref[i + 1] = pref[i] + _vdist(ring[i], ring[(i + 1) % n])
    return pref


def _directed_arc(prefix, i, j):
    """Distance from start of edge i forward to start of edge j."""
    n = len(prefix) - 1
    if n <= 0 or i == j:
        return 0.0
    if i < j:
        return prefix[j] - prefix[i]
    return prefix[n] - prefix[i] + prefix[j]


def _along_from_origin(ring, prefix, origin, skip_edge, hit_edge):
    """Shorter along-boundary distance from a sample on skip_edge to hit_edge."""
    n = len(ring)
    if n <= 0:
        return 0.0
    edge_len = prefix[skip_edge + 1] - prefix[skip_edge]
    t = _vdist(ring[skip_edge], origin)
    if t > edge_len:
        t = edge_len
    rem = edge_len - t
    if skip_edge == hit_edge:
        return t if t < rem else rem
    fwd = rem + _directed_arc(prefix, (skip_edge + 1) % n, hit_edge)
    back = t + _directed_arc(prefix, (hit_edge + 1) % n, skip_edge)
    return fwd if fwd < back else back


def _local_width(origin, inward, ring, skip_edge, min_width=0.0, prefix=None):
    """Distance to the opposite wall of a slot.

    The other arm of a V / corner is nearby along the boundary and not
    parallel — that is a corner, not a slot. A closest-point on this same
    wall is along the tangent, not across the opening.
    """
    n = len(ring)
    if prefix is None:
        prefix = _edge_prefix(ring)
    tangent = (inward[1], -inward[0])
    skip_along = max(min_width * 2.0, 8.0)
    best = float("inf")
    for i in range(n):
        wrap = min(abs(i - skip_edge), n - abs(i - skip_edge))
        if wrap <= 1:
            continue
        hit_tan = _vunit(_vsub(ring[(i + 1) % n], ring[i]))
        parallel = abs(_vdot(tangent, hit_tan)) >= 0.82
        nearby = _along_from_origin(ring, prefix, origin, skip_edge, i) < skip_along
        if nearby and not parallel:
            continue
        hit = _ray_seg_t(origin, inward, ring[i], ring[(i + 1) % n])
        if hit is not None and hit < best:
            best = hit
        close = _closest_on_seg(origin, ring[i], ring[(i + 1) % n])
        gap = _vdist(origin, close)
        if gap >= best or gap < 1e-4:
            continue
        # Same-wall closest points lie along the tangent, not across the opening.
        if abs(_vdot(_vsub(close, origin), tangent)) > 0.85 * gap:
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


def _flag_runs(flags):
    """Inclusive (start, end) runs. A wrap-around run has start > end."""
    n = len(flags)
    if n == 0 or not any(flags):
        return []
    if all(flags):
        return [(0, n - 1)]
    runs = []
    i = 0
    while i < n:
        if not flags[i]:
            i += 1
            continue
        j = i
        while j < n and flags[j]:
            j += 1
        runs.append((i, j - 1))
        i = j
    if flags[0] and flags[-1] and len(runs) >= 2:
        start = runs[-1][0]
        end = runs[0][1]
        runs = [(start, end)] + runs[1:-1]
    return runs


def _run_indices(start, end, n):
    if start <= end:
        return list(range(start, end + 1))
    return list(range(start, n)) + list(range(0, end + 1))


def _ordered_runs(flags):
    """All True/False runs in cyclic order, starting at a run boundary."""
    n = len(flags)
    if n == 0:
        return []
    start = 0
    for i in range(n):
        if flags[i] != flags[(i - 1 + n) % n]:
            start = i
            break
    else:
        return [(0, n - 1, flags[0])]
    out = []
    i = start
    while True:
        val = flags[i]
        j = i
        while True:
            nxt = (j + 1) % n
            if nxt == start or flags[nxt] != val:
                out.append((i, j, val))
                i = nxt
                break
            j = nxt
        if i == start:
            break
    return out


def _value_at_t(samples, values, t):
    """Lerp a closed sample series at curve parameter t."""
    n = len(samples)
    if n == 0:
        return 0.0
    if n == 1:
        return values[0]
    ts = [s["t"] for s in samples]
    if t <= ts[0] or t >= ts[-1]:
        if abs(t - ts[0]) <= abs(t - ts[-1]):
            return values[0]
        return values[-1]
    lo = 0
    hi = n - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if ts[mid] <= t:
            lo = mid
        else:
            hi = mid
    span = ts[hi] - ts[lo]
    if span < 1e-12:
        return values[lo]
    u = (t - ts[lo]) / span
    return values[lo] * (1.0 - u) + values[hi] * u


def _suppress_short_runs(flags, min_len):
    """Drop isolated 1–2 sample 'thin' flags; those are corner noise, not a slot."""
    n = len(flags)
    out = list(flags)
    for start, end in _flag_runs(out):
        idxs = _run_indices(start, end, n)
        if len(idxs) < min_len:
            for i in idxs:
                out[i] = False
    return out


def _thin_runs(flags):
    return len(_flag_runs(flags))


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


def _seg_intersect(a, b, c, d):
    """Return intersection point of proper overlap, else None."""
    ab = _vsub(b, a)
    cd = _vsub(d, c)
    den = _vcross(ab, cd)
    if abs(den) < 1e-12:
        return None
    ac = _vsub(c, a)
    t = _vcross(ac, cd) / den
    u = _vcross(ac, ab) / den
    if t <= 1e-6 or t >= 1.0 - 1e-6 or u <= 1e-6 or u >= 1.0 - 1e-6:
        return None
    return (a[0] + ab[0] * t, a[1] + ab[1] * t)


def _polyline_self_intersects(pts):
    n = len(pts)
    if n < 4:
        return False
    for i in range(n):
        a = pts[i]
        b = pts[(i + 1) % n]
        for j in range(i + 2, n):
            if i == 0 and j == n - 1:
                continue
            if _seg_intersect(a, b, pts[j], pts[(j + 1) % n]) is not None:
                return True
    return False


def _remove_loops(pts):
    """Cut bowtie / tip loops out of a closed ring."""
    guard = 0
    while guard < 12:
        guard += 1
        n = len(pts)
        if n < 4:
            return pts
        found = None
        for i in range(n):
            a = pts[i]
            b = pts[(i + 1) % n]
            for j in range(i + 2, n):
                if i == 0 and j == n - 1:
                    continue
                hit = _seg_intersect(a, b, pts[j], pts[(j + 1) % n])
                if hit is None:
                    continue
                loop_a = (j - i) % n
                loop_b = n - loop_a
                found = (i, j, hit, loop_a <= loop_b)
                break
            if found:
                break
        if not found:
            return pts
        i, j, hit, keep_short = found
        if keep_short:
            # drop the short loop between i+1 and j
            nxt = pts[: i + 1] + [hit] + pts[j + 1 :]
        else:
            nxt = [hit] + pts[i + 1 : j + 1]
        nxt = _clean_ring(nxt, 0.02)
        if len(nxt) < 3 or len(nxt) >= n:
            return pts
        pts = nxt
    return pts


def _arc_points(center, start, end, radius, ring):
    pts = _exterior_arc(center, start, end, radius, ring)
    return pts


def _simple_corner(a, b, c, radius, ring):
    """Fillet walls at b with a compact radius. Does not wrap a bulge around the original vertex."""
    if radius < 0.04:
        return [b]
    t1 = _vunit(_vsub(b, a))
    t2 = _vunit(_vsub(c, b))
    turn = math.atan2(_vcross(t1, t2), t1[0] * t2[0] + t1[1] * t2[1])
    if abs(turn) < 0.18:
        return [b]
    d1 = _vdist(a, b)
    d2 = _vdist(b, c)
    if d1 < 0.05 or d2 < 0.05:
        return [b]

    # Hairpin: cap with a semicircle that fits between the two walls — no extra bulb.
    if abs(turn) > 2.05:
        p0 = (b[0] - t1[0] * min(radius, d1 * 0.35), b[1] - t1[1] * min(radius, d1 * 0.35))
        p1 = (b[0] + t2[0] * min(radius, d2 * 0.35), b[1] + t2[1] * min(radius, d2 * 0.35))
        mid = ((p0[0] + p1[0]) * 0.5, (p0[1] + p1[1]) * 0.5)
        fit_r = max(_vdist(p0, p1) * 0.5, 0.08)
        return [p0] + _arc_points(mid, p0, p1, fit_r, ring) + [p1]

    half = abs(turn) * 0.5
    tan_h = math.tan(half)
    if tan_h < 1e-6:
        return [b]
    # Limit trim so an acute corner cannot balloon past a simple radius.
    trim = min(radius * tan_h, radius * 1.1, d1 * 0.42, d2 * 0.42)
    if trim < 0.04:
        return [b]
    r_used = trim / tan_h
    p0 = (b[0] - t1[0] * trim, b[1] - t1[1] * trim)
    p1 = (b[0] + t2[0] * trim, b[1] + t2[1] * trim)
    left = (-t1[1], t1[0])
    outward = (-left[0], -left[1]) if turn > 0 else left
    center = (p0[0] + outward[0] * r_used, p0[1] + outward[1] * r_used)
    if _point_in_ring(center, ring):
        outward = (-outward[0], -outward[1])
        center = (p0[0] + outward[0] * r_used, p0[1] + outward[1] * r_used)
    return [p0] + _arc_points(center, p0, p1, r_used, ring) + [p1]


def _fillet_offset_ring(points, radii, ring, closed=True):
    n = len(points)
    if n < 3:
        return points
    if not closed:
        out = [points[0]]
        for i in range(1, n - 1):
            out.extend(_simple_corner(points[i - 1], points[i], points[i + 1], radii[i], ring))
        out.append(points[-1])
        return _clean_ring(out, 0.02)
    out = []
    for i in range(n):
        a = points[(i - 1 + n) % n]
        b = points[i]
        c = points[(i + 1) % n]
        out.extend(_simple_corner(a, b, c, radii[i], ring))
    return _clean_ring(out, 0.02)


def _turn_at(a, b, c):
    t1 = _vunit(_vsub(b, a))
    t2 = _vunit(_vsub(c, b))
    return math.atan2(_vcross(t1, t2), t1[0] * t2[0] + t1[1] * t2[1])


def _round_short_ends(points, ring, min_width):
    """Turn a blunt bar-end (two corners + short cap) into one semicircle."""
    n = len(points)
    if n < 8 or min_width <= 0:
        return points
    used = [False] * n
    out = []
    i = 0
    while i < n:
        if used[i]:
            i += 1
            continue
        a = points[(i - 1 + n) % n]
        b = points[i]
        c = points[(i + 1) % n]
        d = points[(i + 2) % n]
        edge = _vdist(b, c)
        prev_len = _vdist(a, b)
        next_len = _vdist(c, d)
        tb = _turn_at(a, b, c)
        tc = _turn_at(b, c, d)
        in_dir = _vunit(_vsub(b, a))
        out_dir = _vunit(_vsub(d, c))
        # A real blunt slot-end has opposite walls. A flattened V does not.
        opposite_walls = _vdot(in_dir, out_dir) < -0.72
        blunt = (
            min_width * 0.3 < edge < min_width * 1.4
            and prev_len > max(edge * 1.5, min_width * 0.8)
            and next_len > max(edge * 1.5, min_width * 0.8)
            and abs(tb) > 0.65
            and abs(tc) > 0.65
            and tb * tc > 0
            and opposite_walls
        )
        if blunt and not used[(i + 1) % n]:
            mid = ((b[0] + c[0]) * 0.5, (b[1] + c[1]) * 0.5)
            radius = max(edge * 0.5, 0.08)
            out.append(b)
            out.extend(_arc_points(mid, b, c, radius, ring))
            out.append(c)
            used[i] = True
            used[(i + 1) % n] = True
            i += 2
            continue
        out.append(b)
        used[i] = True
        i += 1
    return _clean_ring(out, 0.02) if len(out) >= 3 else points


def _assign_widths(samples, ring, min_width):
    prefix = _edge_prefix(ring)
    for sample in samples:
        sample["width"] = _local_width(
            sample["point"],
            sample["inward"],
            ring,
            sample["edge"],
            min_width,
            prefix,
        )


def _prepare_ring(points, min_width=0.0):
    ring = _cap_ring(_ensure_ccw(_clean_ring(points)), MAX_SAMPLES)
    if len(ring) < 3:
        return (ring, [], 0.25)
    perimeter = _ring_length(ring)
    spacing = max(0.25, perimeter / float(MAX_SAMPLES))
    samples = _sample_boundary(ring, spacing)
    _assign_widths(samples, ring, min_width)
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


def _compute_deltas(samples, spacing, min_width):
    """Per-sample outward grow. Isolated blips and leaked smoothing stay at 0."""
    if len(samples) < 3 or min_width <= 0:
        return [], 0
    raw = []
    for sample in samples:
        width = sample.get("width", float("inf"))
        if width == float("inf"):
            raw.append(0.0)
        else:
            raw.append(max(0.0, (min_width - width) * 0.5))
    flags = _suppress_short_runs([d > 0.04 for d in raw], 3)
    for i in range(len(raw)):
        if not flags[i]:
            raw[i] = 0.0
    if not any(d > 1e-4 for d in raw):
        return [0.0] * len(samples), 0
    sigma = max(1.2, (min_width * 0.55) / max(spacing, 1e-6))
    deltas = _smooth_closed_values(raw, sigma)
    # Smoothing may leak into already-wide walls. Keep it only near a real pinch.
    n = len(deltas)
    keep = [False] * n
    radius = max(2, int(math.ceil(sigma * 2.0)))
    for i, flag in enumerate(flags):
        if not flag:
            continue
        for j in range(-radius, radius + 1):
            keep[(i + j) % n] = True
    for i in range(n):
        if (not keep[i]) or deltas[i] <= 0.08:
            deltas[i] = 0.0
    pinches = _thin_runs([d > 0.08 for d in deltas])
    return deltas, pinches


def _apply_min_width(ring, samples, spacing, min_width, round_corners=True):
    if len(ring) < 3 or min_width <= 0:
        return (ring, 0)
    if len(samples) < 3:
        return (ring, 0)
    deltas, pinches = _compute_deltas(samples, spacing, min_width)
    if pinches == 0:
        return (ring, 0)
    moved = [_offset_point(samples[i], deltas[i], ring) for i in range(len(samples))]
    cleaned = _remove_loops(_clean_ring(moved, 0.02))
    n = len(cleaned)
    if n >= 3:
        radii = []
        for i in range(n):
            # Match each corner to nearby sample delta; never use a fat vertex-centered cap.
            nearest = min(
                range(len(samples)),
                key=lambda k: _vdist(cleaned[i], samples[k]["point"]),
            )
            radii.append(max(deltas[nearest], 0.0))
        cleaned = _fillet_offset_ring(cleaned, radii, ring)
        cleaned = _round_short_ends(cleaned, ring, min_width)
        cleaned = _remove_loops(cleaned)
    return (cleaned if len(cleaned) >= 3 else ring, pinches)


def ensure_min_width_ring(points, min_width):
    """Widen only under-min-width stretches of a closed 2D ring.

    points: list of (x, y). Returns (moved_points, pinches).
    """
    ring, samples, spacing = _prepare_ring(points, min_width)
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
        samples.append({
            "point": xy,
            "inward": inward,
            "edge": edge,
            "t": t,
        })
    return samples


def _curve_self_intersects(curve):
    try:
        events = rg.Intersect.Intersection.CurveSelf(curve, max(_tol(), 0.02))
        return events is not None and events.Count > 0
    except Exception:
        return False


def _interpolated_closed(plane, ring):
    """Degree-3 NURBS through the offset points. Reject a looped interpolation."""
    if len(ring) < 4:
        return _polyline_curve(plane, ring)
    pts = [_from_xy(plane, p) for p in ring]
    candidates = []
    styles = []
    try:
        styles.append(rg.CurveKnotStyle.Chord)
        styles.append(rg.CurveKnotStyle.ChordPeriodic)
    except Exception:
        pass
    for style in styles:
        try:
            crv = rg.Curve.CreateInterpolatedCurve(pts, 3, style)
            if crv is not None and crv.IsValid:
                candidates.append(crv)
        except Exception:
            pass
    try:
        crv = rg.Curve.CreateInterpolatedCurve(list(pts) + [pts[0]], 3)
        if crv is not None and crv.IsValid:
            candidates.append(crv)
    except Exception:
        pass
    for crv in candidates:
        if not crv.IsClosed:
            try:
                crv.MakeClosed(_tol() * 4)
            except Exception:
                pass
        if crv.IsValid and not _curve_self_intersects(crv):
            return crv
    return _polyline_curve(plane, ring)


def _shape_thin_run(pts, deltas, ring, min_width, closed):
    if len(pts) < 2:
        return pts
    cleaned = _clean_ring(pts, 0.02) if closed else list(pts)
    if len(cleaned) < 2:
        return pts
    radii = []
    for i in range(len(cleaned)):
        nearest = min(range(len(pts)), key=lambda k: _vdist(cleaned[i], pts[k]))
        radii.append(max(deltas[nearest], 0.0))
    cleaned = _fillet_offset_ring(cleaned, radii, ring, closed)
    if closed:
        cleaned = _round_short_ends(cleaned, ring, min_width)
        cleaned = _remove_loops(cleaned)
    return cleaned if len(cleaned) >= 2 else pts


def _interpolated_open(plane, pts):
    if not HAS_RHINO or len(pts) < 2:
        return None
    rh_pts = [_from_xy(plane, p) for p in pts]
    if len(pts) == 2:
        try:
            return rg.LineCurve(rh_pts[0], rh_pts[1])
        except Exception:
            return None
    try:
        crv = rg.Curve.CreateInterpolatedCurve(rh_pts, 3)
        if crv is not None and crv.IsValid:
            return crv
    except Exception:
        pass
    try:
        return rg.PolylineCurve(rh_pts)
    except Exception:
        return None


def _trim_span(curve, t0, t1):
    try:
        if abs(t1 - t0) < 1e-9:
            return []
        if t1 > t0:
            return [c for c in _as_list(curve.Trim(t0, t1)) if c]
        domain = curve.Domain
        a = _as_list(curve.Trim(t0, domain.Max))
        b = _as_list(curve.Trim(domain.Min, t1))
        return [c for c in a + b if c]
    except Exception:
        return []


def _safe_length(curve):
    try:
        return curve.GetLength()
    except Exception:
        return 0.0


def _param_at_length(curve, length):
    total = _safe_length(curve)
    if total < 1e-9:
        return None
    s = max(0.0, min(1.0, length / total))
    try:
        rc = curve.NormalizedLengthParameter(s)
        if isinstance(rc, tuple):
            if rc[0]:
                return rc[1]
        elif rc is not None:
            return rc
    except Exception:
        pass
    domain = curve.Domain
    return domain.Min + (domain.Max - domain.Min) * s


def _shorten_both(curve, start_len, end_len):
    total = _safe_length(curve)
    if total < 0.4:
        return curve
    start_len = max(0.0, start_len)
    end_len = max(0.0, end_len)
    if start_len + end_len >= total * 0.85:
        cap = total * 0.2
        start_len = min(start_len, cap)
        end_len = min(end_len, cap)
    t0 = _param_at_length(curve, start_len) if start_len > 1e-6 else curve.Domain.Min
    t1 = _param_at_length(curve, total - end_len) if end_len > 1e-6 else curve.Domain.Max
    if t0 is None or t1 is None or t1 <= t0:
        return curve
    got = [c for c in _as_list(curve.Trim(t0, t1)) if c]
    return got[0] if got else curve


def _make_blend(a, b):
    styles = []
    try:
        styles.append(rg.BlendContinuity.Curvature)
        styles.append(rg.BlendContinuity.Tangency)
        styles.append(rg.BlendContinuity.Position)
    except Exception:
        styles.extend([2, 1, 0])
    for style in styles:
        try:
            blend = rg.Curve.CreateBlendCurve(a, b, style)
            if blend is not None and blend.IsValid:
                return blend
        except Exception:
            continue
    return None


def _g2_join_closed(pieces, blend_len):
    """Join a cyclic list of curves with curvature blends at each seam."""
    n = len(pieces)
    if n == 0:
        return None
    if n == 1:
        crv = pieces[0]
        if crv and (not crv.IsClosed):
            try:
                crv.MakeClosed(max(_tol() * 8, 0.2))
            except Exception:
                pass
        return crv
    uses = []
    for i in range(n):
        use = min(
            blend_len,
            _safe_length(pieces[i]) * 0.35,
            _safe_length(pieces[(i + 1) % n]) * 0.35,
        )
        uses.append(use if use > 0.2 else 0.0)
    short = []
    for i in range(n):
        short.append(_shorten_both(pieces[i], uses[(i - 1 + n) % n], uses[i]))
    parts = []
    for i in range(n):
        parts.append(short[i])
        if uses[i] > 0:
            blend = _make_blend(short[i], short[(i + 1) % n])
            if blend is not None:
                parts.append(blend)
    try:
        joined = rg.Curve.JoinCurves(parts, max(_tol() * 8, 0.25), False)
        got = [c for c in _as_list(joined) if c]
    except Exception:
        got = []
    if not got:
        return None
    closed = [c for c in got if getattr(c, "IsClosed", False)]
    if len(closed) == 1 and not _curve_self_intersects(closed[0]):
        return closed[0]
    if len(got) == 1:
        crv = got[0]
        if not crv.IsClosed:
            try:
                crv.MakeClosed(max(_tol() * 8, 0.25))
            except Exception:
                pass
        if crv.IsValid and crv.IsClosed and not _curve_self_intersects(crv):
            return crv
    return None


def _sample_at(curve, plane, ring, t):
    pt = curve.PointAt(t)
    xy = _to_xy(plane, pt)
    tan = curve.TangentAt(t)
    tan2 = _vunit((tan * plane.XAxis, tan * plane.YAxis))
    inward = _vleft(tan2)
    probe = _vadd(xy, _vmul(inward, 0.2))
    if not _point_in_ring(probe, ring):
        inward = _vmul(inward, -1.0)
    return {
        "point": xy,
        "inward": inward,
        "edge": _nearest_edge(xy, ring),
        "t": t,
    }


def _eval_span(curve, plane, ring, t0, t1, count):
    count = max(4, int(count))
    ts = []
    if t1 > t0:
        ts = [t0 + (t1 - t0) * (i / float(count - 1)) for i in range(count)]
    else:
        domain = curve.Domain
        a_steps = max(2, int(round((count - 1) * 0.5)))
        b_steps = max(2, count - a_steps)
        ts = [t0 + (domain.Max - t0) * (i / float(a_steps)) for i in range(a_steps)]
        ts += [domain.Min + (t1 - domain.Min) * (i / float(b_steps - 1)) for i in range(b_steps)]
    return [_sample_at(curve, plane, ring, t) for t in ts]


def _offset_span_curve(curve, plane, ring, t0, t1, samples, deltas, min_width):
    """Smooth offset of one thin parameter span. Endpoints stay on the original."""
    span = _eval_span(curve, plane, ring, t0, t1, 48)
    if len(span) < 4:
        return None
    local = [_value_at_t(samples, deltas, s["t"]) for s in span]
    local[0] = 0.0
    local[-1] = 0.0
    pts = [_offset_point(span[i], local[i], ring) for i in range(len(span))]
    pts[0] = span[0]["point"]
    pts[-1] = span[-1]["point"]
    shaped = _shape_thin_run(pts, local, ring, min_width, False)
    if shaped:
        shaped[0] = pts[0]
        shaped[-1] = pts[-1]
    return _interpolated_open(plane, shaped)


def _rebuild_with_original(curve, plane, samples, deltas, ring, min_width):
    """Keep original NURBS on wide spans; G2-blend grown pinches back in."""
    if not HAS_RHINO or len(samples) < 4:
        return None
    thin = [d > 0.08 for d in deltas]
    if not any(thin):
        return curve.DuplicateCurve()
    grown = list(thin)
    n = len(samples)
    for i in range(n):
        if thin[i]:
            grown[(i - 1 + n) % n] = True
            grown[(i + 1) % n] = True
    if all(grown):
        return None
    pieces = []
    for start, end, is_thin in _ordered_runs(grown):
        idxs = _run_indices(start, end, n)
        if len(idxs) < 2:
            continue
        t0 = samples[idxs[0]]["t"]
        t1 = samples[idxs[-1]]["t"]
        if is_thin:
            piece = _offset_span_curve(curve, plane, ring, t0, t1, samples, deltas, min_width)
            if piece is None:
                return None
            pieces.append(piece)
        else:
            trimmed = _trim_span(curve, t0, t1)
            if not trimmed:
                return None
            pieces.extend(trimmed)
    if not pieces:
        return None
    return _g2_join_closed(pieces, max(min_width * 0.7, 1.5))


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
        width_count = min(320, max(96, int(math.ceil(length / 1.0))))
        samples = _samples_from_curve(curve, plane, ring, width_count)
        spacing = length / float(max(len(samples), 1))
        prep = (plane, ring, samples, spacing)
        _CLOSED_PREP[key] = prep
    plane, ring, samples, spacing = prep
    _assign_widths(samples, ring, min_width)
    deltas, pinches = _compute_deltas(samples, spacing, min_width)
    if pinches == 0:
        return [curve.DuplicateCurve()], 0, []
    outline_curve = _rebuild_with_original(curve, plane, samples, deltas, ring, min_width)
    if outline_curve is None:
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
    assert_true(
        max(xs) - min(xs) > 81.5,
        "thin slot ends should be a single radius, not a flat bar ({0})".format(max(xs) - min(xs)),
    )

    tiny = [(0.0, 0.0), (5.0, 0.0), (5.0, 1.0), (0.0, 1.0)]
    moved, pinches = ensure_min_width_ring(tiny, 6.0)
    xs = [p[0] for p in moved]
    ys = [p[1] for p in moved]
    assert_true(pinches > 0, "tiny closed slot should widen locally")
    assert_true(max(ys) - min(ys) > 5.2, "tiny slot should reach min width")
    assert_true(max(xs) - min(xs) < 12.0, "tiny slot should not become a stadium ribbon")

    hairpin = []
    for y in range(0, 41):
        hairpin.append((0.0, float(y)))
    for i in range(1, 17):
        a = math.pi * 0.5 - i * math.pi / 16.0
        hairpin.append((0.5 + 0.5 * math.cos(a), 40.0 + 0.5 * math.sin(a)))
    for y in range(40, -1, -1):
        hairpin.append((1.0, float(y)))
    ring, samples, spacing = _prepare_ring(hairpin, 6.0)
    moved, pinches = _apply_min_width(ring, samples, spacing, 6.0, round_corners=False)
    ys = [p[1] for p in moved]
    assert_true(pinches > 0, "hairpin tip should be under min width")
    assert_true(not _polyline_self_intersects(moved), "hairpin tip must not loop")
    assert_true(max(ys) > 42.0, "hairpin tip should grow to a round cap, maxY={0}".format(max(ys)))

    # 90° corner of a 3 mm band grown to 6 mm: simple radius, not a bulb.
    elbow = [
        (0.0, 0.0), (40.0, 0.0), (40.0, 40.0), (37.0, 40.0),
        (37.0, 3.0), (0.0, 3.0),
    ]
    moved, pinches = ensure_min_width_ring(elbow, 6.0)
    xs = [p[0] for p in moved]
    assert_true(pinches > 0, "thin elbow should widen")
    assert_true(max(xs) < 44.5, "elbow corner should be a simple radius, not a bulge (maxX={0})".format(max(xs)))

    # Tessellated V of a wide opening: stay pointed, do not grow into a square bar.
    chevron = []
    for i in range(50):
        t = i / 49.0
        chevron.append((50.0 * t, 80.0 - 80.0 * t))
    for i in range(1, 50):
        t = i / 49.0
        chevron.append((50.0 + 50.0 * t, 80.0 * t))
    chevron.append((0.0, 80.0))
    moved, pinches = ensure_min_width_ring(chevron, 6.0)
    ys = [p[1] for p in moved]
    tip = [p for p in moved if p[1] < 4.0]
    tip_span = (max(p[0] for p in tip) - min(p[0] for p in tip)) if tip else 0.0
    assert_true(pinches == 0, "wide V opening should not count as a pinch")
    assert_true(min(ys) < 1.2, "V tip should stay pointed, minY={0}".format(min(ys)))
    assert_true(tip_span < 8.0, "V tip should not become a square bar (span={0})".format(tip_span))

    ellipse = []
    for i in range(72):
        a = (math.pi * 2.0 * i) / 72.0
        ellipse.append((50.0 * math.cos(a), 30.0 * math.sin(a)))
    moved, pinches = ensure_min_width_ring(ellipse, 6.0)
    assert_true(pinches == 0, "wide ellipse should stay as drawn")
    assert_true(max(p[0] for p in moved) < 51.0, "wide ellipse should not offset")

    # Hourglass neck is not parallel walls, but it is a real pinch.
    hourglass = [
        (20.0, 8.0), (58.0, 8.0), (72.0, 22.0), (86.0, 8.0), (124.0, 8.0),
        (124.0, 32.0), (86.0, 32.0), (72.0, 18.0), (58.0, 32.0), (20.0, 32.0),
    ]
    moved, pinches = ensure_min_width_ring(hourglass, 6.0)
    neck = [p for p in moved if 64.0 < p[0] < 80.0]
    neck_h = (max(p[1] for p in neck) - min(p[1] for p in neck)) if neck else 0.0
    assert_true(pinches > 0, "hourglass neck should be under min width")
    assert_true(neck_h > 5.2, "hourglass neck should grow toward 6 mm, got {0}".format(neck_h))

    # Wide bulbs next to a 3 mm neck: the bulbs must not inherit the pinch grow.
    barbell = [
        (0.0, 0.0), (30.0, 0.0), (30.0, 8.5), (50.0, 8.5), (50.0, 0.0), (80.0, 0.0),
        (80.0, 20.0), (50.0, 20.0), (50.0, 11.5), (30.0, 11.5), (30.0, 20.0), (0.0, 20.0),
    ]
    moved, pinches = ensure_min_width_ring(barbell, 6.0)
    left = [p for p in moved if p[0] < 18.0]
    left_h = (max(p[1] for p in left) - min(p[1] for p in left)) if left else 0.0
    assert_true(pinches > 0, "barbell neck should pinch")
    assert_true(left_h < 21.2, "wide barbell bulb should stay on the original, h={0}".format(left_h))

    # One closed ring: no seam gap between a grown pinch and a wide wall.
    thin = [(0.0, 0.0), (80.0, 0.0), (80.0, 3.0), (0.0, 3.0)]
    moved, pinches = ensure_min_width_ring(thin, 6.0)
    steps = [_vdist(moved[i], moved[(i + 1) % len(moved)]) for i in range(len(moved))]
    steps.sort()
    typical = steps[len(steps) // 2]
    assert_true(pinches > 0, "continuity slot should still pinch")
    assert_true(max(steps) < typical * 12 + 4.0, "outline must stay one curve, max step={0}".format(max(steps)))

    print("PlasmaKerf math tests passed")


if __name__ == "__main__":
    if HAS_RHINO:
        RunCommand()
    else:
        _self_test()
