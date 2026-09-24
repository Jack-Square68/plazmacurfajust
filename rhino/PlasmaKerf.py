"""
PlasmaKerf — offset curves for plasma kerf and minimum slot width.

Click a curve (or pre-select one), then adjust MinWidth / Kerf in the
command line. A live black preview is the finished cut; orange is the
torch centerline. Enter bakes the result onto layers.

Rhino 7 / 8
-----------
Closed openings are sampled once to a polyline (capped) for width only.
The baked outline keeps the original few-CV NURBS when it can (CVs slide
outward). Otherwise it is one closed periodic cubic B-spline through the
offset walls — never CreateInterpolatedCurve, which looped and spiked.
Slot ends get a MinWidth/2 semicircle around the original tip. Pointed
corners of a wide opening stay as drawn. Rhino is not asked to CurveCurve
/ GetLength / Contains on every sample, so a koru no longer locks the UI.

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


def _walk_along(points, start, step, distance):
    n = len(points)
    acc = 0.0
    i = start
    guard = 0
    while acc < distance - 1e-9 and guard < n:
        guard += 1
        j = (i + step + n) % n
        seg = _vdist(points[i], points[j])
        if seg < 1e-12:
            i = j
            continue
        if acc + seg >= distance:
            t = (distance - acc) / seg
            pt = (points[i][0] + (points[j][0] - points[i][0]) * t,
                  points[i][1] + (points[j][1] - points[i][1]) * t)
            return pt, i
        acc += seg
        i = j
    return points[i], i


def _mark_span(used, start, end, through, n):
    for step in (1, -1):
        idxs = []
        i = start
        for _ in range(n):
            idxs.append(i)
            if i == end:
                break
            i = (i + step + n) % n
        if through in idxs:
            for i in idxs:
                used[i] = True
            return


def _semicircle_cap(p0, p1, radius, ring):
    mid = ((p0[0] + p1[0]) * 0.5, (p0[1] + p1[1]) * 0.5)
    return [p0] + _arc_points(mid, p0, p1, radius, ring) + [p1]


def _ring_path_len(ring, start, end):
    n = len(ring)
    if n == 0 or start == end:
        return 0.0
    total = 0.0
    i = start
    for _ in range(n):
        nxt = (i + 1) % n
        total += _vdist(ring[i], ring[nxt])
        if nxt == end:
            return total
        i = nxt
    return total


def _grow_near(point, samples, grow):
    if not samples or not grow:
        return 0.0
    best = 0
    best_d = 1e300
    for i, sample in enumerate(samples):
        d = _vdist(point, sample["point"])
        if d < best_d:
            best_d = d
            best = i
    return grow[best] if best < len(grow) else 0.0


def _near_pinch(point, samples, grow, min_width):
    if not samples or not grow:
        return False
    thresh = max(min_width * 1.8, 6.0)
    for i, sample in enumerate(samples):
        if i < len(grow) and grow[i] > 0.04 and _vdist(point, sample["point"]) < thresh:
            return True
    return False


def _grow_at_point(point, samples, grow, default):
    if not samples or not grow or point is None:
        return default
    best_i, _ = _nearest_index([s["point"] for s in samples], point)
    if best_i < len(grow) and grow[best_i] > 0.04:
        return grow[best_i]
    return default


def _sample_width_near(point, samples, default=1e9):
    if not samples or point is None:
        return default
    best_i, _ = _nearest_index([s["point"] for s in samples], point)
    width = samples[best_i].get("width", default)
    if width is None:
        return default
    return width


def _is_tapered_end(center, samples, min_width):
    """True for a koru / scroll tip: the ribbon widens away from the end."""
    w0 = _sample_width_near(center, samples)
    if w0 >= 1e8:
        return False
    backs = []
    for sample in samples:
        dist = _vdist(sample["point"], center)
        if dist < 10.0 or dist > 26.0:
            continue
        width = sample.get("width")
        if width is None or width >= 1e8:
            continue
        backs.append(width)
    if not backs:
        return False
    backs.sort()
    w_back = backs[len(backs) // 2]
    return w_back > max(w0 * 1.7, w0 + 1.2)


def _walk_until_radius(points, start, step, center, radius):
    """Walk a ring until we are `radius` from `center`."""
    n = len(points)
    if n < 2:
        return points[start], start
    prev = points[start]
    d_prev = _vdist(prev, center)
    i = start
    for _ in range(n):
        j = (i + step + n) % n
        nxt = points[j]
        d = _vdist(nxt, center)
        if d >= radius and d_prev <= radius:
            den = d - d_prev
            t = 0.0 if abs(den) < 1e-12 else (radius - d_prev) / den
            if t < 0.0:
                t = 0.0
            elif t > 1.0:
                t = 1.0
            return (_vadd(prev, _vmul(_vsub(nxt, prev), t)), i)
        if d >= radius:
            return nxt, i
        prev = nxt
        d_prev = d
        i = j
    return points[i], i


def _tapered_tip_chain(center, moved, samples, grow, min_width, ring):
    """Thicker copy of the original scroll tip — same spine, larger radius."""
    if center is None or len(ring) < 6:
        return []
    d = max(_grow_at_point(center, samples, grow, 0.0), 0.0)
    w0 = _sample_width_near(center, samples, 0.6)
    if w0 >= 1e8:
        w0 = 0.6
    new_r = max(w0 * 0.5 + d, 0.8)
    n = len(ring)
    i0, _ = _nearest_index(ring, center)
    best_i, best_t = i0, -1.0
    for k in range(-16, 17):
        j = (i0 + k + n) % n
        turn = abs(_turn_at(ring[(j - 1 + n) % n], ring[j], ring[(j + 1) % n]))
        if turn > best_t:
            best_t = turn
            best_i = j
    apex = ring[best_i]
    back, _ = _walk_along(ring, best_i, -1, max(new_r * 2.2, 6.0))
    spine = _vunit(_vsub(apex, back))
    if spine[0] == 0.0 and spine[1] == 0.0:
        back, _ = _walk_along(ring, best_i, 1, max(new_r * 2.2, 6.0))
        spine = _vunit(_vsub(apex, back))
    if spine[0] == 0.0 and spine[1] == 0.0:
        return []
    left = _vleft(spine)
    p0 = _vadd(apex, _vmul(left, new_r))
    p1 = _vadd(apex, _vmul(left, -new_r))
    return _semicircle_cap(p0, p1, new_r, ring)


def _cluster_original_turns(ring, min_width):
    """Group nearby same-sign turns so a tessellated CAD corner is one feature."""
    n = len(ring)
    if n < 3:
        return []
    turns = [_turn_at(ring[(i - 1 + n) % n], ring[i], ring[(i + 1) % n]) for i in range(n)]
    window = max(min_width * 0.45, 1.2)
    clusters = []
    used = [False] * n
    for i in range(n):
        if used[i] or abs(turns[i]) < 0.12:
            continue
        sign = 1.0 if turns[i] > 0.0 else -1.0
        acc = turns[i]
        idxs = [i]
        used[i] = True
        j = i
        dist = 0.0
        while True:
            nxt = (j + 1) % n
            if used[nxt]:
                break
            seg = _vdist(ring[j], ring[nxt])
            if dist + seg > window:
                break
            if turns[nxt] * sign < -0.08:
                break
            if abs(turns[nxt]) >= 0.08:
                acc += turns[nxt]
                idxs.append(nxt)
                used[nxt] = True
            elif dist > 0.25:
                break
            dist += seg
            j = nxt
        if abs(acc) < 0.35:
            for k in idxs:
                used[k] = False
            continue
        apex = max(idxs, key=lambda k: abs(turns[k]))
        clusters.append({
            "idxs": idxs,
            "apex": apex,
            "turn": acc,
            "a": ring[(idxs[0] - 1 + n) % n],
            "b": ring[apex],
            "c": ring[(idxs[-1] + 1) % n],
        })
    return clusters


def _line_intersect_unbounded(a, b, c, d):
    ab = _vsub(b, a)
    cd = _vsub(d, c)
    den = _vcross(ab, cd)
    if abs(den) < 1e-12:
        return None
    ac = _vsub(c, a)
    t = _vcross(ac, cd) / den
    return (a[0] + ab[0] * t, a[1] + ab[1] * t)


def _wall_outward(a, b, ring):
    tangent = _vunit(_vsub(b, a))
    inward = _vleft(tangent)
    mid = ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5)
    probe = _vadd(mid, _vmul(inward, 0.35))
    if not _point_in_ring(probe, ring):
        inward = _vmul(inward, -1.0)
    return _vmul(inward, -1.0)


def _vertex_round_join(a, b, c, radius, ring):
    """One circular join of `radius` around original vertex b — not an S-wave."""
    if radius < 0.04:
        return [b]
    out1 = _wall_outward(a, b, ring)
    out2 = _wall_outward(b, c, ring)
    p0 = _vadd(b, _vmul(out1, radius))
    p1 = _vadd(b, _vmul(out2, radius))
    if _vdist(p0, p1) < 0.04:
        return [p0]
    return [p0] + _arc_points(b, p0, p1, radius, ring) + [p1]


def _fillet_grown_corner(a, b, c, grow1, grow2, radius, ring):
    """Simple half-min-width radius on the two grown walls at original corner b."""
    if radius < 0.04:
        return [b]
    out1 = _wall_outward(a, b, ring)
    out2 = _wall_outward(b, c, ring)
    g1a = _vadd(a, _vmul(out1, grow1))
    g1b = _vadd(b, _vmul(out1, grow1))
    g2b = _vadd(b, _vmul(out2, grow2))
    g2c = _vadd(c, _vmul(out2, grow2))
    corner = _line_intersect_unbounded(g1a, g1b, g2b, g2c)
    if corner is None:
        return _vertex_round_join(a, b, c, radius, ring)
    t1 = _vunit(_vsub(corner, g1a))
    if t1[0] == 0.0 and t1[1] == 0.0:
        t1 = _vunit(_vsub(g1b, g1a))
    t2 = _vunit(_vsub(g2c, corner))
    if t2[0] == 0.0 and t2[1] == 0.0:
        t2 = _vunit(_vsub(g2c, g2b))
    if (t1[0] == 0.0 and t1[1] == 0.0) or (t2[0] == 0.0 and t2[1] == 0.0):
        return _vertex_round_join(a, b, c, radius, ring)
    turn = math.atan2(_vcross(t1, t2), _vdot(t1, t2))
    if abs(turn) < 0.12:
        return [corner]
    tan_h = math.tan(abs(turn) * 0.5)
    if tan_h < 1e-8:
        return [corner]
    trim = radius * tan_h
    d1 = _vdist(g1a, corner)
    d2 = _vdist(corner, g2c)
    if trim > d1 * 0.98:
        trim = d1 * 0.98
    if trim > d2 * 0.98:
        trim = d2 * 0.98
    if trim < 0.04:
        return [corner]
    r_used = trim / tan_h
    p0 = (corner[0] - t1[0] * trim, corner[1] - t1[1] * trim)
    p1 = (corner[0] + t2[0] * trim, corner[1] + t2[1] * trim)
    left = (-t1[1], t1[0])
    inward = left if turn > 0.0 else (-left[0], -left[1])
    center = (p0[0] + inward[0] * r_used, p0[1] + inward[1] * r_used)
    return [p0] + _arc_points(center, p0, p1, r_used, ring) + [p1]


def _fair_grown_corner(a, b, c, grow1, grow2, radius, ring):
    """Offset the original walls, then G2-blend the miter — one fair tip."""
    if radius < 0.04:
        return [b]
    out1 = _wall_outward(a, b, ring)
    out2 = _wall_outward(b, c, ring)
    g1a = _vadd(a, _vmul(out1, grow1))
    g1b = _vadd(b, _vmul(out1, grow1))
    g2b = _vadd(b, _vmul(out2, grow2))
    g2c = _vadd(c, _vmul(out2, grow2))
    corner = _line_intersect_unbounded(g1a, g1b, g2b, g2c)
    if corner is None:
        return _fair_tip_blend(a, b, c, radius, ring)
    chain = _fair_tip_blend(g1a, corner, g2c, radius, ring)
    if len(chain) >= 4:
        return chain
    return _fillet_grown_corner(a, b, c, grow1, grow2, radius, ring)


def _end_cap_chain(center, p0, p1, radius, ring):
    """Semicircle of `radius` around the original square / pointed end."""
    chord = _vunit(_vsub(p1, p0))
    if chord[0] == 0.0 and chord[1] == 0.0:
        return [center]
    a = _vadd(center, _vmul(chord, -radius))
    b = _vadd(center, _vmul(chord, radius))
    return _semicircle_cap(a, b, radius, ring)


def _nearest_index(points, target):
    best_i = 0
    best = 1e300
    for i, p in enumerate(points):
        d = _vdist(p, target)
        if d < best:
            best = d
            best_i = i
    return best_i, best


def _replace_span(moved, start, end, chain):
    n = len(moved)
    if n < 3 or start == end:
        return moved
    before = moved[(start - 1 + n) % n]
    after = moved[(end + 1) % n]
    d_fwd = _vdist(before, chain[0]) + _vdist(chain[-1], after)
    d_rev = _vdist(before, chain[-1]) + _vdist(chain[0], after)
    if d_rev < d_fwd:
        chain = list(reversed(chain))
    if start <= end:
        return moved[:start] + chain + moved[end + 1:]
    return chain + moved[end + 1:start]


def _splice_chain(moved, chain):
    """Replace the short span between the chain's landing points with the chain."""
    if len(moved) < 4 or len(chain) < 2:
        return moved
    i0, d0 = _nearest_index(moved, chain[0])
    i1, d1 = _nearest_index(moved, chain[-1])
    if i0 == i1:
        return moved
    n = len(moved)
    span_fwd = (i1 - i0) % n
    span_rev = (i0 - i1) % n
    if span_fwd <= span_rev:
        start, end, span = i0, i1, span_fwd
    else:
        start, end, span = i1, i0, span_rev
        chain = list(reversed(chain))
    if span < 1 or span > max(6, int(n * 0.35)):
        return moved
    return _replace_span(moved, start, end, chain)


def _splice_near_vertex(moved, vertex, chain, radius):
    """Replace the offset loop around an original vertex when endpoint splice misses."""
    n = len(moved)
    if n < 4 or len(chain) < 2:
        return moved
    thresh = max(radius * 1.8, 1.0)
    flags = [_vdist(p, vertex) < thresh for p in moved]
    if not any(flags):
        return moved
    runs = _flag_runs(flags)
    if not runs:
        return moved

    def run_len(se):
        start, end = se
        if start <= end:
            return end - start + 1
        return n - start + end + 1

    start, end = max(runs, key=run_len)
    if run_len((start, end)) < 1 or run_len((start, end)) > max(8, int(n * 0.35)):
        return moved
    return _replace_span(moved, start, end, chain)


def _splice_feature(moved, chain, vertex, radius):
    nxt = _splice_chain(moved, chain)
    if nxt is not moved and len(nxt) >= 3:
        return nxt
    nxt = _splice_near_vertex(moved, vertex, chain, radius)
    return nxt if len(nxt) >= 3 else moved


def _splice_tip(moved, chain, vertex, radius):
    """Replace everything near the tip. Endpoint splice can land on one wall."""
    nxt = _splice_near_vertex(moved, vertex, chain, max(radius * 1.4, 3.2))
    if nxt is not moved and len(nxt) >= 3:
        return nxt
    return _splice_feature(moved, chain, vertex, radius)


def _pair_square_end(ring, ia, ib, min_width):
    """Two convex corners joined by a short edge → one blunt slot end."""
    n = len(ring)
    fwd = _ring_path_len(ring, ia, ib)
    rev = _ring_path_len(ring, ib, ia)
    if fwd <= rev:
        start, end, span = ia, ib, fwd
    else:
        start, end, span = ib, ia, rev
    if not (min_width * 0.12 < span < min_width * 1.65):
        return None
    prev_s = ring[(start - 1 + n) % n]
    next_e = ring[(end + 1) % n]
    if _vdist(prev_s, ring[start]) < span * 0.9:
        return None
    if _vdist(ring[end], next_e) < span * 0.9:
        return None
    in_dir = _vunit(_vsub(ring[start], prev_s))
    out_dir = _vunit(_vsub(next_e, ring[end]))
    if _vdot(in_dir, out_dir) > -0.28:
        return None
    center = (
        (ring[start][0] + ring[end][0]) * 0.5,
        (ring[start][1] + ring[end][1]) * 0.5,
    )
    return {
        "kind": "end",
        "center": center,
        "p0": ring[start],
        "p1": ring[end],
    }


def _original_features(feature_ring, samples, grow, min_width):
    """Sharp original corners / square ends that sit in a pinch."""
    clusters = _cluster_original_turns(feature_ring, min_width)
    if not clusters:
        return []
    used = [False] * len(clusters)
    features = []
    for a in range(len(clusters)):
        if used[a] or clusters[a]["turn"] <= 0.35:
            continue
        for b in range(a + 1, len(clusters)):
            if used[b] or clusters[b]["turn"] <= 0.35:
                continue
            paired = _pair_square_end(
                feature_ring,
                clusters[a]["apex"],
                clusters[b]["apex"],
                min_width,
            )
            if paired is None:
                continue
            if not _near_pinch(paired["center"], samples, grow, min_width):
                continue
            features.append(paired)
            used[a] = True
            used[b] = True
            break
    for i, cluster in enumerate(clusters):
        if used[i]:
            continue
        if cluster["turn"] <= 0.35:
            continue
        if not _near_pinch(cluster["b"], samples, grow, min_width):
            continue
        t1 = _vunit(_vsub(cluster["b"], cluster["a"]))
        t2 = _vunit(_vsub(cluster["c"], cluster["b"]))
        opposite = _vdot(t1, t2) < -0.90
        # A 60° / concave-sided triangle corner is a fillet. Slot ends only
        # when the two walls run back on themselves (hairpin).
        kind = "end" if opposite and cluster["turn"] > 2.4 else "fillet"
        features.append({
            "kind": kind,
            "center": cluster["b"],
            "p0": cluster["a"],
            "p1": cluster["c"],
            "a": cluster["a"],
            "b": cluster["b"],
            "c": cluster["c"],
        })
    return features


def _bezier_point(ctrl, t):
    """de Casteljau evaluation of a 2D Bézier."""
    pts = list(ctrl)
    u = 1.0 - t
    n = len(pts) - 1
    for _ in range(n):
        nxt = []
        for i in range(len(pts) - 1):
            nxt.append((pts[i][0] * u + pts[i + 1][0] * t, pts[i][1] * u + pts[i + 1][1] * t))
        pts = nxt
    return pts[0]


def _bezier_derivative(ctrl, t):
    n = len(ctrl) - 1
    if n < 1:
        return (0.0, 0.0)
    diffs = []
    for i in range(n):
        diffs.append(((ctrl[i + 1][0] - ctrl[i][0]) * n, (ctrl[i + 1][1] - ctrl[i][1]) * n))
    return _bezier_point(diffs, t)


def _sample_bezier(ctrl, spacing=0.12):
    if len(ctrl) < 2:
        return list(ctrl)
    prev = ctrl[0]
    acc = 0.0
    probe = 24
    for i in range(1, probe + 1):
        p = _bezier_point(ctrl, i / float(probe))
        acc += _vdist(prev, p)
        prev = p
    steps = max(10, int(math.ceil(acc / max(spacing, 0.04))))
    return [_bezier_point(ctrl, i / float(steps)) for i in range(steps + 1)]


def _g2_quintic_controls(p0, t0, k0, p1, t1, k1, alpha, beta):
    """Hermite G2 quintic: matches position, tangent and curvature at both ends."""
    t0 = _vunit(t0)
    t1 = _vunit(t1)
    if (t0[0] == 0.0 and t0[1] == 0.0) or (t1[0] == 0.0 and t1[1] == 0.0):
        return None
    n0 = (-t0[1], t0[0])
    n1 = (-t1[1], t1[0])
    a = max(alpha, 0.08)
    b = max(beta, 0.08)
    c1 = (p0[0] + (a / 5.0) * t0[0], p0[1] + (a / 5.0) * t0[1])
    c2 = (
        p0[0] + (2.0 * a / 5.0) * t0[0] + (a * a * k0 / 20.0) * n0[0],
        p0[1] + (2.0 * a / 5.0) * t0[1] + (a * a * k0 / 20.0) * n0[1],
    )
    c4 = (p1[0] - (b / 5.0) * t1[0], p1[1] - (b / 5.0) * t1[1])
    c3 = (
        p1[0] - (2.0 * b / 5.0) * t1[0] + (b * b * k1 / 20.0) * n1[0],
        p1[1] - (2.0 * b / 5.0) * t1[1] + (b * b * k1 / 20.0) * n1[1],
    )
    return [p0, c1, c2, c3, c4, p1]


def _circular_fillet_mid(a, b, c, radius):
    """Midpoint of the G1 circular fillet — target extent for the fair blend."""
    t1 = _vunit(_vsub(b, a))
    t2 = _vunit(_vsub(c, b))
    if (t1[0] == 0.0 and t1[1] == 0.0) or (t2[0] == 0.0 and t2[1] == 0.0):
        return None
    turn = math.atan2(_vcross(t1, t2), _vdot(t1, t2))
    if abs(turn) < 0.08:
        return None
    tan_h = math.tan(abs(turn) * 0.5)
    if tan_h < 1e-8:
        return None
    trim = radius * tan_h
    p0 = (b[0] - t1[0] * trim, b[1] - t1[1] * trim)
    p1 = (b[0] + t2[0] * trim, b[1] + t2[1] * trim)
    left = (-t1[1], t1[0])
    inward = left if turn > 0.0 else (-left[0], -left[1])
    center = (p0[0] + inward[0] * radius, p0[1] + inward[1] * radius)
    mid_a = math.atan2(p0[1] - center[1], p0[0] - center[0])
    mid_b = math.atan2(p1[1] - center[1], p1[0] - center[0])
    da = mid_b - mid_a
    while da <= -math.pi:
        da += math.pi * 2.0
    while da > math.pi:
        da -= math.pi * 2.0
    ang = mid_a + da * 0.5
    return (center[0] + math.cos(ang) * radius, center[1] + math.sin(ang) * radius)


def _fair_g2_controls(p0, t0, k0, p1, t1, k1, radius, corner):
    """G2 quintic that eases out of both walls and reaches the min-width tip."""
    t0 = _vunit(t0)
    t1 = _vunit(t1)
    if (t0[0] == 0.0 and t0[1] == 0.0) or (t1[0] == 0.0 and t1[1] == 0.0):
        return None
    target = None
    if corner is not None:
        target = _circular_fillet_mid(
            (p0[0] - t0[0], p0[1] - t0[1]),
            corner,
            (p1[0] + t1[0], p1[1] + t1[1]),
            radius,
        )
    if target is None:
        target = ((p0[0] + p1[0]) * 0.5, (p0[1] + p1[1]) * 0.5)
    span = max(_vdist(p0, p1), radius * 2.0, 0.5)
    lo = span * 0.35
    hi = span * 3.6
    best = None
    best_d = 1e300
    for _ in range(18):
        mid = 0.5 * (lo + hi)
        ctrl = _g2_quintic_controls(p0, t0, k0, p1, t1, k1, mid, mid)
        if ctrl is None:
            break
        pt = _bezier_point(ctrl, 0.5)
        d = _vdist(pt, target)
        if d < best_d:
            best_d = d
            best = ctrl
        toward = _vdot(_vsub(pt, p0), _vsub(target, p0))
        past = _vdist(pt, p0) > _vdist(target, p0) and toward > 0.0
        if past:
            hi = mid
        else:
            lo = mid
    return best


def _polyline_kappa(points, i):
    n = len(points)
    if n < 3:
        return 0.0
    a = points[(i - 1 + n) % n]
    b = points[i]
    c = points[(i + 1) % n]
    turn = _turn_at(a, b, c)
    ds = 0.5 * (_vdist(a, b) + _vdist(b, c))
    if ds < 1e-9:
        return 0.0
    return turn / ds


def _path_tangent(points, i, sense):
    n = len(points)
    j = (i + sense + n) % n
    t = _vunit(_vsub(points[j], points[i]))
    if t[0] == 0.0 and t[1] == 0.0:
        k = (i + 2 * sense + n) % n
        t = _vunit(_vsub(points[k], points[i]))
    return t


def _circular_g1_chain(a, b, c, radius, ring):
    t1 = _vunit(_vsub(b, a))
    t2 = _vunit(_vsub(c, b))
    if (t1[0] == 0.0 and t1[1] == 0.0) or (t2[0] == 0.0 and t2[1] == 0.0):
        return [b]
    turn = math.atan2(_vcross(t1, t2), _vdot(t1, t2))
    if abs(turn) < 0.08:
        return [b]
    tan_h = math.tan(abs(turn) * 0.5)
    if tan_h < 1e-8:
        return [b]
    trim = radius * tan_h
    p0 = (b[0] - t1[0] * trim, b[1] - t1[1] * trim)
    p1 = (b[0] + t2[0] * trim, b[1] + t2[1] * trim)
    left = (-t1[1], t1[0])
    inward = left if turn > 0.0 else (-left[0], -left[1])
    center = (p0[0] + inward[0] * radius, p0[1] + inward[1] * radius)
    return [p0] + _arc_points(center, p0, p1, radius, ring) + [p1]


def _fair_tip_blend(a, b, c, radius, ring, k0=0.0, k1=0.0):
    """G2 fair blend of about `radius` — same on every tip, eases out of both walls."""
    if radius < 0.04:
        return [b]
    t1 = _vunit(_vsub(b, a))
    t2 = _vunit(_vsub(c, b))
    if (t1[0] == 0.0 and t1[1] == 0.0) or (t2[0] == 0.0 and t2[1] == 0.0):
        return [b]
    turn = math.atan2(_vcross(t1, t2), _vdot(t1, t2))
    if abs(turn) < 0.08:
        return [b]
    if _vdot(t1, t2) < -0.90:
        return _end_cap_chain(b, a, c, radius, ring)
    tan_h = math.tan(abs(turn) * 0.5)
    if tan_h < 1e-8:
        return [b]
    # Longer than a circular trim so curvature eases in from the walls.
    trim = radius * tan_h * 1.65
    p0 = (b[0] - t1[0] * trim, b[1] - t1[1] * trim)
    p1 = (b[0] + t2[0] * trim, b[1] + t2[1] * trim)
    ctrl = _fair_g2_controls(p0, t1, k0, p1, t2, k1, radius, b)
    if ctrl is None:
        return _circular_g1_chain(a, b, c, radius, ring)
    return _sample_bezier(ctrl)


def _constant_radius_fillet(a, b, c, radius, ring):
    """Same-radius tip: G2 fair blend (hairpin stays a semicircle)."""
    return _fair_tip_blend(a, b, c, radius, ring)


def _cap_all_tips_same(moved, tips, radius, ring):
    """Replace every tip with the same G2 fair blend so they match and stay fair."""
    if len(moved) < 6 or not tips or radius < 0.04:
        return moved, []
    protected = []
    # Blend between points that already lie on the offset walls so the join stays G2.
    walk = max(radius * 2.6, 7.0)
    for tip in tips:
        if tip is None:
            continue
        i, _ = _nearest_index(moved, tip)
        p_l, i_l = _walk_along(moved, i, -1, walk)
        p_r, i_r = _walk_along(moved, i, 1, walk)
        t0 = _path_tangent(moved, i_l, 1)
        t1 = _path_tangent(moved, i_r, 1)
        k0 = _polyline_kappa(moved, i_l)
        k1 = _polyline_kappa(moved, i_r)
        corner = _line_intersect_unbounded(
            p_l,
            (p_l[0] + t0[0], p_l[1] + t0[1]),
            p_r,
            (p_r[0] - t1[0], p_r[1] - t1[1]),
        )
        if corner is None:
            corner = moved[i]
        ctrl = _fair_g2_controls(p_l, t0, k0, p_r, t1, k1, radius, corner)
        if ctrl is None:
            chain = _circular_g1_chain(
                (p_l[0] - t0[0], p_l[1] - t0[1]),
                corner,
                (p_r[0] + t1[0], p_r[1] + t1[1]),
                radius,
                ring,
            )
        else:
            chain = _sample_bezier(ctrl)
        if len(chain) < 2:
            continue
        nxt = _replace_span(moved, i_l, i_r, chain)
        if len(nxt) >= 3:
            moved = nxt
            protected.extend(chain)
    cleaned = _clean_ring(moved, 0.02) if len(moved) >= 3 else moved
    return cleaned, protected


def _apply_original_features(moved, feature_ring, interior_ring, samples, grow, min_width):
    """Slot-end semicircles, plus a parallel round join on band corners.

    Fillet radius is the wall grow — never MinWidth/2 on walls that only
    moved a millimetre, which is what turned the triangle into lollipops.
    """
    protected = []
    if len(moved) < 4 or min_width <= 0:
        return moved, protected
    radius = min_width * 0.5
    features = _original_features(feature_ring, samples, grow, min_width)
    for feat in features:
        if feat.get("kind") == "end":
            vertex = feat.get("center")
            splice_r = radius
            if feat.get("a") is not None and _is_tapered_end(vertex, samples, min_width):
                d = max(_grow_at_point(vertex, samples, grow, 0.0), 0.0)
                w0 = _sample_width_near(vertex, samples, 0.6)
                if w0 >= 1e8:
                    w0 = 0.6
                splice_r = max(w0 * 0.5 + d, 0.8)
                chain = _tapered_tip_chain(
                    vertex, moved, samples, grow, min_width, interior_ring
                )
            else:
                chain = _end_cap_chain(
                    feat["center"], feat["p0"], feat["p1"], radius, interior_ring
                )
            if len(chain) >= 2:
                nxt = _splice_tip(moved, chain, vertex, splice_r)
                if nxt is not moved and len(nxt) >= 3:
                    moved = nxt
                    protected.extend(chain)
            continue
        a = feat.get("a")
        b = feat.get("b") or feat.get("center")
        c = feat.get("c")
        if a is None or b is None or c is None:
            continue
        turn = _turn_at(a, b, c)
        # True band corner (the elbow). Skip hairpin transitions and sharp V tips.
        if abs(turn) < 1.0 or abs(turn) > 2.2:
            continue
        join_r = min(
            max(_grow_at_point(b, samples, grow, min_width * 0.25), 0.08),
            radius,
        )
        chain = _vertex_round_join(a, b, c, join_r, interior_ring)
        if len(chain) >= 2:
            nxt = _splice_feature(moved, chain, b, join_r)
            if nxt is not moved and len(nxt) >= 3:
                moved = nxt
                protected.extend(chain)
    cleaned = _clean_ring(moved, 0.02) if len(moved) >= 3 else moved
    return cleaned, protected


def _near_protected(point, protected, radius):
    if not protected:
        return False
    thresh = radius * 1.2
    for q in protected:
        if _vdist(point, q) < thresh:
            return True
    return False


def _round_short_ends(points, ring, min_width, protected=None, skip_centers=None):
    """Cap parallel slot ends with a semicircle of radius min_width/2."""
    n = len(points)
    if n < 8 or min_width <= 0:
        return points
    radius = min_width * 0.5
    used = [False] * n
    out = []
    i = 0
    while i < n:
        if skip_centers and any(
            _vdist(points[i], c) < max(min_width * 1.8, 6.0) for c in skip_centers if c is not None
        ):
            if not used[i]:
                out.append(points[i])
                used[i] = True
            i += 1
            continue
        if used[i] or _near_protected(points[i], protected, radius):
            if not used[i]:
                out.append(points[i])
                used[i] = True
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
        opposite_walls = _vdot(in_dir, out_dir) < -0.72
        blunt = (
            min_width * 0.45 < edge < min_width * 1.45
            and prev_len > max(edge * 1.4, min_width * 0.7)
            and next_len > max(edge * 1.4, min_width * 0.7)
            and abs(tb) > 0.65
            and abs(tc) > 0.65
            and tb * tc > 0
            and opposite_walls
        )
        if blunt and not used[(i + 1) % n]:
            out.extend(_semicircle_cap(b, c, radius, ring))
            used[i] = True
            used[(i + 1) % n] = True
            i += 2
            continue

        # Hairpin / leftover offset loop: only when the walls are opposite.
        # A 60° triangle corner turns ~120° and must stay a simple fillet.
        wall_dot = _vdot(_vunit(_vsub(b, a)), _vunit(_vsub(c, b)))
        if abs(tb) > 2.4 and wall_dot < -0.90:
            cap = None
            for dist in (radius * 0.75, radius, radius * 1.2, radius * 1.5):
                p0, i0 = _walk_along(points, i, -1, dist)
                p1, i1 = _walk_along(points, i, 1, dist)
                chord = _vdist(p0, p1)
                if min_width * 0.7 < chord < min_width * 1.25:
                    cap = (p0, p1, i0, i1)
                    break
            if cap is not None:
                p0, p1, i0, i1 = cap
                out.extend(_semicircle_cap(p0, p1, radius, ring))
                _mark_span(used, i0, i1, i, n)
                i += 1
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


def _raw_deltas(samples, min_width):
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
    return raw


def _closed_arc_prefix(samples):
    n = len(samples)
    pref = [0.0] * n
    for i in range(1, n):
        pref[i] = pref[i - 1] + _vdist(samples[i - 1]["point"], samples[i]["point"])
    total = pref[-1] + _vdist(samples[-1]["point"], samples[0]["point"])
    return pref, total


def _closed_arc_dist(pref, total, i, j):
    if i == j:
        return 0.0
    if i < j:
        fwd = pref[j] - pref[i]
    else:
        fwd = total - (pref[i] - pref[j])
    rev = total - fwd
    return fwd if fwd < rev else rev


def _smootherstep(t):
    """C2 fade. 0 and 1 are flat so the offset lands back on the original."""
    if t <= 0.0:
        return 0.0
    if t >= 1.0:
        return 1.0
    return t * t * t * (t * (t * 6.0 - 15.0) + 10.0)


def _distance_scaled_deltas(samples, raw, min_width):
    """Move each node by a falloff of the grow needed at the nearest pinch."""
    n = len(samples)
    if n < 3 or not any(d > 1e-4 for d in raw):
        return [0.0] * n, 0
    pref, total = _closed_arc_prefix(samples)
    pinch = [i for i, d in enumerate(raw) if d > 0.04]
    fade = max(min_width * 2.8, 10.0)
    deltas = [0.0] * n
    for i in range(n):
        best = raw[i]
        for p in pinch:
            if p == i:
                continue
            dist = _closed_arc_dist(pref, total, i, p)
            if dist >= fade:
                continue
            val = raw[p] * _smootherstep(1.0 - dist / fade)
            if val > best:
                best = val
        deltas[i] = best if best > 1e-4 else 0.0
    return deltas, _thin_runs([d > 0.08 for d in deltas])


def _compute_deltas(samples, spacing, min_width):
    """Per-node outward grow, scaled by distance along the curve from the pinch."""
    if len(samples) < 3 or min_width <= 0:
        return [], 0
    raw = _raw_deltas(samples, min_width)
    return _distance_scaled_deltas(samples, raw, min_width)


def _solve_linear_nd(matrix, rhs):
    """Gaussian elimination with partial pivoting. rhs is n x k."""
    n = len(matrix)
    if n == 0 or len(rhs) != n:
        return None
    k = len(rhs[0])
    rows = []
    for i in range(n):
        rows.append([float(matrix[i][j]) for j in range(n)] + [float(rhs[i][c]) for c in range(k)])
    for col in range(n):
        pivot = col
        best = abs(rows[col][col])
        for r in range(col + 1, n):
            val = abs(rows[r][col])
            if val > best:
                best = val
                pivot = r
        if best < 1e-14:
            return None
        if pivot != col:
            rows[col], rows[pivot] = rows[pivot], rows[col]
        scale = rows[col][col]
        inv = 1.0 / scale
        for c in range(col, n + k):
            rows[col][c] *= inv
        for r in range(n):
            if r == col:
                continue
            fac = rows[r][col]
            if abs(fac) < 1e-18:
                continue
            for c in range(col, n + k):
                rows[r][c] -= fac * rows[col][c]
    return [[rows[i][n + c] for c in range(k)] for i in range(n)]


def _periodic_cubic_derivs(pts):
    """C2 periodic cubic first derivatives (dS/dt, t = chord length)."""
    n = len(pts)
    if n < 3:
        return None
    h = []
    for i in range(n):
        d = _vdist(pts[i], pts[(i + 1) % n])
        h.append(d if d > 1e-8 else 1e-8)
    matrix = [[0.0] * n for _ in range(n)]
    rhs = [[0.0, 0.0] for _ in range(n)]
    for i in range(n):
        im = (i - 1 + n) % n
        ip = (i + 1) % n
        hm = h[im]
        hi = h[i]
        matrix[i][im] = hm
        matrix[i][i] = 2.0 * (hm + hi)
        matrix[i][ip] = hi
        dx0 = pts[i][0] - pts[im][0]
        dy0 = pts[i][1] - pts[im][1]
        dx1 = pts[ip][0] - pts[i][0]
        dy1 = pts[ip][1] - pts[i][1]
        rhs[i][0] = 3.0 * (dx1 * hm / hi + dx0 * hi / hm)
        rhs[i][1] = 3.0 * (dy1 * hm / hi + dy0 * hi / hm)
    return _solve_linear_nd(matrix, rhs)


def _hermite_point(p0, p1, d0, d1, span, u):
    u2 = u * u
    u3 = u2 * u
    h00 = 2.0 * u3 - 3.0 * u2 + 1.0
    h10 = u3 - 2.0 * u2 + u
    h01 = -2.0 * u3 + 3.0 * u2
    h11 = u3 - u2
    return (
        h00 * p0[0] + h10 * span * d0[0] + h01 * p1[0] + h11 * span * d1[0],
        h00 * p0[1] + h10 * span * d0[1] + h01 * p1[1] + h11 * span * d1[1],
    )


def _sample_periodic_cubic(pts, spacing=0.35):
    """Evaluate a closed C2 cubic interpolant. Output is a dense polyline."""
    if len(pts) < 3:
        return list(pts)
    derivs = _periodic_cubic_derivs(pts)
    if derivs is None:
        return list(pts)
    n = len(pts)
    out = []
    for i in range(n):
        p0 = pts[i]
        p1 = pts[(i + 1) % n]
        d0 = derivs[i]
        d1 = derivs[(i + 1) % n]
        span = _vdist(p0, p1)
        if span < 1e-10:
            continue
        steps = max(2, int(math.ceil(span / max(spacing, 0.08))))
        for s in range(steps):
            out.append(_hermite_point(p0, p1, d0, d1, span, s / float(steps)))
    cleaned = _clean_ring(out, min(spacing * 0.25, 0.05))
    return cleaned if len(cleaned) >= 3 else list(pts)


def _arc_length_resample(points, count, closed=True):
    """Evenly spaced samples along a (closed) polyline, few enough to stay fair."""
    if len(points) < 2 or count < 3:
        return list(points)
    n = len(points)
    segs = n if closed else (n - 1)
    lengths = []
    total = 0.0
    for i in range(segs):
        d = _vdist(points[i], points[(i + 1) % n])
        lengths.append(d)
        total += d
    if total < 1e-9:
        return list(points)
    out = []
    denom = float(count) if closed else float(count - 1)
    for k in range(count):
        target = total * (k / denom)
        acc = 0.0
        chosen = points[0]
        for i in range(segs):
            seg = lengths[i]
            if acc + seg >= target - 1e-12 or i == segs - 1:
                t = 0.0 if seg < 1e-12 else (target - acc) / seg
                if t < 0.0:
                    t = 0.0
                elif t > 1.0:
                    t = 1.0
                a = points[i]
                b = points[(i + 1) % n]
                chosen = (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
                break
            acc += seg
        out.append(chosen)
    return _clean_ring(out, 1e-6) if closed else out


def _drop_near_tips(points, tips, omit):
    """Keep wall samples; drop the miter / house vertices at sharp tips."""
    if not points or not tips or omit <= 0.0:
        return list(points)
    kept = []
    for p in points:
        near = False
        for tip in tips:
            if tip is None:
                continue
            if _vdist(p, tip) < omit:
                near = True
                break
        if not near:
            kept.append(p)
    if len(kept) < 6:
        return list(points)
    return kept


def _fair_ctrl_count(n_tips):
    return min(64, max(24, max(int(n_tips), 1) * 8))


def _bspline_cubic_point(cvs, u):
    """Uniform periodic cubic B-spline. u in [0, n)."""
    n = len(cvs)
    if n < 3:
        return cvs[0] if cvs else (0.0, 0.0)
    while u < 0.0:
        u += float(n)
    i = int(math.floor(u)) % n
    t = u - math.floor(u)
    t2 = t * t
    t3 = t2 * t
    b0 = (1.0 - t) ** 3 / 6.0
    b1 = (3.0 * t3 - 6.0 * t2 + 4.0) / 6.0
    b2 = (-3.0 * t3 + 3.0 * t2 + 3.0 * t + 1.0) / 6.0
    b3 = t3 / 6.0
    p0 = cvs[(i - 1 + n) % n]
    p1 = cvs[i]
    p2 = cvs[(i + 1) % n]
    p3 = cvs[(i + 2) % n]
    return (
        b0 * p0[0] + b1 * p1[0] + b2 * p2[0] + b3 * p3[0],
        b0 * p0[1] + b1 * p1[1] + b2 * p2[1] + b3 * p3[1],
    )


def _sample_periodic_bspline(cvs, spacing=0.35):
    """Dense polyline of a closed cubic B-spline — one fair curve, few CVs."""
    n = len(cvs)
    if n < 4:
        return list(cvs)
    perim = 0.0
    for i in range(n):
        perim += _vdist(cvs[i], cvs[(i + 1) % n])
    steps = max(n * 8, int(math.ceil(perim / max(spacing, 0.08))))
    out = [_bspline_cubic_point(cvs, n * s / float(steps)) for s in range(steps)]
    cleaned = _clean_ring(out, min(spacing * 0.25, 0.05))
    return cleaned if len(cleaned) >= 4 else list(cvs)


def _offset_miter(a, b, c, distance, ring):
    """Intersection of the two walls offset by `distance` — outside the tip."""
    if a is None or b is None or c is None or distance <= 1e-9:
        return None
    out1 = _wall_outward(a, b, ring)
    out2 = _wall_outward(b, c, ring)
    if (out1[0] == 0.0 and out1[1] == 0.0) or (out2[0] == 0.0 and out2[1] == 0.0):
        return None
    g1a = _vadd(a, _vmul(out1, distance))
    g1b = _vadd(b, _vmul(out1, distance))
    g2b = _vadd(b, _vmul(out2, distance))
    g2c = _vadd(c, _vmul(out2, distance))
    corner = _line_intersect_unbounded(g1a, g1b, g2b, g2c)
    if corner is None:
        bis = _vunit(_vadd(out1, out2))
        if bis[0] == 0.0 and bis[1] == 0.0:
            return None
        corner = _vadd(b, _vmul(bis, distance))
    # Must sit outside the original opening, otherwise the spline cuts the tip.
    if _point_in_ring(corner, ring):
        bis = _vunit(_vsub(b, corner))
        if bis[0] == 0.0 and bis[1] == 0.0:
            return None
        corner = _vadd(b, _vmul(bis, max(distance, 0.4)))
    return corner


def _fair_controls(offset, features, samples, grow, interior, min_width):
    """Few wall CVs plus one outside miter CV at each fillet tip."""
    feats = []
    for feat in features or []:
        if feat.get("kind") != "fillet":
            continue
        tip = feat.get("b") or feat.get("center")
        if tip is None:
            continue
        feats.append(feat)
    tips = [f.get("b") or f.get("center") for f in feats]
    if len(offset) < 6:
        return None
    if not tips:
        return _arc_length_resample(offset, _fair_ctrl_count(0), True)

    omit = max(min_width * 0.45, 2.0)
    n = len(offset)
    near = []
    for p in offset:
        flag = False
        for tip in tips:
            if _vdist(p, tip) < omit:
                flag = True
                break
        near.append(flag)
    if all(near) or not any(near):
        return _arc_length_resample(offset, _fair_ctrl_count(len(tips)), True)

    start = 0
    while start < n and near[start]:
        start += 1
    if start >= n:
        return _arc_length_resample(offset, _fair_ctrl_count(len(tips)), True)
    guard = 0
    while not near[(start - 1 + n) % n] and guard < n:
        start = (start - 1 + n) % n
        guard += 1
        if start == 0 and not near[n - 1]:
            break

    walls = []
    miters = []
    cur = []
    for k in range(n):
        i = (start + k) % n
        if not near[i]:
            cur.append(offset[i])
            continue
        if not cur:
            continue
        tip = min(tips, key=lambda t: _vdist(offset[i], t))
        feat = None
        for cand in feats:
            tb = cand.get("b") or cand.get("center")
            if tb is not None and _vdist(tb, tip) < 1e-6:
                feat = cand
                break
        dist = _grow_at_point(tip, samples, grow, min_width * 0.25)
        dist = max(dist, min_width * 0.2)
        tip_cvs = None
        if feat is not None:
            chain = _vertex_round_join(feat.get("a"), tip, feat.get("c"), dist, interior)
            if chain and len(chain) >= 2:
                tip_cvs = _arc_length_resample(chain, 5, False)
            if not tip_cvs:
                miter = _offset_miter(feat.get("a"), tip, feat.get("c"), dist, interior)
                tip_cvs = [miter] if miter is not None else None
        if not tip_cvs:
            bis = _vunit(_vsub(tip, cur[-1])) if cur else (0.0, 0.0)
            tip_cvs = [_vadd(tip, _vmul(bis, dist))] if bis[0] or bis[1] else None
        walls.append(cur)
        miters.append(tip_cvs)
        cur = []
    if cur:
        if walls:
            walls[0] = cur + walls[0]
        else:
            walls.append(cur)
            miters.append(None)

    if len(walls) < 2:
        return _arc_length_resample(offset, _fair_ctrl_count(len(tips)), True)

    per = 7
    ctrl = []
    for wall, tip_cvs in zip(walls, miters):
        if len(wall) >= 2:
            ctrl.extend(_arc_length_resample(wall, per, False))
        elif wall:
            ctrl.append(wall[0])
        if tip_cvs:
            ctrl.extend(tip_cvs)
    ctrl = _clean_ring(ctrl, 0.05)
    return ctrl if len(ctrl) >= 6 else None


def _fair_curve_from_offset(offset, tips, min_width, features=None, samples=None, grow=None, interior=None):
    """One closed periodic cubic through the offset walls — no grafted caps."""
    if len(offset) < 6:
        return None
    interior = interior if interior is not None else offset
    ctrl = _fair_controls(offset, features, samples, grow, interior, min_width)
    if ctrl is None or len(ctrl) < 6:
        kept = _drop_near_tips(offset, tips, max(min_width * 0.45, 2.0))
        ctrl = _arc_length_resample(kept if len(kept) >= 6 else offset, _fair_ctrl_count(len(tips) if tips else 0), True)
    if ctrl is None or len(ctrl) < 6:
        return None
    sampled = _sample_periodic_bspline(ctrl, 0.35)
    if len(sampled) < 6 or _polyline_self_intersects(sampled):
        sampled = _sample_periodic_cubic(ctrl, 0.35)
    if len(sampled) < 3 or _polyline_self_intersects(sampled):
        return None
    return sampled


def _uniform_outward_offset(ring, distance, interior):
    """Parallel of the original ring — keeps the same fairness as the input walls."""
    n = len(ring)
    if n < 3 or distance <= 1e-9:
        return list(ring)
    out = []
    for i in range(n):
        prev = ring[(i - 1 + n) % n]
        nxt = ring[(i + 1) % n]
        tangent = _vunit(_vsub(nxt, prev))
        if tangent[0] == 0.0 and tangent[1] == 0.0:
            tangent = _vunit(_vsub(nxt, ring[i]))
        inward = _vleft(tangent)
        probe = _vadd(ring[i], _vmul(inward, 0.25))
        if not _point_in_ring(probe, interior):
            inward = _vmul(inward, -1.0)
        out.append(_vadd(ring[i], _vmul(inward, -distance)))
    return _clean_ring(out, 0.02) if len(out) >= 3 else out


def _rebuild_fair_opening(ring, features, min_width, interior, distance=None):
    """Uniform parallel of the walls, then one periodic cubic through those walls."""
    if distance is None or distance <= 1e-9:
        distance = min_width * 0.5
    offset = _uniform_outward_offset(ring, distance, interior)
    if len(offset) < 6:
        return None
    tips = [f.get("b") or f.get("center") for f in features if f.get("kind") == "fillet"]
    tips = [t for t in tips if t is not None]
    grow = [distance] * max(len(offset), 1)
    fake_samples = [{"point": p} for p in offset]
    return _fair_curve_from_offset(
        offset, tips, min_width, features, fake_samples, grow, interior
    )


def _fillet_tips(features):
    tips = []
    for feat in features or []:
        if feat.get("kind") != "fillet":
            continue
        tip = feat.get("b") or feat.get("center")
        if tip is not None:
            tips.append(tip)
    return tips


def _apply_min_width(ring, samples, spacing, min_width, round_corners=True):
    if len(ring) < 3 or min_width <= 0:
        return (ring, 0)
    if len(samples) < 3:
        return (ring, 0)
    deltas, pinches = _compute_deltas(samples, spacing, min_width)
    if pinches == 0:
        return (ring, 0)
    raw = _raw_deltas(samples, min_width)
    features = _original_features(ring, samples, deltas, min_width)
    tips = _fillet_tips(features)
    if (
        _mostly_thin(raw)
        and tips
        and all(f.get("kind") == "fillet" for f in features)
    ):
        rebuilt = _rebuild_fair_opening(
            ring, features, min_width, ring, _median_positive(raw)
        )
        if rebuilt is not None:
            return (rebuilt, pinches)
    moved = [_offset_point(samples[i], deltas[i], ring) for i in range(len(samples))]
    cleaned = _remove_loops(_clean_ring(moved, 0.02))
    # Only rebuild the whole outline when every pinch feature is a fillet.
    # Mixed slot-ends + a corner (hairpin, elbow) keep their end caps.
    if (
        len(cleaned) >= 3
        and tips
        and features
        and all(f.get("kind") == "fillet" for f in features)
    ):
        faired = _fair_curve_from_offset(
            cleaned, tips, min_width, features, samples, deltas, ring
        )
        if faired is not None:
            cleaned = faired
    if len(cleaned) >= 3:
        cleaned, protected = _apply_original_features(
            cleaned, ring, ring, samples, deltas, min_width
        )
        taper_tips = [
            f.get("center")
            for f in features
            if f.get("kind") == "end"
            and f.get("a") is not None
            and _is_tapered_end(f.get("center"), samples, min_width)
        ]
        cleaned = _round_short_ends(cleaned, ring, min_width, protected, taper_tips)
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


def _curve_corners_xy(curve, plane):
    """Original polyline vertices / C1 kinks — the CAD corners to fillet or cap."""
    pts = []
    try:
        ok, pline = curve.TryGetPolyline()
        if ok and pline is not None:
            count = pline.Count
            for i in range(count):
                pts.append(_to_xy(plane, pline[i]))
            if len(pts) > 2 and _vdist(pts[0], pts[-1]) <= max(_tol(), 0.02):
                pts.pop()
            if len(pts) >= 3:
                return _ensure_ccw(_clean_ring(pts, max(_tol(), 0.02)))
    except Exception:
        pts = []
    try:
        domain = curve.Domain
        t = domain.Min
        pts = [_to_xy(plane, curve.PointAt(t))]
        while True:
            got = curve.GetNextDiscontinuity(rg.Continuity.C1_locus, t, domain.Max)
            if isinstance(got, tuple):
                ok, t = got[0], got[1]
            else:
                ok = bool(got)
                if not ok:
                    break
                t = got
            if not ok:
                break
            pts.append(_to_xy(plane, curve.PointAt(t)))
        if curve.IsClosed and len(pts) > 1 and _vdist(pts[0], pts[-1]) <= max(_tol(), 0.02):
            pts.pop()
        if len(pts) >= 3:
            return _ensure_ccw(_clean_ring(pts, max(_tol(), 0.02)))
    except Exception:
        pass
    return []


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
    """Dense interpolant — do not use for the baked outline (few-node NURBS only)."""
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


def _force_closed_curve(curve):
    """The finished outline must always be a closed curve."""
    if curve is None or not curve.IsValid:
        return curve
    if curve.IsClosed:
        return curve
    try:
        if curve.MakeClosed(_tol() * 4) and curve.IsClosed:
            return curve
    except Exception:
        pass
    try:
        start = curve.PointAtStart
        end = curve.PointAtEnd
        if start.DistanceTo(end) > _tol():
            closer = rg.LineCurve(end, start)
            joined = rg.Curve.JoinCurves([curve, closer], _tol() * 4, True)
            got = _as_list(joined)
            if got:
                curve = got[0]
        if not curve.IsClosed:
            curve.MakeClosed(_tol() * 4)
    except Exception:
        pass
    return curve


def _median_positive(values):
    pos = [v for v in values if v > 0.04]
    if not pos:
        return 0.0
    pos = sorted(pos)
    n = len(pos)
    if n % 2:
        return pos[n // 2]
    return 0.5 * (pos[n // 2 - 1] + pos[n // 2])


def _mostly_thin(raw, frac=0.55):
    if not raw:
        return False
    return sum(1 for d in raw if d > 0.04) >= frac * len(raw)


def _grow_opening(curve, distance, corners="Round"):
    """Parallel offset that enlarges a closed opening, with round corners."""
    if curve is None or distance <= _tol():
        return None
    plane = _curve_plane(curve)
    candidates = []
    for signed in (distance, -distance):
        candidates.extend(_offset(curve, plane, signed, corners))
    valid = [c for c in candidates if c is not None and c.IsValid]
    if not valid:
        return None

    def _area(crv):
        try:
            amp = rg.AreaMassProperties.Compute(crv)
            if amp is not None:
                return abs(amp.Area)
        except Exception:
            pass
        try:
            return crv.GetLength()
        except Exception:
            return 0.0

    best = max(valid, key=_area)
    if _curve_self_intersects(best):
        return None
    return _force_closed_curve(best)


def _kink_parameters(curve):
    """C1 breaks around a closed curve, including a kink at Domain.Min."""
    ts = []
    try:
        domain = curve.Domain
    except Exception:
        return ts
    t = domain.Min
    try:
        if curve.IsClosed and not curve.IsContinuous(rg.Continuity.C1_locus, domain.Min):
            ts.append(domain.Min)
    except Exception:
        pass
    guard = 0
    while guard < 64:
        guard += 1
        try:
            got = curve.GetNextDiscontinuity(rg.Continuity.C1_locus, t, domain.Max)
        except Exception:
            break
        ok = False
        nxt = t
        if isinstance(got, tuple):
            if len(got) >= 2:
                ok, nxt = bool(got[0]), got[1]
        elif got:
            ok = True
            nxt = got
        if not ok:
            break
        t = nxt
        if ts and abs(t - ts[0]) <= _tol():
            break
        ts.append(t)
        if t >= domain.Max - _tol():
            break
    return ts


def _polyline_spans(curve):
    try:
        ok, pline = curve.TryGetPolyline()
    except Exception:
        return []
    if not ok or pline is None:
        return []
    try:
        count = pline.Count
    except Exception:
        return []
    pts = [pline[i] for i in range(count)]
    if len(pts) > 2 and pts[0].DistanceTo(pts[-1]) <= _tol() * 4:
        pts = pts[:-1]
    if len(pts) < 2:
        return []
    spans = []
    n = len(pts)
    last = n if curve.IsClosed else n - 1
    for i in range(last):
        a = pts[i]
        b = pts[(i + 1) % n]
        if a.DistanceTo(b) <= _tol():
            continue
        try:
            spans.append(rg.LineCurve(a, b))
        except Exception:
            pass
    return spans


def _split_closed_spans(curve):
    """Wall spans between original kinks — keeps each NURBS piece intact."""
    poly = _polyline_spans(curve)
    if len(poly) >= 2:
        return poly
    ts = _kink_parameters(curve)
    if len(ts) >= 1:
        try:
            pieces = curve.Split(ts)
        except Exception:
            pieces = None
        got = [c for c in _as_list(pieces) if c is not None and c.IsValid]
        if len(got) >= 2:
            return got
    dup = curve.DuplicateCurve()
    return [dup] if dup else []


def _span_length(curve):
    try:
        return curve.GetLength()
    except Exception:
        return 0.0


def _drop_short_tip_spans(spans, radius):
    """Drop pre-rounded tip blends so every tip can take the same fillet."""
    if len(spans) <= 2:
        return spans
    lengths = [_span_length(s) for s in spans]
    ordered = sorted(lengths)
    typical = ordered[len(ordered) // 2]
    keep = []
    for span, length in zip(spans, lengths):
        if length < max(radius * 0.85, 0.4) and length < typical * 0.35:
            continue
        keep.append(span)
    return keep if len(keep) >= 2 else spans


def _span_mid_xy(span, plane):
    try:
        t = 0.5 * (span.Domain.Min + span.Domain.Max)
        return _to_xy(plane, span.PointAt(t))
    except Exception:
        return None


def _span_outward_offset(span, plane, distance, ring):
    """Offset one open wall away from the opening interior."""
    if span is None:
        return None
    if distance <= _tol():
        dup = span.DuplicateCurve()
        return dup if dup and dup.IsValid else span
    candidates = []
    for signed in (distance, -distance):
        candidates.extend(_offset(span, plane, signed, "Sharp"))
    best = None
    best_score = -1e300
    orig_mid = _span_mid_xy(span, plane)
    for cand in candidates:
        if cand is None or not cand.IsValid:
            continue
        mid = _span_mid_xy(cand, plane)
        if mid is None:
            continue
        inside = _point_in_ring(mid, ring)
        score = -1000.0 if inside else 1000.0
        score += _span_length(cand)
        if orig_mid is not None and not inside:
            score += _vdist(mid, orig_mid)
        if score > best_score:
            best = cand
            best_score = score
    return best


def _orient_span_loop(spans):
    """Flip each span so End of i sits next to Start of i+1."""
    if not spans:
        return []
    out = [s.DuplicateCurve() for s in spans]
    if len(out) == 1:
        return out
    for i in range(1, len(out)):
        prev_end = out[i - 1].PointAtEnd
        if out[i].PointAtStart.DistanceTo(prev_end) > out[i].PointAtEnd.DistanceTo(prev_end):
            out[i].Reverse()
    if out[0].PointAtStart.DistanceTo(out[-1].PointAtEnd) > out[0].PointAtEnd.DistanceTo(out[-1].PointAtEnd):
        out[0].Reverse()
        for i in range(1, len(out)):
            prev_end = out[i - 1].PointAtEnd
            if out[i].PointAtStart.DistanceTo(prev_end) > out[i].PointAtEnd.DistanceTo(prev_end):
                out[i].Reverse()
    return out


def _length_parameter(curve, length):
    try:
        got = curve.LengthParameter(max(0.0, length))
    except Exception:
        return None
    if isinstance(got, tuple):
        if len(got) >= 2 and got[0]:
            return got[1]
        return None
    return got


def _nurbs_from_xy_controls(plane, cvs, degree=5):
    if not cvs or len(cvs) < 2:
        return None
    deg = min(int(degree), len(cvs) - 1)
    pts = [_from_xy(plane, p) for p in cvs]
    try:
        crv = rg.NurbsCurve.Create(False, deg, pts)
        if crv is not None and crv.IsValid:
            return crv
    except Exception:
        pass
    return None


def _nurbs_periodic_from_ring(plane, ring, n_ctrl):
    """Few-node periodic cubic B-spline of a fair ring.

    Never CreateInterpolatedCurve: a chord interpolant through a handful of
    tip samples overshoots into loops, spikes, and an open curve that
    `_force_closed_curve` then stitches with a straight line.
    """
    if plane is None or len(ring) < 6:
        return None
    ctrl = _arc_length_resample(ring, max(6, int(n_ctrl)), True)
    if len(ctrl) < 6:
        return None
    pts = [_from_xy(plane, p) for p in ctrl]
    try:
        crv = rg.NurbsCurve.Create(True, 3, pts)
    except Exception:
        crv = None
    if crv is None or not getattr(crv, "IsValid", False):
        return None
    crv = _force_closed_curve(crv)
    if crv is None or not crv.IsValid or _curve_self_intersects(crv):
        return None
    return crv


def _nurbs_usable(curve):
    """Closed, valid, and not self-crossing — reject interpolant wreckage."""
    if curve is None or not getattr(curve, "IsValid", False):
        return False
    curve = _force_closed_curve(curve)
    if curve is None or not getattr(curve, "IsValid", False):
        return False
    if not getattr(curve, "IsClosed", False):
        return False
    if _curve_self_intersects(curve):
        return False
    return True


def _fair_blend_between(a, b, radius, plane, ring):
    """G2 fair blend between two offset walls. Few-CV NURBS, never an interpolant."""
    if a is None or b is None or radius < _tol():
        return None
    la = _span_length(a)
    lb = _span_length(b)
    if la < _tol() or lb < _tol():
        return None
    blend_len = min(max(radius * 2.8, 5.0), la * 0.38, lb * 0.38)
    tA = _length_parameter(a, max(la - blend_len, la * 0.55))
    tB = _length_parameter(b, min(blend_len, lb * 0.45))
    if tA is None:
        tA = a.Domain.Max
    if tB is None:
        tB = b.Domain.Min

    continuity = None
    try:
        continuity = rg.BlendContinuity.Curvature
    except Exception:
        try:
            continuity = rg.BlendContinuity.Tangency
        except Exception:
            continuity = None

    candidates = []
    if continuity is not None:
        for rev_a, rev_b in ((False, False), (False, True), (True, False), (True, True)):
            try:
                crv = rg.Curve.CreateBlendCurve(
                    a, tA, rev_a, continuity, b, tB, rev_b, continuity
                )
            except Exception:
                crv = None
            for piece in _as_list(crv):
                if piece is not None and getattr(piece, "IsValid", True) and _span_length(piece) > _tol():
                    candidates.append(piece)
        try:
            crv = rg.Curve.CreateBlendCurve(a, b, continuity)
        except Exception:
            crv = None
        for piece in _as_list(crv):
            if piece is not None and getattr(piece, "IsValid", True) and _span_length(piece) > _tol():
                candidates.append(piece)

    def _score(crv):
        try:
            mid = crv.PointAtNormalizedLength(0.5)
        except Exception:
            mid = crv.PointAt(0.5 * (crv.Domain.Min + crv.Domain.Max))
        xy = _to_xy(plane, mid)
        outside = 0.0 if _point_in_ring(xy, ring) else 1000.0
        return outside + _span_length(crv)

    if candidates:
        return max(candidates, key=_score)

    # Geometric quintic — 6 CVs, still a fair few-node NURBS.
    try:
        pa = _to_xy(plane, a.PointAt(tA))
        pb = _to_xy(plane, b.PointAt(tB))
        ta = a.TangentAt(tA)
        tb = b.TangentAt(tB)
        t0 = _vunit((ta * plane.XAxis, ta * plane.YAxis))
        t1 = _vunit((tb * plane.XAxis, tb * plane.YAxis))
        k0 = 0.0
        k1 = 0.0
        try:
            k0 = a.CurvatureAt(tA).Length
            k1 = b.CurvatureAt(tB).Length
            # Signed by whether curvature points left of the tangent.
            c0 = a.CurvatureAt(tA)
            c1 = b.CurvatureAt(tB)
            left0 = (-t0[1], t0[0])
            left1 = (-t1[1], t1[0])
            k0 = _vdot(left0, (c0 * plane.XAxis, c0 * plane.YAxis))
            k1 = _vdot(left1, (c1 * plane.XAxis, c1 * plane.YAxis))
        except Exception:
            k0 = 0.0
            k1 = 0.0
        corner = _line_intersect_unbounded(
            pa, (pa[0] + t0[0], pa[1] + t0[1]),
            pb, (pb[0] - t1[0], pb[1] - t1[1]),
        )
        ctrl = _fair_g2_controls(pa, t0, k0, pb, t1, k1, radius, corner)
        if ctrl:
            return _nurbs_from_xy_controls(plane, ctrl, 5)
    except Exception:
        pass
    return None


def _arc_from_chain(plane, chain, radius, ring):
    """Build one ArcCurve through a 2D G1 chain (p0 + arc + p1). Slot ends only."""
    if not chain or len(chain) < 2:
        return None
    p0 = chain[0]
    p1 = chain[-1]
    mid = chain[len(chain) // 2]
    try:
        arc = rg.Arc(_from_xy(plane, p0), _from_xy(plane, mid), _from_xy(plane, p1))
        if arc.IsValid:
            crv = rg.ArcCurve(arc)
            if crv is not None and crv.IsValid:
                return crv
    except Exception:
        pass
    return rg.LineCurve(_from_xy(plane, p0), _from_xy(plane, p1))


def _geometric_fillet_arc(a, b, radius, plane, ring):
    """Fallback G2 quintic when CreateBlendCurve is not available."""
    return _fair_blend_between(a, b, radius, plane, ring)


def _end_cap_arc(feat, radius, plane, ring):
    chain = _end_cap_chain(feat["center"], feat["p0"], feat["p1"], radius, ring)
    return _arc_from_chain(plane, chain, radius, ring)


def _closest_end_feature(features, point_xy, radius):
    best = None
    best_d = radius * 4.0
    for feat in features:
        if feat.get("kind") != "end":
            continue
        center = feat.get("center")
        if center is None:
            continue
        d = _vdist(point_xy, center)
        if d < best_d:
            best = feat
            best_d = d
    return best


def _span_end_xy(span, plane, at_start):
    pt = span.PointAtStart if at_start else span.PointAtEnd
    return _to_xy(plane, pt)


def _nearest_span_index(spans, point_xy, plane):
    best_i = 0
    best = 1e300
    for i, span in enumerate(spans):
        mid = _span_mid_xy(span, plane)
        if mid is None:
            continue
        d = _vdist(mid, point_xy)
        if d < best:
            best = d
            best_i = i
    return best_i


def _touch_param(wall, cap, prefer_start):
    """Parameter on `wall` nearest the cap end that lands on this wall."""
    if wall is None or cap is None:
        return None
    pts = [cap.PointAtStart, cap.PointAtEnd]
    best_t = None
    best_d = 1e300
    prefer = wall.PointAtStart if prefer_start else wall.PointAtEnd
    for pt in pts:
        try:
            ok, t = wall.ClosestPoint(pt)
        except Exception:
            ok, t = False, None
        if not ok:
            continue
        d = wall.PointAt(t).DistanceTo(pt)
        d += wall.PointAt(t).DistanceTo(prefer) * 0.01
        if d < best_d:
            best_d = d
            best_t = t
    return best_t


def _trim_wall_to_caps(wall, cap_prev, cap_next):
    t0 = _touch_param(wall, cap_prev, True)
    t1 = _touch_param(wall, cap_next, False)
    if t0 is None or t1 is None:
        return wall
    if abs(t1 - t0) <= _tol():
        return wall
    if t0 > t1:
        try:
            wall = wall.DuplicateCurve()
            wall.Reverse()
        except Exception:
            return wall
        t0 = _touch_param(wall, cap_prev, True)
        t1 = _touch_param(wall, cap_next, False)
        if t0 is None or t1 is None or t0 > t1 or abs(t1 - t0) <= _tol():
            return wall
    try:
        trimmed = wall.Trim(t0, t1)
        if trimmed is not None and trimmed.IsValid:
            return trimmed
    except Exception:
        pass
    return wall


def _join_closed_pieces(pieces):
    valid = [p for p in pieces if p is not None and getattr(p, "IsValid", True)]
    if not valid:
        return None
    joined = _join(valid)
    if not joined:
        return None
    best = max(joined, key=_span_length)
    best = _force_closed_curve(best)
    if best is None or not best.IsValid:
        return None
    if _curve_self_intersects(best):
        return None
    return best


def _nurbs_same_tip_caps(curve, distance, radius, plane, ring, features):
    """Offset the original spans and close every tip with the same G2 fair blend.

    Output is a PolyCurve of the original-quality walls plus few-CV NURBS
    blends — never a dense interpolant.
    """
    if curve is None or radius < _tol():
        return None
    spans = _drop_short_tip_spans(_split_closed_spans(curve), radius)
    if not spans:
        return None

    if len(spans) == 1:
        grown = _grow_opening(curve, distance, "Sharp") if distance > _tol() else curve.DuplicateCurve()
        return _force_closed_curve(grown)

    offset_spans = []
    for span in spans:
        off = _span_outward_offset(span, plane, distance, ring)
        if off is None:
            line = None
            try:
                line = rg.LineCurve(span.PointAtStart, span.PointAtEnd)
            except Exception:
                line = None
            off = _span_outward_offset(line, plane, distance, ring) if line else None
        if off is None:
            return None
        offset_spans.append(off)
    offset_spans = _orient_span_loop(offset_spans)
    n = len(offset_spans)
    if n < 2:
        return None

    keep = [True] * n
    end_for_span = [None] * n
    for feat in features:
        if feat.get("kind") != "end":
            continue
        center = feat.get("center")
        if center is None:
            continue
        idx = _nearest_span_index(offset_spans, center, plane)
        # Only drop a short end-wall, not a long side that merely sits near a tip.
        if _span_length(offset_spans[idx]) < max(radius * 3.2, 8.0):
            keep[idx] = False
            end_for_span[idx] = feat

    kept_idx = [i for i in range(n) if keep[i]]
    if len(kept_idx) < 2:
        kept_idx = list(range(n))
        keep = [True] * n

    caps = []
    walls = []
    m = len(kept_idx)
    for k in range(m):
        i = kept_idx[k]
        j = kept_idx[(k + 1) % m]
        walls.append(offset_spans[i])
        dropped = []
        t = (i + 1) % n
        while t != j:
            dropped.append(t)
            t = (t + 1) % n
        feat = None
        for d in dropped:
            if end_for_span[d] is not None:
                feat = end_for_span[d]
                break
        joint = _span_end_xy(offset_spans[i], plane, False)
        if feat is None:
            feat = _closest_end_feature(features, joint, radius)
        if dropped:
            kind_end = feat is not None
        elif feat is not None:
            kind_end = _vdist(joint, feat["center"]) < radius * 1.6
        else:
            kind_end = False

        a = offset_spans[i]
        b = offset_spans[j]
        cap = None
        if kind_end and feat is not None:
            cap = _end_cap_arc(feat, radius, plane, ring)
        if cap is None:
            cap = _fair_blend_between(a, b, radius, plane, ring)
        if cap is None:
            cap = _geometric_fillet_arc(a, b, radius, plane, ring)
        if cap is None:
            return None
        caps.append(cap)

    pieces = []
    for k in range(m):
        wall = _trim_wall_to_caps(walls[k], caps[(k - 1 + m) % m], caps[k])
        pieces.append(wall)
        pieces.append(caps[k])
    return _join_closed_pieces(pieces)


def _move_nurbs_cvs(curve, plane, samples, deltas, ring):
    """Slide the original CVs — same few nodes, local grow, no interpolant."""
    if curve is None or not samples or not deltas:
        return None
    try:
        nurbs = curve.ToNurbsCurve()
    except Exception:
        return None
    if nurbs is None or not nurbs.IsValid:
        return None
    try:
        count = nurbs.Points.Count
    except Exception:
        return None
    if count < 3:
        return None
    greville = None
    try:
        greville = list(nurbs.GrevilleParameters())
    except Exception:
        greville = None
    for i in range(count):
        try:
            loc = nurbs.Points[i].Location
        except Exception:
            try:
                loc = nurbs.Points[i]
            except Exception:
                continue
        t = greville[i] if greville and i < len(greville) else None
        if t is not None:
            try:
                pt = nurbs.PointAt(t)
                tan = nurbs.TangentAt(t)
            except Exception:
                pt, tan = loc, None
            delta = _value_at_t(samples, deltas, t)
        else:
            pt, tan = loc, None
            xy = _to_xy(plane, loc)
            best_i = 0
            best_d = 1e300
            for si, sample in enumerate(samples):
                d = _vdist(xy, sample["point"])
                if d < best_d:
                    best_d = d
                    best_i = si
            delta = deltas[best_i] if best_i < len(deltas) else 0.0
        if delta <= 1e-6:
            continue
        xy = _to_xy(plane, pt)
        if tan is not None:
            tan2 = _vunit((tan * plane.XAxis, tan * plane.YAxis))
            inward = _vleft(tan2)
        else:
            inward = (0.0, 0.0)
        if inward[0] == 0.0 and inward[1] == 0.0:
            continue
        probe = _vadd(xy, _vmul(inward, 0.2))
        if not _point_in_ring(probe, ring):
            inward = _vmul(inward, -1.0)
        outward = _vmul(inward, -1.0)
        cv_xy = _vadd(_to_xy(plane, loc), _vmul(outward, delta))
        new_pt = _from_xy(plane, cv_xy)
        try:
            nurbs.Points.SetPoint(i, new_pt)
        except Exception:
            try:
                nurbs.Points[i] = new_pt
            except Exception:
                return None
    if not nurbs.IsValid:
        return None
    return _force_closed_curve(nurbs)


def _grow_is_uniform(raw):
    pos = [v for v in raw if v > 0.04]
    if len(pos) < 3:
        return False
    return max(pos) <= min(pos) * 1.35 + 0.15


_CLOSED_PREP = {}


def _clear_closed_prep():
    _CLOSED_PREP.clear()


def _ensure_min_width(curve, min_width, corners):
    """Widen under-min-width stretches into one fair few-node closed curve."""
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
        corners_xy = _curve_corners_xy(curve, plane)
        prep = (plane, ring, samples, spacing, corners_xy)
        _CLOSED_PREP[key] = prep
    plane, ring, samples, spacing, corners_xy = prep
    _assign_widths(samples, ring, min_width)
    raw = _raw_deltas(samples, min_width)
    if not any(d > 1e-4 for d in raw):
        return [curve.DuplicateCurve()], 0, []
    deltas, pinches = _distance_scaled_deltas(samples, raw, min_width)
    if pinches == 0:
        return [curve.DuplicateCurve()], 0, []
    feature_ring = corners_xy if len(corners_xy) >= 3 else ring
    features = _original_features(feature_ring, samples, deltas, min_width)
    radius = min_width * 0.5
    tips = _fillet_tips(features)
    has_end = any(f.get("kind") == "end" for f in features)

    outline = None
    try:
        orig_nurbs = curve.ToNurbsCurve()
        orig_cvs = orig_nurbs.Points.Count if orig_nurbs is not None else 0
    except Exception:
        orig_cvs = 0

    # Keep the artist's few CVs whenever we can. Rebuilding that NURBS as an
    # interpolant is what produced the loop / spike / open vertical in Rhino.
    if orig_cvs and orig_cvs <= 64:
        moved = _move_nurbs_cvs(curve, plane, samples, deltas, ring)
        if _nurbs_usable(moved):
            if has_end:
                capped = _nurbs_same_tip_caps(moved, 0.0, radius, plane, ring, features)
                outline = capped if _nurbs_usable(capped) else moved
            else:
                outline = moved

    if outline is None and (_mostly_thin(raw) or _grow_is_uniform(raw)):
        grown = _grow_opening(curve, _median_positive(raw), "Round")
        if _nurbs_usable(grown):
            outline = grown

    if outline is None:
        moved_ring, _ = _apply_min_width(ring, samples, spacing, min_width)
        n_ctrl = _fair_ctrl_count(len(tips) if tips else 3)
        outline = _nurbs_periodic_from_ring(plane, moved_ring, n_ctrl)
        if not _nurbs_usable(outline):
            outline = None

    if outline is None or not getattr(outline, "IsValid", False):
        # Last resort: keep the original NURBS. Never emit an interpolant.
        return [curve.DuplicateCurve()], pinches, []
    outline = _force_closed_curve(outline)
    if outline is None or not outline.IsValid:
        return [curve.DuplicateCurve()], pinches, []
    return [outline], pinches, []


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
        84.5 < max(xs) - min(xs) < 88.0,
        "thin slot ends should be a half-min-width radius (len={0})".format(max(xs) - min(xs)),
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

    # Square ends: one semicircle of r = min_width/2 around the original tip, no hook.
    thin = [(0.0, 0.0), (80.0, 0.0), (80.0, 3.0), (0.0, 3.0)]
    moved, pinches = ensure_min_width_ring(thin, 6.0)
    right_cap = [p for p in moved if p[0] > 79.5]
    left_cap = [p for p in moved if p[0] < 0.5]
    assert_true(pinches > 0, "square-end slot should pinch")
    assert_true(len(right_cap) >= 3, "right end should be a sampled cap")
    assert_true(
        all(_vdist(p, (80.0, 1.5)) < 3.45 for p in right_cap),
        "right end must stay on a half-min-width semicircle, not a hook",
    )
    assert_true(
        all(_vdist(p, (0.0, 1.5)) < 3.45 for p in left_cap),
        "left end must stay on a half-min-width semicircle, not a hook",
    )
    assert_true(max(p[0] for p in moved) < 83.6, "right cap must not overshoot r=3")
    assert_true(min(p[1] for p in right_cap) > -3.4, "right cap must not curl into a hook")

    # Thin-slot outer corner: one circular radius, no S-wave inflection.
    elbow = [
        (0.0, 0.0), (40.0, 0.0), (40.0, 40.0), (37.0, 40.0),
        (37.0, 3.0), (0.0, 3.0),
    ]
    moved, pinches = ensure_min_width_ring(elbow, 6.0)
    corner = [p for p in moved if p[0] >= 40.0 and p[1] <= 0.0]
    signs = []
    for i in range(1, len(corner) - 1):
        turn = _turn_at(corner[i - 1], corner[i], corner[i + 1])
        if abs(turn) > 0.03:
            signs.append(1 if turn > 0.0 else -1)
    flips = 0
    for i in range(1, len(signs)):
        if signs[i] != signs[i - 1]:
            flips += 1
    assert_true(pinches > 0, "elbow corner check should still pinch")
    assert_true(len(corner) >= 4, "elbow corner should keep a sampled radius")
    assert_true(flips == 0, "elbow corner should be one radius, not an S-wave (flips={0})".format(flips))

    # Concave-sided triangle: three sharp corners must get the same simple
    # offset join, not a split lollipop on some tips and a point on others.
    centroid = (40.0, 70.0 / 3.0)
    bowed = []
    for a, b in [((0.0, 0.0), (80.0, 0.0)), ((80.0, 0.0), (40.0, 70.0)), ((40.0, 70.0), (0.0, 0.0))]:
        mid = ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5)
        inward = _vunit(_vsub(centroid, mid))
        ctrl = _vadd(mid, _vmul(inward, 8.0))
        for k in range(24):
            t = k / 24.0
            bowed.append((
                (1.0 - t) * (1.0 - t) * a[0] + 2.0 * (1.0 - t) * t * ctrl[0] + t * t * b[0],
                (1.0 - t) * (1.0 - t) * a[1] + 2.0 * (1.0 - t) * t * ctrl[1] + t * t * b[1],
            ))
    moved, pinches = ensure_min_width_ring(bowed, 6.0)
    assert_true(pinches > 0, "bowed triangle should pinch at the corners")
    assert_true(not _polyline_self_intersects(moved), "triangle corners must not split")
    extents = []
    for vertex in ((0.0, 0.0), (80.0, 0.0), (40.0, 70.0)):
        outward = _vunit(_vsub(vertex, centroid))
        extent = 0.0
        for p in moved:
            if _vdist(p, vertex) > 10.0:
                continue
            proj = _vdot(_vsub(p, vertex), outward)
            if proj > extent:
                extent = proj
        extents.append(extent)
        assert_true(extent > 0.35, "triangle corner must not stay a sharp point (ext={0})".format(extent))
    assert_true(
        max(extents) - min(extents) < 2.0,
        "all three triangle corners should close the same way (ext={0})".format(extents),
    )
    # Wide sides stay on the original — a grafted r=3 bulb / uniform offset would
    # shove the mid-side out by about 3 mm.
    for a, b in [((0.0, 0.0), (80.0, 0.0)), ((80.0, 0.0), (40.0, 70.0)), ((40.0, 70.0), (0.0, 0.0))]:
        mid = ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5)
        inward = _vunit(_vsub(centroid, mid))
        ctrl = _vadd(mid, _vmul(inward, 8.0))
        orig_mid = (
            0.25 * a[0] + 0.5 * ctrl[0] + 0.25 * b[0],
            0.25 * a[1] + 0.5 * ctrl[1] + 0.25 * b[1],
        )
        nearest = min(_vdist(p, orig_mid) for p in moved)
        assert_true(
            nearest < 1.6,
            "triangle side should stay on the original, not take a 3 mm bulb (d={0})".format(nearest),
        )
    kappas = []
    for i in range(len(moved)):
        turn = _turn_at(moved[(i - 1) % len(moved)], moved[i], moved[(i + 1) % len(moved)])
        ds = 0.5 * (
            _vdist(moved[(i - 1) % len(moved)], moved[i])
            + _vdist(moved[i], moved[(i + 1) % len(moved)])
        )
        if ds > 1e-6:
            kappas.append(turn / ds)
    # Grafted r = 3 caps are a flat κ ≈ 1/3 pulse. A fair tip eases through.
    circle_k = 1.0 / 3.0
    near_circle = [
        abs(abs(k) - circle_k) < 0.06
        for k in kappas
    ]
    circle_run = 0
    best_run = 0
    for flag in near_circle + near_circle[:2]:
        if flag:
            circle_run += 1
            if circle_run > best_run:
                best_run = circle_run
        else:
            circle_run = 0
    assert_true(
        best_run < 8,
        "triangle tips must not be grafted min-width circles (circle-run={0})".format(best_run),
    )
    # Rhino must bake a B-spline through these CVs, not an interpolant.
    # Interpolating a handful of tip samples is what looped and spiked.
    bake_cvs = _arc_length_resample(moved, 24, True)
    baked = _sample_periodic_bspline(bake_cvs, 0.35)
    assert_true(len(baked) >= 12, "periodic B-spline bake should stay a curve")
    assert_true(not _polyline_self_intersects(baked), "B-spline bake must not loop")
    assert_true(
        min(p[0] for p in baked) > min(p[0] for p in moved) - 4.0
        and max(p[0] for p in baked) < max(p[0] for p in moved) + 4.0,
        "B-spline bake must not spike past the fair outline",
    )

    # Same G2 fair blend on every tip — tangent to both walls, curvature eases in.
    fillet_r = 3.0
    walls = ((-8.0, 0.0), (0.0, 0.0), (4.0, 6.928))
    chain = _constant_radius_fillet(walls[0], walls[1], walls[2], fillet_r, bowed)
    assert_true(len(chain) >= 6, "fair blend should be a sampled quintic")
    t_in = _vunit(_vsub(walls[1], walls[0]))
    t_out = _vunit(_vsub(walls[2], walls[1]))
    t_chain0 = _vunit(_vsub(chain[1], chain[0]))
    t_chain1 = _vunit(_vsub(chain[-1], chain[-2]))
    assert_true(_vdot(t_in, t_chain0) > 0.97, "blend must be G1 with the incoming wall")
    assert_true(_vdot(t_out, t_chain1) > 0.97, "blend must be G1 with the outgoing wall")
    kappas = []
    for i in range(1, len(chain) - 1):
        turn = _turn_at(chain[i - 1], chain[i], chain[i + 1])
        ds = 0.5 * (_vdist(chain[i - 1], chain[i]) + _vdist(chain[i], chain[i + 1]))
        if ds > 1e-6:
            kappas.append(turn / ds)
    assert_true(len(kappas) >= 4, "fair blend should have a curvature comb")
    peak = max(kappas, key=lambda k: abs(k))
    assert_true(abs(kappas[0]) < abs(peak) * 0.55, "curvature must ease in, not jump to a circle")
    assert_true(abs(kappas[-1]) < abs(peak) * 0.55, "curvature must ease out, not jump to a circle")
    flips = 0
    signs = [1 if k > 0 else -1 for k in kappas if abs(k) > 0.02]
    for i in range(1, len(signs)):
        if signs[i] != signs[i - 1]:
            flips += 1
    assert_true(flips == 0, "fair blend must not S-wave (flips={0})".format(flips))

    # 2.5-turn tapered koru: thin inner coil opens, wide belly stays, no loop.
    koru_spine = []
    for i in range(121):
        t = i / 120.0
        a = math.pi * 0.20 + t * math.pi * 2.0 * 2.55
        r = 82.0 * (0.20 ** t) + 5.5
        koru_spine.append((130.0 + math.cos(a) * r, 120.0 + math.sin(a) * r))
    koru_left = []
    koru_right = []
    for i, p in enumerate(koru_spine):
        t = i / 120.0
        half = (12.0 * (1.0 - t) ** 1.4 + 0.28) * 0.5
        nxt = koru_spine[i + 1] if i < 120 else koru_spine[i]
        prv = koru_spine[i - 1] if i else koru_spine[i]
        tan = _vunit(_vsub(nxt, p) if i < 120 else _vsub(p, prv))
        nrm = _vleft(tan)
        koru_left.append(_vadd(p, _vmul(nrm, half)))
        koru_right.append(_vadd(p, _vmul(nrm, -half)))
    koru = koru_left + list(reversed(koru_right))
    moved, pinches = ensure_min_width_ring(koru, 6.0)
    belly = max(koru, key=lambda q: q[0])
    tip = koru_spine[-1]
    belly_drift = min(_vdist(belly, q) for q in moved)
    tip_span = 0.0
    for i, a in enumerate(moved):
        if _vdist(a, tip) > 10.0:
            continue
        for b in moved[i + 1 :]:
            if _vdist(b, tip) > 10.0:
                continue
            tip_span = max(tip_span, _vdist(a, b))
    assert_true(pinches > 0, "complex koru tip should pinch")
    assert_true(not _polyline_self_intersects(moved), "complex koru must not loop")
    assert_true(belly_drift < 0.8, "wide koru belly should stay, drift={0}".format(belly_drift))
    assert_true(tip_span > 3.2, "inner koru coil should open, span={0}".format(tip_span))
    # Neck just behind the tip vs the cap: a lollipop is a wide U on a thin stem.
    spine_dir = _vunit(_vsub(tip, koru_spine[-8]))
    cap_w = 0.0
    neck_w = 0.0
    for p in moved:
        rel = _vsub(p, tip)
        along = _vdot(rel, spine_dir)
        across = abs(_vcross(spine_dir, rel))
        if 0.0 <= along < 3.6:
            cap_w = max(cap_w, across * 2.0)
        if -8.0 < along < -4.0:
            neck_w = max(neck_w, across * 2.0)
    if neck_w > 0.8 and cap_w > 0.8:
        assert_true(
            cap_w < neck_w * 1.35 + 1.0,
            "koru tip must not be a grafted bulb (cap={0} neck={1})".format(cap_w, neck_w),
        )

    print("PlasmaKerf math tests passed")


if __name__ == "__main__":
    if HAS_RHINO:
        RunCommand()
    else:
        _self_test()
