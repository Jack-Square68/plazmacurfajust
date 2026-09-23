"""
PlasmaKerf — offset curves for plasma kerf and minimum slot width.

Click a curve (or pre-select one), then adjust MinWidth / Kerf in the
command line. A live black preview is the finished cut; orange is the
torch centerline. Enter bakes the result onto layers.

Rhino 7 / 8
-----------
Closed openings are sampled once to a polyline (capped) for width only.
The baked outline is the original NURBS, offset as spans, with the same
MinWidth/2 G1 arc on every tip — not a 1200-point interpolant. Slot ends
get a MinWidth/2 semicircle around the original tip. Pointed corners of a
wide opening stay as drawn. Rhino is not asked to CurveCurve / GetLength /
Contains on every sample, so a koru no longer locks the UI.

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


def _constant_radius_fillet(a, b, c, radius, ring):
    """G1 fillet of exactly `radius` — same on every tip, tangent to both walls."""
    if radius < 0.04:
        return [b]
    t1 = _vunit(_vsub(b, a))
    t2 = _vunit(_vsub(c, b))
    if (t1[0] == 0.0 and t1[1] == 0.0) or (t2[0] == 0.0 and t2[1] == 0.0):
        return [b]
    turn = math.atan2(_vcross(t1, t2), _vdot(t1, t2))
    if abs(turn) < 0.08:
        return [b]
    if abs(turn) > 2.4 or _vdot(t1, t2) < -0.90:
        return _end_cap_chain(b, a, c, radius, ring)
    tan_h = math.tan(abs(turn) * 0.5)
    if tan_h < 1e-8:
        return [b]
    trim = radius * tan_h
    p0 = (b[0] - t1[0] * trim, b[1] - t1[1] * trim)
    p1 = (b[0] + t2[0] * trim, b[1] + t2[1] * trim)
    left = (-t1[1], t1[0])
    inward = left if turn > 0.0 else (-left[0], -left[1])
    # Centre sits on the inside of the offset corner. Do not flip from the
    # original ring: that centre often still lies in the old slot.
    center = (p0[0] + inward[0] * radius, p0[1] + inward[1] * radius)
    return [p0] + _arc_points(center, p0, p1, radius, ring) + [p1]


def _cap_all_tips_same(moved, tips, radius, ring):
    """Replace every tip with the same tangent radius so they match and stay G1."""
    if len(moved) < 6 or not tips or radius < 0.04:
        return moved, []
    protected = []
    walk = max(radius * 2.2, 3.0)
    for tip in tips:
        if tip is None:
            continue
        i, _ = _nearest_index(moved, tip)
        p_l, i_l = _walk_along(moved, i, -1, walk)
        p_r, i_r = _walk_along(moved, i, 1, walk)
        p_ll, _ = _walk_along(moved, i_l, -1, max(radius, 1.0))
        p_rr, _ = _walk_along(moved, i_r, 1, max(radius, 1.0))
        corner = _line_intersect_unbounded(p_ll, p_l, p_r, p_rr)
        if corner is None:
            corner = moved[i]
        chain = _constant_radius_fillet(p_ll, corner, p_rr, radius, ring)
        if len(chain) < 2:
            continue
        nxt = _replace_span(moved, i_l, i_r, chain)
        if len(nxt) >= 3:
            moved = nxt
            protected.extend(chain)
    cleaned = _clean_ring(moved, 0.02) if len(moved) >= 3 else moved
    return cleaned, protected


def _apply_original_features(moved, feature_ring, interior_ring, samples, grow, min_width):
    """Same min-width-radius cap on every tip, tangent to the offset walls."""
    protected = []
    if len(moved) < 4 or min_width <= 0:
        return moved, protected
    radius = min_width * 0.5
    features = _original_features(feature_ring, samples, grow, min_width)
    tips = []
    for feat in features:
        if feat["kind"] == "end":
            vertex = feat.get("center")
            chain = _end_cap_chain(feat["center"], feat["p0"], feat["p1"], radius, interior_ring)
            if len(chain) >= 2:
                nxt = _splice_feature(moved, chain, vertex, radius)
                if nxt is not moved and len(nxt) >= 3:
                    moved = nxt
                    protected.extend(chain)
        else:
            tips.append(feat.get("b") or feat.get("center"))
    if tips:
        moved, extra = _cap_all_tips_same(moved, tips, radius, interior_ring)
        protected.extend(extra)
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


def _round_short_ends(points, ring, min_width, protected=None):
    """Cap slot / taper ends with a semicircle of radius min_width/2."""
    n = len(points)
    if n < 8 or min_width <= 0:
        return points
    radius = min_width * 0.5
    used = [False] * n
    out = []
    i = 0
    while i < n:
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
    if len(cleaned) >= 3:
        cleaned, protected = _apply_original_features(
            cleaned, ring, ring, samples, deltas, min_width
        )
        cleaned = _round_short_ends(cleaned, ring, min_width, protected)
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


def _fillet_arc_between(a, b, radius):
    """True Arc of `radius` tangent to two offset walls. Never an interpolant."""
    if a is None or b is None or radius < _tol():
        return None
    pairs = [
        (a.PointAtEnd, b.PointAtStart),
        (a.PointAtEnd, b.PointAtEnd),
        (a.PointAtStart, b.PointAtStart),
        (a.PointAtStart, b.PointAtEnd),
    ]
    pa, pb = min(pairs, key=lambda p: p[0].DistanceTo(p[1]))
    try:
        result = rg.Curve.CreateFilletCurves(
            a, pa, b, pb, radius, False, False, True, _tol(), 0.1
        )
    except Exception:
        result = None
    for crv in _as_list(result):
        if crv is not None and crv.IsValid and _span_length(crv) > _tol():
            return crv
    try:
        ok_a, ta = a.ClosestPoint(pa)
        ok_b, tb = b.ClosestPoint(pb)
        if ok_a and ok_b:
            arc = rg.Curve.CreateFillet(a, b, radius, ta, tb)
            if arc is not None:
                try:
                    crv = rg.ArcCurve(arc)
                except Exception:
                    crv = arc
                if crv is not None and getattr(crv, "IsValid", True):
                    return crv
    except Exception:
        pass
    return None


def _arc_from_chain(plane, chain, radius, ring):
    """Build one ArcCurve through a 2D G1 chain (p0 + arc + p1)."""
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
    pa = _to_xy(plane, a.PointAtEnd)
    pb = _to_xy(plane, b.PointAtStart)
    try:
        ta = a.TangentAt(a.Domain.Max)
        tb = b.TangentAt(b.Domain.Min)
        a_prev = _to_xy(plane, a.PointAt(a.Domain.Max) - ta * max(radius, 1.0))
        b_next = _to_xy(plane, b.PointAt(b.Domain.Min) + tb * max(radius, 1.0))
    except Exception:
        a_prev = pa
        b_next = pb
    corner = _line_intersect_unbounded(a_prev, pa, pb, b_next)
    if corner is None:
        corner = ((pa[0] + pb[0]) * 0.5, (pa[1] + pb[1]) * 0.5)
    chain = _constant_radius_fillet(a_prev, corner, b_next, radius, ring)
    return _arc_from_chain(plane, chain, radius, ring)


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
    """Offset the original spans and close every tip with the same G1 radius.

    Output is a PolyCurve of the original-quality walls plus true ArcCurves —
    never a dense interpolant.
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
            cap = _fillet_arc_between(a, b, radius)
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
    """Widen under-min-width stretches; keep the original NURBS + same-radius tips."""
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
    distance = _median_positive(raw)

    outline = None
    if _mostly_thin(raw) or _grow_is_uniform(raw):
        outline = _nurbs_same_tip_caps(curve, distance, radius, plane, ring, features)
        if outline is None:
            grown = _grow_opening(curve, distance, "Sharp")
            if grown is not None:
                outline = _nurbs_same_tip_caps(grown, 0.0, radius, plane, ring, features)
                if outline is None:
                    outline = grown
    else:
        moved = _move_nurbs_cvs(curve, plane, samples, deltas, ring)
        if moved is not None:
            outline = _nurbs_same_tip_caps(moved, 0.0, radius, plane, ring, features)
            if outline is None:
                outline = moved

    if outline is None or not getattr(outline, "IsValid", False):
        # Last resort: keep the original NURBS. Never emit a dense interpolant.
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
        "all three triangle corners should close with the same min-diameter circle (ext={0})".format(extents),
    )

    # Same G1 radius on every tip — the chain must land tangent to both walls.
    fillet_r = 3.0
    walls = ((-8.0, 0.0), (0.0, 0.0), (4.0, 6.928))
    chain = _constant_radius_fillet(walls[0], walls[1], walls[2], fillet_r, bowed)
    assert_true(len(chain) >= 4, "constant-radius fillet should be a sampled arc")
    t_in = _vunit(_vsub(walls[1], walls[0]))
    t_out = _vunit(_vsub(walls[2], walls[1]))
    t_chain0 = _vunit(_vsub(chain[1], chain[0]))
    t_chain1 = _vunit(_vsub(chain[-1], chain[-2]))
    assert_true(_vdot(t_in, t_chain0) > 0.97, "fillet must be G1 with the incoming wall")
    assert_true(_vdot(t_out, t_chain1) > 0.97, "fillet must be G1 with the outgoing wall")

    print("PlasmaKerf math tests passed")


if __name__ == "__main__":
    if HAS_RHINO:
        RunCommand()
    else:
        _self_test()
