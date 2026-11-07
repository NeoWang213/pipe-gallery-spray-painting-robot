"""Hardware-independent paths and the original experimental coordinate mapping.

Coordinates: pixels for 2D paths; millimetres for projected points.
The mapping is an empirical transform, not a calibrated eye-in-hand model.
"""
import math


def _finite(*values):
    if not all(math.isfinite(v) for v in values):
        raise ValueError("Coordinates must be finite")


def rectangle_path(x0, y0, x1, y1, step=20):
    _finite(x0, y0, x1, y1, step)
    if step <= 0 or x0 == x1 or y0 == y1:
        raise ValueError("Select a non-empty rectangle and a positive spacing")
    left, right = sorted((x0, x1))
    top, bottom = sorted((y0, y1))
    rows = max(1, math.ceil((bottom - top) / step))
    points = []
    for i in range(rows + 1):
        y = top + (bottom - top) * i / rows
        xs = (left, right) if i % 2 == 0 else (right, left)
        points.extend((x, y) for x in xs)
    return points


def circle_path(cx, cy, radius, step=15):
    """Historical concentric-circle mode; it is not a pipe surface model."""
    _finite(cx, cy, radius, step)
    if radius <= 0 or step <= 0:
        raise ValueError("Radius and spacing must be positive")
    rings = max(1, math.ceil(radius / step))
    points = []
    for i in range(1, rings + 1):
        r = radius * i / rings
        count = max(8, math.ceil(2 * math.pi * r / step))
        points.extend((cx + r * math.cos(2 * math.pi * j / count),
                       cy + r * math.sin(2 * math.pi * j / count))
                      for j in range(count + 1))
    return points


def map_2d(x, y, cfg):
    _finite(x, y)
    ax = cfg["origin_x_mm"] + (x - cfg["center_x_px"]) * cfg["mm_per_px"]
    ay = (cfg["center_y_px"] - y) * cfg["mm_per_px"]
    if not (cfg["x_limits_mm"][0] <= ax <= cfg["x_limits_mm"][1]
            and cfg["y_limits_mm"][0] <= ay <= cfg["y_limits_mm"][1]):
        raise ValueError("Point exceeds the experimental 2D mapping range")
    return round(ax, 1), round(ay, 1)


def project_path(points, depth_at, intrinsics, mapping, max_depth=2000):
    """Reject the entire path on a missing depth, before issuing any movement."""
    if not points:
        raise ValueError("Empty trajectory")
    fx, fy = intrinsics["fx"], intrinsics["fy"]
    _finite(fx, fy, intrinsics["cx"], intrinsics["cy"], max_depth)
    if fx <= 0 or fy <= 0 or max_depth <= 0:
        raise ValueError("Invalid camera intrinsics or depth range")
    result, depths = [], []
    ox, oy, oz = mapping["offset_mm"]
    for x, y in points:
        _finite(x, y)
        depth = depth_at(int(round(x)), int(round(y)))
        if depth is None or not math.isfinite(depth) or not 0 < depth <= max_depth:
            raise ValueError(f"Invalid depth at ({x:.0f}, {y:.0f}); reacquire the image")
        xc = (x - intrinsics["cx"]) * depth / fx
        yc = (y - intrinsics["cy"]) * depth / fy
        p = (mapping["origin_x_mm"] + xc * mapping["scale_x"] + ox,
             yc * mapping["scale_y"] + oy, depth + oz)
        _finite(*p)
        result.append(p)
        depths.append(depth)
    return result, sum(depths) / len(depths)
