"""Shared settings. Example mapping values are from the historical experiment."""
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CONFIG = json.loads((ROOT / "config/settings.json").read_text(encoding="utf-8"))
OFFLINE = False
RUNTIME = ROOT / "runtime"


def configure(path=None, offline=False):
    global OFFLINE
    if path:
        overrides = json.loads(Path(path).read_text(encoding="utf-8"))
        # Nested sections are replaced as a whole; the shipped file is a template.
        CONFIG.update(overrides)
    OFFLINE = offline
    if not isinstance(CONFIG["mapping_reviewed"], bool):
        raise ValueError("mapping_reviewed must be a JSON boolean")
    for key in ("max_depth_mm", "max_frame_age_s"):
        if not math.isfinite(CONFIG[key]) or CONFIG[key] <= 0:
            raise ValueError(f"{key} must be finite and positive")
    bounds = CONFIG["paint_bounds_px"]
    if (len(bounds) != 4 or not all(isinstance(v, int) for v in bounds)
            or not 0 <= bounds[0] < bounds[2] < 640
            or not 0 <= bounds[1] < bounds[3] < 480):
        raise ValueError("paint_bounds_px must fit the 640 x 480 image")
    RUNTIME.mkdir(exist_ok=True)
