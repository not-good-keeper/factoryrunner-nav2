"""Merge opt-in overlay files onto nav2_params.yaml. No ROS imports, so pytest covers it.

sim.launch.py uses this for its depth_costmap and velocity_stop arguments: with both off it
passes nav2_params.yaml through untouched, so default runs see exactly the measured config.
"""

from pathlib import Path
import tempfile

import yaml


def merge(base, overlay):
    """Return base updated with overlay: mappings merge recursively, anything else replaces."""
    merged = dict(base)
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = merge(merged[key], value)
        else:
            merged[key] = value
    return merged


def write_merged(base_path, overlay_paths, out_dir=None):
    """Write base_path with every overlay applied in order and return the new file's path."""
    params = yaml.safe_load(Path(base_path).read_text())
    for path in overlay_paths:
        overlay = yaml.safe_load(Path(path).read_text()) or {}
        if not isinstance(overlay, dict):
            raise ValueError(f'{path} must be a mapping at the top level')
        params = merge(params, overlay)
    out = tempfile.NamedTemporaryFile('w', suffix='_nav2_params.yaml', dir=out_dir,
                                      delete=False)
    with out:
        yaml.safe_dump(params, out, sort_keys=False)
    return out.name
