#!/usr/bin/env python3
"""Build the keepout and speed filter masks on top of Nav2's depot map.

The masks must share resolution and origin with depot.yaml, so they are painted
onto a blank copy of depot.pgm rather than drawn by hand. Re-run after editing ZONES.

usage: python3 scripts/make_masks.py [--map /opt/ros/jazzy/share/nav2_bringup/maps/depot.yaml]
"""

import argparse
from pathlib import Path

from PIL import Image
import yaml

# Rectangles in the map frame: (x_min, y_min, x_max, y_max), metres.
FORKLIFT_LANE = (2.0, -2.5, 2.8, 2.5)
CNC_CELL_ZONE = (10.0, 0.3, 14.0, 2.3)
CNC_SPEED_PERCENT = 40

# Speed mask is read in "scale" mode: grey level g maps to
# value = ((1 - g/255) - free_thresh) / (occupied_thresh - free_thresh) * 100,
# and the speed filter turns value v into a limit of base + multiplier * v = 100 - v.
SPEED_FREE_THRESH = 0.05
SPEED_OCCUPIED_THRESH = 0.95

REPO = Path(__file__).resolve().parents[1]
DEFAULT_MAP = Path('/opt/ros/jazzy/share/nav2_bringup/maps/depot.yaml')


def grey_for_limit(percent):
    value = 100 - percent
    occupancy = SPEED_FREE_THRESH + value / 100 * (SPEED_OCCUPIED_THRESH - SPEED_FREE_THRESH)
    return round(255 * (1 - occupancy))


def paint(image, rect, origin, resolution, level):
    x_min, y_min, x_max, y_max = rect
    width, height = image.size
    # image row 0 is the top edge of the map, i.e. the largest y
    col0 = int((x_min - origin[0]) / resolution)
    col1 = int((x_max - origin[0]) / resolution)
    row0 = height - 1 - int((y_max - origin[1]) / resolution)
    row1 = height - 1 - int((y_min - origin[1]) / resolution)
    if col0 < 0 or row0 < 0 or col1 >= width or row1 >= height:
        raise SystemExit(f'zone {rect} falls outside the {width}x{height} map')
    image.paste(level, (col0, row0, col1 + 1, row1 + 1))


def write_mask(name, image, base_meta, mode, free_thresh, occupied_thresh):
    out_dir = REPO / 'maps'
    out_dir.mkdir(exist_ok=True)
    image.save(out_dir / f'{name}.pgm')
    meta = {
        'image': f'{name}.pgm',
        'mode': mode,
        'resolution': base_meta['resolution'],
        'origin': base_meta['origin'],
        'negate': 0,
        'occupied_thresh': occupied_thresh,
        'free_thresh': free_thresh,
    }
    (out_dir / f'{name}.yaml').write_text(yaml.safe_dump(meta, sort_keys=False))
    print(f'wrote maps/{name}.pgm and maps/{name}.yaml')


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--map', type=Path, default=DEFAULT_MAP)
    args = parser.parse_args()

    base_meta = yaml.safe_load(args.map.read_text())
    size = Image.open(args.map.parent / base_meta['image']).size
    origin = base_meta['origin']
    resolution = base_meta['resolution']

    keepout = Image.new('L', size, 255)
    paint(keepout, FORKLIFT_LANE, origin, resolution, 0)
    write_mask('keepout_mask', keepout, base_meta, 'trinary', 0.25, 0.65)

    speed = Image.new('L', size, 255)
    paint(speed, CNC_CELL_ZONE, origin, resolution, grey_for_limit(CNC_SPEED_PERCENT))
    write_mask('speed_mask', speed, base_meta, 'scale', SPEED_FREE_THRESH, SPEED_OCCUPIED_THRESH)


if __name__ == '__main__':
    main()
