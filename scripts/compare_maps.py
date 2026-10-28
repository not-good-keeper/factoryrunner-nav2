#!/usr/bin/env python3
"""Compare a map built with SLAM against Nav2's shipped depot map.

python3 scripts/compare_maps.py maps/depot_slam.yaml

Prints the agreement figures, writes them to results/slam/comparison.txt and draws
results/slam_map_overlay.png. Only cells the SLAM map has actually observed are judged.
"""

import argparse
from pathlib import Path
import sys

REPO = Path(__file__).resolve().parents[1]


def load(yaml_path):
    from PIL import Image
    import numpy as np
    import yaml

    from factoryrunner.map_compare import classify

    meta = yaml.safe_load(Path(yaml_path).read_text())
    pixels = np.asarray(Image.open(Path(yaml_path).parent / meta['image']).convert('L'))
    return classify(pixels, meta['occupied_thresh'], meta['free_thresh']), meta


def main(argv=None):
    parser = argparse.ArgumentParser(description='Compare a SLAM map with the shipped one.')
    parser.add_argument('built', type=Path)
    parser.add_argument('--reference', type=Path, default=Path(
        '/opt/ros/jazzy/share/nav2_bringup/maps/depot.yaml'))
    parser.add_argument('--tolerance-cells', type=int, default=2)
    args = parser.parse_args(argv)

    sys.path.insert(0, str(REPO))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np

    from factoryrunner.map_compare import agreement, OCCUPIED, resample, UNKNOWN

    reference, ref_meta = load(args.reference)
    built_native, built_meta = load(args.built)
    built = resample(built_native, built_meta['origin'][:2], built_meta['resolution'],
                     reference.shape, ref_meta['origin'][:2], ref_meta['resolution'])
    result = agreement(built, reference, args.tolerance_cells)

    tol_m = args.tolerance_cells * ref_meta['resolution']
    lines = [
        f'built map: {args.built.name}, reference: {args.reference.name}',
        f'origin offset: x {built_meta["origin"][0] - ref_meta["origin"][0]:+.3f} m, '
        f'y {built_meta["origin"][1] - ref_meta["origin"][1]:+.3f} m',
        f'cells observed by the built map: {result["observed_cells"]} '
        f'of {reference.size} ({result["observed_cells"] / reference.size:.0%})',
        f'reference obstacles found by SLAM (within {tol_m:.2f} m): '
        f'{result["occupied_recall"]:.1%}',
        f'SLAM obstacles present in the reference (within {tol_m:.2f} m): '
        f'{result["occupied_precision"]:.1%}',
        f'SLAM free cells that the reference also calls free: {result["free_agreement"]:.1%}',
    ]
    out_dir = REPO / 'results' / 'slam'
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / 'comparison.txt').write_text('\n'.join(lines) + '\n')
    print('\n'.join(lines))

    from factoryrunner.map_compare import dilate
    ref_occ = reference == OCCUPIED
    built_occ = built == OCCUPIED
    near_ref = dilate(ref_occ, args.tolerance_cells)
    image = np.ones(reference.shape + (3,))
    image[built == UNKNOWN] = (0.93, 0.93, 0.92)
    image[ref_occ] = (0.55, 0.55, 0.53)
    image[built_occ & near_ref] = matplotlib.colors.to_rgb('#2a78d6')
    image[built_occ & ~near_ref] = matplotlib.colors.to_rgb('#eb6834')
    x0, y0 = ref_meta['origin'][:2]
    res = ref_meta['resolution']
    extent = (x0, x0 + reference.shape[1] * res, y0, y0 + reference.shape[0] * res)

    fig, ax = plt.subplots(figsize=(9, 5.4), dpi=150)
    ax.imshow(image, extent=extent, interpolation='nearest')
    ax.set_title('SLAM map from one station loop over the shipped depot map', loc='left',
                 fontsize=11)
    ax.set_xlabel('map x (m)', fontsize=8, color='#52514e')
    ax.set_ylabel('map y (m)', fontsize=8, color='#52514e')
    ax.tick_params(labelsize=7, colors='#52514e')
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    handles = [matplotlib.patches.Patch(color=(0.55, 0.55, 0.53), label='Shipped map obstacle'),
               matplotlib.patches.Patch(color='#2a78d6',
                                        label='SLAM obstacle, also in shipped map'),
               matplotlib.patches.Patch(color='#eb6834', label='SLAM obstacle, not in shipped map'),
               matplotlib.patches.Patch(color=(0.93, 0.93, 0.92), label='Not seen by SLAM')]
    ax.legend(handles=handles, frameon=False, fontsize=7, loc='upper left',
              bbox_to_anchor=(0, -0.1), ncol=2)
    fig.tight_layout()
    out = REPO / 'results' / 'slam_map_overlay.png'
    fig.savefig(out)
    print(f'wrote {out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
