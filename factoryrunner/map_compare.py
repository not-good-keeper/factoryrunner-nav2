"""Compare a self-built occupancy map with a reference one. No ROS imports.

Cells follow the trinary map convention: occupancy p = 1 - pixel/255 (negate off), occupied
above occupied_thresh, free below free_thresh, unknown in between. Pixel 205 is what Nav2's map
saver writes for unknown cells, and it is treated as unknown even though p = 0.196 would fall
under a 0.25 free threshold.
"""

import numpy as np

OCCUPIED, FREE, UNKNOWN = 1, 0, -1
UNKNOWN_PIXEL = 205


def classify(pixels, occupied_thresh, free_thresh):
    """Turn a greyscale map image (0-255, row 0 at the top) into OCCUPIED/FREE/UNKNOWN cells."""
    pixels = np.asarray(pixels)
    p = 1.0 - pixels.astype(float) / 255.0
    cells = np.full(p.shape, UNKNOWN, dtype=int)
    cells[p > occupied_thresh] = OCCUPIED
    cells[(p < free_thresh) & (pixels != UNKNOWN_PIXEL)] = FREE
    return cells


def resample(cells, origin, resolution, target_shape, target_origin, target_resolution):
    """Nearest-cell lookup of `cells` at each cell centre of a target grid (row 0 at the top).

    Cells of the target that fall outside the source map come back UNKNOWN.
    """
    rows, cols = target_shape
    out = np.full(target_shape, UNKNOWN, dtype=int)
    # World coordinates of every target cell centre.
    xs = target_origin[0] + (np.arange(cols) + 0.5) * target_resolution
    ys = target_origin[1] + (rows - 1 - np.arange(rows) + 0.5) * target_resolution
    src_cols = np.floor((xs - origin[0]) / resolution).astype(int)
    src_rows = cells.shape[0] - 1 - np.floor((ys - origin[1]) / resolution).astype(int)
    col_ok = (src_cols >= 0) & (src_cols < cells.shape[1])
    row_ok = (src_rows >= 0) & (src_rows < cells.shape[0])
    out[np.ix_(row_ok, col_ok)] = cells[np.ix_(src_rows[row_ok], src_cols[col_ok])]
    return out


def dilate(mask, radius):
    grown = mask.copy()
    for dr in range(-radius, radius + 1):
        for dc in range(-radius, radius + 1):
            grown |= np.roll(np.roll(mask, dr, axis=0), dc, axis=1)
    return grown


def agreement(built, reference, tolerance_cells=2):
    """Compare two maps on the same grid, judging only cells the built map has observed.

    Returns occupied recall (reference obstacles the built map also marks, within the
    tolerance), occupied precision (built obstacles that exist in the reference), free
    agreement (built free cells the reference also calls free) and the observed cell count.
    """
    observed = built != UNKNOWN
    ref_occ = (reference == OCCUPIED) & observed
    built_occ = built == OCCUPIED
    recall_hits = ref_occ & dilate(built_occ, tolerance_cells)
    precision_hits = built_occ & dilate(reference == OCCUPIED, tolerance_cells)
    built_free = built == FREE
    return {
        'observed_cells': int(observed.sum()),
        'occupied_recall': float(recall_hits.sum() / ref_occ.sum()) if ref_occ.any() else None,
        'occupied_precision': (float(precision_hits.sum() / built_occ.sum())
                               if built_occ.any() else None),
        'free_agreement': (float((built_free & (reference == FREE)).sum() / built_free.sum())
                           if built_free.any() else None),
    }
