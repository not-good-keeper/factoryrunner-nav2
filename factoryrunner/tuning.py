"""Search space and objective for the Nav2 parameter study (scripts/tune.py). No ROS imports.

The objective weights were fixed before the study ran and are stated in the
README; changing them after seeing results would make the study meaningless.
"""

from dataclasses import dataclass
from itertools import combinations


@dataclass(frozen=True)
class TunedParam:
    node: str
    name: str
    low: float
    high: float
    default: float  # the value in config/nav2_params.yaml


SEARCH_SPACE = [
    TunedParam('controller_server', 'FollowPath.vx_max', 0.30, 0.50, 0.5),
    TunedParam('controller_server', 'FollowPath.PathFollowCritic.cost_weight', 2.0, 10.0, 5.0),
    TunedParam('controller_server', 'FollowPath.CostCritic.cost_weight', 2.0, 8.0, 3.81),
    TunedParam('local_costmap/local_costmap', 'inflation_layer.cost_scaling_factor', 2.0, 6.0, 3.0),
]

FAILED_LEG_PENALTY = 60.0
RECOVERY_PENALTY = 10.0
CLEARANCE_PENALTY = 20.0
CLEARANCE_TARGET_M = 0.25
STOP_EVENT_PENALTY = 5.0


def defaults():
    return {p.name: p.default for p in SEARCH_SPACE}


def by_node(values):
    """Group {param name: value} into {node: [(name, value)]} for the parameter clients."""
    grouped = {}
    for p in SEARCH_SPACE:
        if p.name in values:
            grouped.setdefault(p.node, []).append((p.name, float(values[p.name])))
    return grouped


def objective(legs, planned_legs):
    """Score one lap (lower is better) from its trial rows.

    Each leg is a dict with duration_s, result, recoveries, min_clearance_m and stop_events.
    Legs that never ran (the lap was cut short) count as failed.
    """
    lap_time = sum(leg['duration_s'] for leg in legs)
    failed = sum(leg['result'] != 'SUCCEEDED' for leg in legs) + (planned_legs - len(legs))
    recoveries = sum(leg['recoveries'] for leg in legs)
    clearances = [leg['min_clearance_m'] for leg in legs if leg['min_clearance_m'] is not None]
    shortfall = max(0.0, CLEARANCE_TARGET_M - min(clearances)) if clearances else 0.0
    stops = sum(leg['stop_events'] for leg in legs)
    return (lap_time
            + FAILED_LEG_PENALTY * failed
            + RECOVERY_PENALTY * recoveries
            + CLEARANCE_PENALTY * shortfall / CLEARANCE_TARGET_M
            + STOP_EVENT_PENALTY * stops)


def mann_whitney_exact(a, b):
    """Exact two-sided Mann-Whitney U test; returns (U for a over b, p-value).

    Enumerates every way of splitting the pooled values into groups of len(a) and len(b), so
    it is exact for the small samples here (10 + 10 is 184,756 splits). Ties count half.
    """
    if not a or not b:
        raise ValueError('both samples need at least one value')

    def u_stat(x, y):
        return sum((xi > yi) + 0.5 * (xi == yi) for xi in x for yi in y)

    pooled = list(a) + list(b)
    centre = len(a) * len(b) / 2
    observed = u_stat(a, b)
    extreme = total = 0
    for idx in combinations(range(len(pooled)), len(a)):
        chosen = set(idx)
        u = u_stat([pooled[i] for i in idx],
                   [pooled[i] for i in range(len(pooled)) if i not in chosen])
        total += 1
        if abs(u - centre) >= abs(observed - centre) - 1e-9:
            extreme += 1
    return observed, extreme / total
