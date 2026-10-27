#!/usr/bin/env python3
"""Tune four Nav2 parameters with Optuna, one trial_runner lap per trial.

Start a fresh headless sim first (the robot must be at the dock), then, from the venv:
    source ~/fr_venv/bin/activate
    python3 scripts/tune.py study --trials 25
    python3 scripts/tune.py validate --laps 5
    python3 scripts/tune.py report

study     TPE sampler (seed 42), SQLite storage at results/tuning.db. The first trial is the
          current config, so the study has a baseline. Parameters change at runtime; Nav2's
          planner and controller are untouched apart from these values.
validate  Alternates default and best parameters lap by lap, with the pallet at a spot the
          study never saw, and writes results/validation_<run>.csv.
report    Writes results/tuning_history.png and results/tuning_study.csv (tuning.db stays out
          of git) and prints the validation table.
"""

import argparse
import csv
import dataclasses
from datetime import datetime
from pathlib import Path
import statistics
import sys

REPO = Path(__file__).resolve().parents[1]
RESULTS = REPO / 'results'
STUDY_NAME = 'factoryrunner_d13'
# Validation spot: 6.7 m along the RAW_MATERIAL -> CNC_CELL plan; the study uses 4.7 m.
HOLDOUT = (9.18, 0.73)


def set_params(navigator, values):
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.parameter_client import AsyncParameterClient

    from factoryrunner.tuning import by_node

    for node, pairs in by_node(values).items():
        client = AsyncParameterClient(navigator, node)
        if not client.wait_for_services(timeout_sec=10.0):
            raise RuntimeError(f'parameter services of {node} not available')
        future = client.set_parameters([Parameter(name, value=value) for name, value in pairs])
        rclpy.spin_until_future_complete(navigator, future, timeout_sec=10.0)
        if not future.done():
            raise RuntimeError(f'setting parameters on {node} timed out')
        rejected = [(name, r.reason) for (name, _), r in zip(pairs, future.result().results)
                    if not r.successful]
        if rejected:
            raise RuntimeError(f'{node} rejected {rejected}')


def start_navigator():
    import rclpy
    from rclpy.parameter import Parameter

    from factoryrunner.mission_controller import to_pose_stamped, warn_unless_cyclone
    from factoryrunner.mission_config import load_mission
    from nav2_simple_commander.robot_navigator import BasicNavigator

    rclpy.init()
    navigator = BasicNavigator('factoryrunner_tuner')
    navigator.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    warn_unless_cyclone(navigator)
    mission = load_mission(REPO / 'config' / 'mission.yaml')
    navigator.setInitialPose(to_pose_stamped(navigator, mission.initial_pose, mission.frame_id))
    navigator.waitUntilNav2Active()
    return navigator, mission


def one_lap(navigator, mission, csv_path, run_id, leg_timeout_s):
    from factoryrunner.trial_runner import run_trials

    return run_trials(navigator, dataclasses.replace(mission, laps=1, obstacle_laps=[1]),
                      csv_path, run_id, leg_timeout_s)


def cmd_study(args):
    import optuna

    from factoryrunner.tuning import defaults, objective, SEARCH_SPACE

    navigator, mission = start_navigator()
    stamp = datetime.now().strftime('%Y%m%d-%H%M%S')
    csv_path = RESULTS / f'tuning_trials_{stamp}.csv'
    study = optuna.create_study(study_name=STUDY_NAME, storage=args.storage, direction='minimize',
                                sampler=optuna.samplers.TPESampler(seed=42),
                                load_if_exists=True)
    if not study.trials:
        study.enqueue_trial(defaults())

    def trial_fn(trial):
        values = {p.name: trial.suggest_float(p.name, p.low, p.high) for p in SEARCH_SPACE}
        set_params(navigator, values)
        legs = one_lap(navigator, mission, csv_path, f'{stamp}-t{trial.number}',
                       args.leg_timeout)
        score = objective(legs, len(mission.stations))
        trial.set_user_attr('lap_time_s', sum(leg['duration_s'] for leg in legs))
        trial.set_user_attr('failed_legs', sum(leg['result'] != 'SUCCEEDED' for leg in legs))
        if any(leg['result'] == 'CANCELED' for leg in legs):
            # A timed-out leg may mean Nav2 is stuck; later trials would only measure that.
            print('leg timed out; stopping the study')
            study.stop()
        return score

    try:
        study.optimize(trial_fn, n_trials=args.trials)
    finally:
        set_params(navigator, defaults())
    print(f'best value {study.best_value:.1f} with {study.best_params}')
    print(f'trial rows in {csv_path}')
    return 0


def cmd_validate(args):
    import optuna

    from factoryrunner.mission_config import ObstacleConfig
    from factoryrunner.tuning import defaults

    best = optuna.load_study(study_name=STUDY_NAME, storage=args.storage).best_params
    navigator, mission = start_navigator()
    holdout = ObstacleConfig(leg_station=mission.obstacle.leg_station, x=args.holdout[0],
                             y=args.holdout[1], spawn_distance=mission.obstacle.spawn_distance)
    mission = dataclasses.replace(mission, obstacle=holdout)
    stamp = datetime.now().strftime('%Y%m%d-%H%M%S')
    csv_path = RESULTS / f'validation_{stamp}.csv'
    print(f'best params {best}; pallet at {args.holdout}')
    try:
        for lap in range(1, args.laps + 1):
            # Alternate so slow drift in the sim cannot favour one configuration.
            for name, values in (('default', defaults()), ('tuned', best)):
                set_params(navigator, values)
                one_lap(navigator, mission, csv_path, f'{stamp}-{name}-lap{lap}',
                        args.leg_timeout)
    finally:
        set_params(navigator, defaults())
    print(f'validation rows in {csv_path}')
    return 0


def summarise(validation_csvs):
    """Return {config: [per-lap dicts]} pooled from one or more validation CSVs."""
    laps = {}
    for path in validation_csvs:
        with open(path, newline='') as f:
            rows = list(csv.DictReader(f))
        for row in rows:
            config = row['run_id'].split('-')[-2]
            lap = laps.setdefault((config, row['run_id']), {
                'lap_time': 0.0, 'failed': 0, 'recoveries': 0, 'clearance': [], 'legs': 0})
            lap['lap_time'] += float(row['duration_s'])
            lap['failed'] += row['result'] != 'SUCCEEDED'
            lap['recoveries'] += int(row['recoveries'])
            lap['legs'] += 1
            if row['min_clearance_m']:
                lap['clearance'].append(float(row['min_clearance_m']))
    by_config = {}
    for (config, _), lap in laps.items():
        by_config.setdefault(config, []).append(lap)
    return by_config


def cmd_report(args):
    import matplotlib
    import matplotlib.pyplot as plt
    import optuna

    from factoryrunner.tuning import mann_whitney_exact, SEARCH_SPACE

    matplotlib.use('Agg')
    study = optuna.load_study(study_name=STUDY_NAME, storage=args.storage)
    done = [t for t in study.trials if t.value is not None]
    best, running = float('inf'), []
    for t in done:
        best = min(best, t.value)
        running.append(best)
    fig, ax = plt.subplots(figsize=(7.5, 3.4), dpi=150)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    ax.grid(axis='y', color='#e4e3df', linewidth=0.8)
    ax.set_axisbelow(True)
    ax.scatter([t.number for t in done], [t.value for t in done], s=22, color='#2a78d6',
               zorder=3, label='Trial (one lap with the pallet)')
    ax.step([t.number for t in done], running, where='post', color='#eb6834', linewidth=2,
            label='Best so far')
    ax.annotate('trial 0 = current config', (done[0].number, done[0].value),
                textcoords='offset points', xytext=(6, 8), fontsize=7, color='#52514e')
    ax.set_xlabel('Trial', fontsize=9, color='#52514e')
    ax.set_ylabel('Objective (lower is better)', fontsize=9, color='#52514e')
    ax.set_title('Optuna TPE study over four Nav2 parameters', loc='left', fontsize=11)
    ax.legend(frameon=False, fontsize=8, loc='upper left', bbox_to_anchor=(0, -0.2), ncol=2)
    fig.tight_layout()
    out = RESULTS / 'tuning_history.png'
    fig.savefig(out)
    print(f'wrote {out} ({len(done)} trials, best {study.best_value:.1f})')
    study_csv = RESULTS / 'tuning_study.csv'
    names = [p.name for p in SEARCH_SPACE]
    with open(study_csv, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['trial', 'state', 'objective', 'lap_time_s', 'failed_legs'] + names)
        for t in study.trials:
            writer.writerow([t.number, t.state.name,
                             '' if t.value is None else f'{t.value:.2f}',
                             f'{t.user_attrs.get("lap_time_s", float("nan")):.2f}',
                             t.user_attrs.get('failed_legs', '')]
                            + [f'{t.params[n]:.4f}' if n in t.params else '' for n in names])
    print(f'wrote {study_csv}')
    print(f'best params: {study.best_params}')

    validation = sorted(RESULTS.glob('validation_*.csv'))
    if not validation:
        print('no validation run yet')
        return 0
    # Every validation run so far used the same best parameters and held-out spot, so pool them.
    print('\nvalidation: ' + ', '.join(v.name for v in validation))
    print('| Config | Laps | Lap time median (s) | Lap time range (s) | Failed legs '
          '| Recoveries | Min clearance (m) |')
    print('|---|---|---|---|---|---|---|')
    pooled = summarise(validation)
    for config, laps in sorted(pooled.items()):
        times = [lap['lap_time'] for lap in laps]
        clear = [c for lap in laps for c in lap['clearance']]
        clearance = f'{min(clear):.2f}' if clear else 'n/a'
        print(f'| {config} | {len(laps)} | {statistics.median(times):.1f} '
              f'| {min(times):.1f}-{max(times):.1f} | {sum(lap["failed"] for lap in laps)} '
              f'| {sum(lap["recoveries"] for lap in laps)} | {clearance} |')
    if {'default', 'tuned'} <= pooled.keys():
        default = [lap['lap_time'] for lap in pooled['default']]
        tuned = [lap['lap_time'] for lap in pooled['tuned']]
        u, p_value = mann_whitney_exact(default, tuned)
        print(f'\nlap times default: {", ".join(f"{t:.1f}" for t in default)}')
        print(f'lap times tuned:   {", ".join(f"{t:.1f}" for t in tuned)}')
        print(f'exact two-sided Mann-Whitney: U = {u:g} of {len(default) * len(tuned)}, '
              f'p = {p_value:.2g}')
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description='Nav2 parameter study.')
    parser.add_argument('--db', type=Path, default=RESULTS / 'tuning.db',
                        help='Optuna SQLite file (a scratch one for dry runs)')
    sub = parser.add_subparsers(dest='command', required=True)
    study = sub.add_parser('study', help='run the Optuna study')
    study.add_argument('--trials', type=int, default=25)
    validate = sub.add_parser('validate', help='default vs best on a held-out obstacle spot')
    validate.add_argument('--laps', type=int, default=5, help='laps per configuration')
    validate.add_argument('--holdout', type=float, nargs=2, default=HOLDOUT,
                          metavar=('X', 'Y'))
    for p in (study, validate):
        p.add_argument('--leg-timeout', type=float, default=120.0)
    sub.add_parser('report', help='history plot and validation table')
    args = parser.parse_args(argv)
    args.storage = f'sqlite:///{args.db.resolve()}'

    sys.path.insert(0, str(REPO))  # run from a checkout without sourcing the install
    RESULTS.mkdir(exist_ok=True)
    return {'study': cmd_study, 'validate': cmd_validate, 'report': cmd_report}[args.command](args)


if __name__ == '__main__':
    sys.exit(main())
