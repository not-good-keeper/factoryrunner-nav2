#!/usr/bin/env python3
"""Plot trial results for the README.

python3 scripts/plot_results.py --trials results/trials_<run>.csv
python3 scripts/plot_results.py --bag results/bags/<run>      # exports, then plots, the profile
python3 scripts/plot_results.py --speed-csv results/speed_profile_<run>.csv

--trials writes results/lap_durations.png. --bag reads /cmd_vel, /speed_limit and
/collision_monitor_state from a recording, saves them as results/speed_profile_<run>.csv (the
bag itself stays out of git) and plots results/speed_profile.png; --speed-csv replots that CSV.
--window START END limits the profile plot to that span, e.g. one lap's start_s and end_s
from the trials CSV (sim seconds, when the bag recorded /clock).
--route BAG (with --trials) exports the robot's AMCL track to results/route_<run>.csv and draws
every lap over the depot map, zones and stations as results/route_map.png; --route-csv redraws
it from that CSV.
--sweep DIR [DIR ...] plots one or more obstacle_sweep.sh runs, pooled, as
results/obstacle_sweep.png; add --trials to draw the no-pallet CNC_CELL legs from that run as
the reference band.
"""

import argparse
import csv
from pathlib import Path
import statistics
import sys

import matplotlib
from matplotlib.patches import Patch
import matplotlib.pyplot as plt

# Blue and orange: a pair that stays distinguishable for the common colour-vision deficiencies.
NORMAL = '#2a78d6'
OBSTACLE = '#eb6834'
INK = '#0b0b0b'
MUTED = '#52514e'
GRID = '#e4e3df'
ZONE_SHADE = '#c9c8c2'
SLOWDOWN_SHADE = '#f6c3ad'


def _style(ax):
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(MUTED)
    ax.tick_params(colors=MUTED, labelsize=9)
    ax.grid(axis='y', color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)


def load_laps(trials_csv):
    """Return [(lap, duration_s, obstacle, all_legs_succeeded)] in lap order."""
    laps = {}
    with open(trials_csv, newline='') as f:
        for row in csv.DictReader(f):
            lap = laps.setdefault(int(row['lap']), {'duration': 0.0, 'obstacle': False, 'ok': True})
            lap['duration'] += float(row['duration_s'])
            lap['obstacle'] |= row['obstacle'] == '1'
            lap['ok'] &= row['result'] == 'SUCCEEDED'
    return [(n, v['duration'], v['obstacle'], v['ok']) for n, v in sorted(laps.items())]


def plot_lap_durations(laps, out_png):
    fig, ax = plt.subplots(figsize=(7.5, 3.6), dpi=150)
    _style(ax)
    clear = [d for _, d, obstacle, ok in laps if ok and not obstacle]
    baseline = statistics.median(clear) if clear else None
    if baseline is not None:
        ax.axhline(baseline, color=MUTED, linewidth=0.8, linestyle=':')
        ax.text(laps[-1][0] + 0.45, baseline, f'median without\nobstacle {baseline:.1f} s',
                fontsize=7, color=MUTED, va='center')
    for lap, duration, obstacle, ok in laps:
        ax.bar(lap, duration, width=0.4, color=OBSTACLE if obstacle else NORMAL,
               hatch='//' if obstacle else None, edgecolor='white', linewidth=0)
        # Label only what the chart is about: the obstacle laps' cost and any failure.
        if obstacle and baseline is not None:
            ax.text(lap, duration + 1.5, f'{duration - baseline:+.1f} s', ha='center',
                    va='bottom', fontsize=8, color=INK)
        if not ok:
            ax.text(lap, duration + 7, 'leg failed', ha='center', fontsize=8, color=INK,
                    fontweight='bold')
    ax.set_xticks([lap for lap, *_ in laps])
    ax.set_xlim(laps[0][0] - 0.6, laps[-1][0] + 1.6)
    ax.set_xlabel('Lap', color=MUTED, fontsize=9)
    ax.set_ylabel('Lap time (sim s)', color=MUTED, fontsize=9)
    top = max(d for _, d, _, _ in laps)
    ax.set_ylim(0, top * 1.18)
    ax.set_title('Station loop lap time, 4 legs per lap', loc='left', color=INK, fontsize=11)
    ax.legend(handles=[Patch(color=NORMAL, label='No obstacle'),
                       Patch(facecolor=OBSTACLE, hatch='//', edgecolor='white',
                             label='Pallet dropped on CNC_CELL leg')],
              frameon=False, fontsize=8, loc='upper left', bbox_to_anchor=(0, -0.18), ncol=2)
    fig.tight_layout()
    fig.savefig(out_png)
    plt.close(fig)


def export_profile(bag_dir, out_csv):
    """Write one row per /cmd_vel message with the speed limit and monitor state in force."""
    from rclpy.serialization import deserialize_message
    import rosbag2_py
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    # Twist has no header. With /clock in the bag, stamp samples with the latest sim time;
    # otherwise fall back to the recorder's wall-clock receive time and say so in the column.
    time_col = 't_sim_s' if '/clock' in types else 't_wall_s'
    limit, action, clock, t0, rows = 0.0, 0, None, None, []
    while reader.has_next():
        topic, data, stamp_ns = reader.read_next()
        if topic not in ('/cmd_vel', '/speed_limit', '/collision_monitor_state', '/clock'):
            continue
        msg = deserialize_message(data, get_message(types[topic]))
        if topic == '/clock':
            clock = msg.clock.sec + msg.clock.nanosec / 1e9
            continue
        if time_col == 't_sim_s':
            if clock is None:
                continue
            # Absolute sim time, so it lines up with start_s/end_s in the trials CSV.
            now, t0 = clock, 0.0
        else:
            now = stamp_ns / 1e9
            t0 = now if t0 is None else t0
        if topic == '/speed_limit':
            limit = msg.speed_limit
        elif topic == '/collision_monitor_state':
            action = msg.action_type
        else:
            rows.append({time_col: f'{now - t0:.3f}', 'cmd_vel_x': f'{msg.linear.x:.4f}',
                         'speed_limit_pct': f'{limit:.1f}', 'monitor_action': action})
    with open(out_csv, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=[time_col, 'cmd_vel_x', 'speed_limit_pct',
                                               'monitor_action'])
        writer.writeheader()
        writer.writerows(rows)
    return len(rows)


def windows(times, flags):
    """Turn per-sample booleans into [(start, end)] spans where the flag holds."""
    spans, begin = [], None
    for t, on in zip(times, flags):
        if on and begin is None:
            begin = t
        elif not on and begin is not None:
            spans.append((begin, t))
            begin = None
    if begin is not None:
        spans.append((begin, times[-1]))
    return spans


def plot_speed_profile(profile_csv, out_png, window=None, title_note=''):
    with open(profile_csv, newline='') as f:
        rows = list(csv.DictReader(f))
    time_col = 't_sim_s' if 't_sim_s' in rows[0] else 't_wall_s'
    if window:
        rows = [r for r in rows if window[0] <= float(r[time_col]) <= window[1]]
        if not rows:
            raise SystemExit(f'no samples between {window[0]} and {window[1]} s')
    t = [float(r[time_col]) for r in rows]
    vx = [float(r['cmd_vel_x']) for r in rows]
    zone = windows(t, [0.0 < float(r['speed_limit_pct']) < 100.0 for r in rows])
    slow = windows(t, [int(r['monitor_action']) in (1, 2) for r in rows])

    fig, ax = plt.subplots(figsize=(9, 3.4), dpi=150)
    _style(ax)
    for a, b in zone:
        ax.axvspan(a, b, color=ZONE_SHADE, alpha=0.6, linewidth=0)
    for a, b in slow:
        ax.axvspan(a, b, color=SLOWDOWN_SHADE, alpha=0.9, linewidth=0)
    ax.plot(t, vx, color=NORMAL, linewidth=1.2)
    ax.axhline(0.5, color=MUTED, linewidth=0.8, linestyle=':')
    ax.text(t[0], 0.51, 'vx_max 0.5', fontsize=7, color=MUTED, va='bottom')
    ax.axhline(0.2, color=MUTED, linewidth=0.8, linestyle=':')
    ax.text(t[0], 0.21, '40 % zone cap 0.2', fontsize=7, color=MUTED, va='bottom')
    ax.set_xlabel('Sim time (s)' if time_col == 't_sim_s'
                  else 'Wall time since recording start (s)', color=MUTED, fontsize=9)
    ax.set_ylabel('Commanded linear speed (m/s)', color=MUTED, fontsize=9)
    ax.set_ylim(min(-0.1, min(vx) - 0.05), max(0.6, max(vx) + 0.05))
    ax.set_title(f'Commanded speed on /cmd_vel{title_note}', loc='left', color=INK, fontsize=11)
    handles = [Patch(color=ZONE_SHADE, alpha=0.6, label='CNC speed zone (40 % limit)')]
    if slow:
        handles.append(Patch(color=SLOWDOWN_SHADE, label='Collision monitor slowdown or stop'))
    ax.legend(handles=handles, frameon=False, fontsize=8, loc='upper left',
              bbox_to_anchor=(0, -0.2), ncol=2)
    fig.tight_layout()
    fig.savefig(out_png)
    plt.close(fig)


def load_sweep(sweep_dirs, leg_station='CNC_CELL'):
    """Return {drop distance: [CNC leg rows]} pooled from obstacle_sweep.sh output directories."""
    legs = {}
    for csv_path in sorted(p for d in sweep_dirs for p in Path(d).glob('d*/trials_*.csv')):
        distance = float(csv_path.parent.name[1:])
        with open(csv_path, newline='') as f:
            legs.setdefault(distance, []).extend(
                r for r in csv.DictReader(f) if r['station'] == leg_station)
    return legs


def plot_sweep(legs, out_png, reference=None):
    """Three panels against drop distance: leg time, closest return, slowdown + stop time."""
    distances = sorted(legs, reverse=True)
    panels = [
        ('CNC_CELL leg time (sim s)', lambda r: float(r['duration_s'])),
        ('Closest scan return (m)', lambda r: float(r['min_clearance_m'])),
        ('Slowdown + stop time (s)', lambda r: float(r['slowdown_s']) + float(r['stop_s'])),
    ]
    fig, axes = plt.subplots(1, 3, figsize=(10, 3.4), dpi=150)
    for ax, (label, value) in zip(axes, panels):
        _style(ax)
        if reference:
            ref = [value(r) for r in reference]
            ax.axhspan(min(ref), max(ref), color=ZONE_SHADE, alpha=0.5, linewidth=0)
        for i, d in enumerate(distances):
            ok = [value(r) for r in legs[d] if r['result'] == 'SUCCEEDED']
            bad = [value(r) for r in legs[d] if r['result'] != 'SUCCEEDED']
            ax.scatter([i] * len(ok), ok, s=26, color=OBSTACLE, zorder=3)
            ax.scatter([i] * len(bad), bad, s=40, marker='x', color=INK, linewidths=1.5, zorder=4)
            ys = ok + bad
            ax.hlines(statistics.median(ys), i - 0.2, i + 0.2, color=INK, linewidth=1.5, zorder=4)
        if label.startswith('Slowdown'):
            top = max((value(r) for d in distances for r in legs[d]), default=0.0)
            ax.set_ylim(0, max(1.0, top * 1.15))
        ax.set_xticks(range(len(distances)))
        ax.set_xticklabels([f'{d:g} m' for d in distances])
        ax.set_xlabel('Configured drop distance', color=MUTED, fontsize=8)
        ax.set_title(label, loc='left', color=INK, fontsize=9)
    handles = [plt.Line2D([], [], marker='o', linestyle='', color=OBSTACLE, label='One lap'),
               plt.Line2D([], [], marker='x', linestyle='', color=INK, label='Leg did not succeed'),
               plt.Line2D([], [], color=INK, linewidth=1.5, label='Median')]
    if reference:
        handles.append(Patch(color=ZONE_SHADE, alpha=0.5, label='Range without a pallet'))
    fig.legend(handles=handles, frameon=False, fontsize=8, loc='lower left', ncol=4)
    fig.tight_layout(rect=(0, 0.08, 1, 1))
    fig.savefig(out_png)
    plt.close(fig)


def export_route(bag_dir, out_csv):
    """Write the robot's /amcl_pose track (sim time from each message's own stamp) to CSV."""
    from rclpy.serialization import deserialize_message
    import rosbag2_py
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    msg_type = get_message('geometry_msgs/msg/PoseWithCovarianceStamped')
    rows = []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != '/amcl_pose':
            continue
        msg = deserialize_message(data, msg_type)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        rows.append((f'{stamp:.3f}', f'{msg.pose.pose.position.x:.3f}',
                     f'{msg.pose.pose.position.y:.3f}'))
    with open(out_csv, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['t_sim_s', 'x', 'y'])
        writer.writerows(rows)
    return len(rows)


def _map_layer(pgm, meta):
    from PIL import Image

    image = Image.open(pgm).convert('L')
    width, height = image.size
    x0, y0 = meta['origin'][:2]
    extent = (x0, x0 + width * meta['resolution'], y0, y0 + height * meta['resolution'])
    return image, extent


def plot_route_map(route_csv, trials_csv, out_png):
    import numpy as np
    import yaml

    from ament_index_python.packages import get_package_share_directory

    repo = Path(__file__).resolve().parents[1]
    depot_yaml = Path(get_package_share_directory('nav2_bringup')) / 'maps' / 'depot.yaml'
    depot_meta = yaml.safe_load(depot_yaml.read_text())
    depot, extent = _map_layer(depot_yaml.parent / depot_meta['image'], depot_meta)
    keepout, _ = _map_layer(repo / 'maps' / 'keepout_mask.pgm', depot_meta)
    speed, _ = _map_layer(repo / 'maps' / 'speed_mask.pgm', depot_meta)
    stations = yaml.safe_load((repo / 'config' / 'mission.yaml').read_text())['mission']

    laps = {}
    with open(trials_csv, newline='') as f:
        for row in csv.DictReader(f):
            lap = laps.setdefault(int(row['lap']), [float('inf'), 0.0, False])
            lap[0] = min(lap[0], float(row['start_s']))
            lap[1] = max(lap[1], float(row['end_s']))
            lap[2] |= row['obstacle'] == '1'
    with open(route_csv, newline='') as f:
        track = [(float(r['t_sim_s']), float(r['x']), float(r['y'])) for r in csv.DictReader(f)]

    fig, ax = plt.subplots(figsize=(8.6, 6.6), dpi=150)
    # Map: occupied cells dark, free white, unknown a light grey, as map_server reads them.
    ax.imshow(np.asarray(depot), cmap='gray', vmin=0, vmax=255, extent=extent, alpha=0.55)
    for mask, value, colour in ((keepout, 0, '#e34948'), (speed, 105, '#6250d6')):
        cells = np.asarray(mask) == value
        overlay = np.zeros(cells.shape + (4,))
        overlay[cells] = matplotlib.colors.to_rgba(colour, 0.35)
        ax.imshow(overlay, extent=extent)

    for lap, (start, end, obstacle) in sorted(laps.items()):
        xs = [x for t, x, _ in track if start <= t <= end]
        ys = [y for t, _, y in track if start <= t <= end]
        ax.plot(xs, ys, color=OBSTACLE if obstacle else NORMAL, linewidth=1.2, alpha=0.8,
                zorder=3)

    spot = stations['obstacle']
    ax.add_patch(matplotlib.patches.Rectangle((spot['x'] - 0.25, spot['y'] - 0.25), 0.5, 0.5,
                                              fill=False, edgecolor=INK, linewidth=1.2, zorder=4))
    ax.annotate('pallet drop spot', (spot['x'], spot['y'] - 0.25), textcoords='offset points',
                xytext=(0, -12), ha='center', fontsize=7, color=INK)
    for st in stations['stations']:
        ax.plot(st['x'], st['y'], marker='o', markersize=6, color=INK, zorder=5)
        ax.annotate(st['name'], (st['x'], st['y']), textcoords='offset points', xytext=(6, 6),
                    fontsize=8, color=INK, fontweight='bold')
    ax.annotate('keepout lane', (2.4, 0.0), ha='center', va='center', rotation=90, fontsize=7,
                color='#a32422')
    ax.annotate('CNC speed zone (40 %)', (11.0, 2.6), ha='center', fontsize=7, color='#3e3197')

    xs = [x for _, x, _ in track] + [st['x'] for st in stations['stations']]
    ys = [y for _, _, y in track] + [st['y'] for st in stations['stations']]
    ax.set_xlim(min(xs) - 1.5, max(xs) + 1.5)
    ax.set_ylim(min(ys) - 1.5, max(ys) + 1.5)
    ax.set_aspect('equal')
    ax.set_xlabel('map x (m)', color=MUTED, fontsize=8)
    ax.set_ylabel('map y (m)', color=MUTED, fontsize=8)
    ax.tick_params(colors=MUTED, labelsize=7)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    ax.set_title(f'{len(laps)} laps of the station loop, as localized by the robot',
                 loc='left', color=INK, fontsize=11)
    handles = [plt.Line2D([], [], color=NORMAL, linewidth=1.5, label='Lap without pallet'),
               plt.Line2D([], [], color=OBSTACLE, linewidth=1.5, label='Lap with pallet dropped'),
               Patch(color='#e34948', alpha=0.35, label='Keepout lane'),
               Patch(color='#6250d6', alpha=0.35, label='Speed zone')]
    ax.legend(handles=handles, frameon=False, fontsize=8, loc='upper left',
              bbox_to_anchor=(0, -0.1), ncol=4)
    fig.tight_layout()
    fig.savefig(out_png)
    plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description='Plot trial results for the README.')
    parser.add_argument('--trials', type=Path, help='trials_<run>.csv from trial_runner')
    parser.add_argument('--bag', type=Path, help='rosbag directory to export the profile from')
    parser.add_argument('--speed-csv', type=Path, help='an exported speed_profile_<run>.csv')
    parser.add_argument('--window', type=float, nargs=2, metavar=('START', 'END'),
                        help='only plot the profile between these times')
    parser.add_argument('--title-note', default='', help='appended to the profile title')
    parser.add_argument('--route', type=Path, help='rosbag directory with /amcl_pose')
    parser.add_argument('--route-csv', type=Path, help='an exported route_<run>.csv')
    parser.add_argument('--sweep', type=Path, nargs='+',
                        help='one or more obstacle_sweep.sh output directories')
    parser.add_argument('--out-dir', type=Path, default=Path('results'))
    args = parser.parse_args(argv)
    if not (args.trials or args.bag or args.speed_csv or args.sweep or args.route
            or args.route_csv):
        parser.error('give --trials, --bag, --speed-csv, --sweep, --route or --route-csv')
    matplotlib.use('Agg')  # files only; no window even when a display is available

    args.out_dir.mkdir(parents=True, exist_ok=True)
    if args.route or args.route_csv:
        if not args.trials:
            parser.error('--route and --route-csv need --trials to tell the laps apart')
        route_csv = args.route_csv
        if args.route:
            route_csv = args.out_dir / f'route_{args.route.name}.csv'
            print(f'wrote {route_csv} ({export_route(args.route, route_csv)} poses)')
        out = args.out_dir / 'route_map.png'
        plot_route_map(route_csv, args.trials, out)
        print(f'wrote {out}')
        return 0
    if args.sweep:
        reference = None
        if args.trials:
            with open(args.trials, newline='') as f:
                reference = [r for r in csv.DictReader(f)
                             if r['station'] == 'CNC_CELL' and r['obstacle'] == '0']
        out = args.out_dir / 'obstacle_sweep.png'
        plot_sweep(load_sweep(args.sweep), out, reference)
        print(f'wrote {out}')
        return 0
    if args.trials:
        out = args.out_dir / 'lap_durations.png'
        plot_lap_durations(load_laps(args.trials), out)
        print(f'wrote {out}')
    speed_csv = args.speed_csv
    if args.bag:
        speed_csv = args.out_dir / f'speed_profile_{args.bag.name}.csv'
        n = export_profile(args.bag, speed_csv)
        print(f'wrote {speed_csv} ({n} samples)')
    if speed_csv:
        out = args.out_dir / 'speed_profile.png'
        plot_speed_profile(speed_csv, out, args.window, args.title_note)
        print(f'wrote {out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
