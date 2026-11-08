#!/usr/bin/env python3
"""Summarize objective-4 ROS bag trials with consistent geometric measurements.

Time to clear is measured FROM FIRST DETECTED CART MOTION until every cart
has crossed a fixed exit line and recovered to the original lane centerline.
This is a reproducible proxy, not an algorithm's internal 'conflict cleared' log.

Footprint overlap is a 2D rectangle-overlap proxy, not a contact-sensor report.
"""
from __future__ import annotations

import csv
import math
import sqlite3
from itertools import combinations
from pathlib import Path

from rclpy.serialization import deserialize_message
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist

BASE = Path.home() / 'thesis_cart' / 'objective4_results'
CART_LENGTH = 0.38
CART_WIDTH = 0.27
CARTS_FOR_TEST = {'T1': 2, 'T2': 2, 'T3': 3,
                  'T4': 3, 'T5': 2, 'T6': 2}
PRIORITY_CART = {'T2': 'cart_2', 'T4': 'cart_3', 'T6': 'cart_2'}
EXIT_LINE = 1.5  # m beyond the intersection center, for all test methods
CENTERLINE_TOLERANCE = 0.20  # m after passing/return-to-path


def yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def geom(p):
    theta = yaw(p.orientation)
    forward = (math.cos(theta), math.sin(theta))
    side = (-forward[1], forward[0])
    return p.position.x, p.position.y, forward, side


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1]


def rectangles_overlap(pa, pb):
    """SAT intersection for 0.38m x 0.27m cart footprint rectangles."""
    ax, ay, af, aside = geom(pa)
    bx, by, bf, bside = geom(pb)
    delta = (bx - ax, by - ay)
    for axis in (af, aside, bf, bside):
        extent_a = CART_LENGTH / 2 * abs(dot(af, axis)) + \
                   CART_WIDTH / 2 * abs(dot(aside, axis))
        extent_b = CART_LENGTH / 2 * abs(dot(bf, axis)) + \
                   CART_WIDTH / 2 * abs(dot(bside, axis))
        if abs(dot(delta, axis)) >= extent_a + extent_b - 1e-6:
            return False
    return True


def records_for_topic(db_file, topic):
    with sqlite3.connect(f'file:{db_file}?mode=ro', uri=True) as conn:
        row = conn.execute(
            'SELECT id FROM topics WHERE name=?', (topic,)
        ).fetchone()
        if row is None:
            return
        query = ('SELECT timestamp, data FROM messages '
                 'WHERE topic_id=? ORDER BY timestamp')
        for stamp, data in conn.execute(query, (row[0],)):
            yield stamp / 1e9, data


def crossed_exit(test, name, p):
    x, y = p.position.x, p.position.y
    if name == 'cart_1':
        return x >= EXIT_LINE
    if name == 'cart_2':
        return x <= -EXIT_LINE if test in ('T5', 'T6') else y <= -EXIT_LINE
    if name == 'cart_3':
        return x <= -EXIT_LINE
    return False


def aligned_with_lane(test, name, p):
    x, y = p.position.x, p.position.y
    if name == 'cart_2' and test not in ('T5', 'T6'):
        return abs(x) <= CENTERLINE_TOLERANCE
    return abs(y) <= CENTERLINE_TOLERANCE


def stopped_seconds(samples, begin, end):
    """Integration of near-zero forward commands over [begin, end]."""
    if begin is None or end is None or not samples:
        return None
    samples.sort()
    total = 0.0
    for i, (at, cmd) in enumerate(samples):
        next_at = samples[i + 1][0] if i + 1 < len(samples) else end
        lo, hi = max(begin, at), min(end, next_at)
        if hi <= lo:
            continue
        # Include commanded stationary turns as waiting/stopped time.
        if abs(cmd.linear.x) < 0.05 and abs(cmd.linear.y) < 0.05:
            total += hi - lo
    return total


def analyze(run):
    dbs = sorted((run / 'bag').glob('*.db3'))
    if not dbs:
        return None
    bits = run.name.split('_')
    test = bits[0]
    if test not in CARTS_FOR_TEST:
        return None
    method = 'no_comm' if '_no_comm_' in run.name else \
             'fcfs' if '_fcfs_' in run.name else 'proposed'
    trial = run.name.rsplit('trial', 1)[-1]
    names = [f'cart_{i}' for i in range(1, CARTS_FOR_TEST[test] + 1)]

    frames = []
    cmds = {name: [] for name in names}
    for db in dbs:
        for at, raw in records_for_topic(db, '/gazebo/model_states'):
            message = deserialize_message(raw, ModelStates)
            poses = {name: pose for name, pose in zip(message.name, message.pose)
                     if name in names}
            if len(poses) == len(names):
                frames.append((at, poses))
        for name in names:
            for at, raw in records_for_topic(db, f'/{name}/cmd_vel'):
                cmds[name].append((at, deserialize_message(raw, Twist)))

    if not frames:
        return None
    frames.sort(key=lambda pair: pair[0])
    first_xys = {name: (p.position.x, p.position.y) for name, p in frames[0][1].items()}
    first_motion = None
    cleared_time = None
    minimum = float('inf')
    closest_pair = ''
    overlap = False
    first_exit = {}

    for at, poses in frames:
        if first_motion is None:
            for name, p in poses.items():
                x0, y0 = first_xys[name]
                if math.hypot(p.position.x - x0, p.position.y - y0) >= 0.08:
                    first_motion = at
                    break

        for a, b in combinations(names, 2):
            pa, pb = poses[a], poses[b]
            distance = math.hypot(pa.position.x - pb.position.x,
                                  pa.position.y - pb.position.y)
            if distance < minimum:
                minimum = distance
                closest_pair = f'{a}-{b}'
            if distance < math.hypot(CART_LENGTH, CART_WIDTH) and \
                    rectangles_overlap(pa, pb):
                overlap = True

        if first_motion is not None and at >= first_motion:
            for name in names:
                if name not in first_exit and crossed_exit(test, name, poses[name]):
                    first_exit[name] = at
            if cleared_time is None and all(
                crossed_exit(test, name, poses[name]) and
                aligned_with_lane(test, name, poses[name])
                for name in names
            ):
                cleared_time = at

    duration = None if cleared_time is None or first_motion is None else \
        max(0.0, cleared_time - first_motion)
    waits = [stopped_seconds(cmds[name], first_motion, cleared_time)
             for name in names]
    avg_wait = sum(waits) / len(waits) if all(w is not None for w in waits) else None
    if len(first_exit) == len(names):
        first_to_exit = min(first_exit, key=first_exit.get)
    else:
        first_to_exit = ''
    priority_observed = ''
    if test in PRIORITY_CART:
        priority_observed = (
            'yes' if first_to_exit == PRIORITY_CART[test]
            else 'no' if first_to_exit else 'not_cleared'
        )

    return {
        'test': test,
        'method': method,
        'trial': trial,
        'minimum_center_distance_m': round(minimum, 4),
        'closest_pair': closest_pair,
        'rectangular_footprint_overlap_proxy': 'yes' if overlap else 'no',
        'first_to_exit': first_to_exit or 'not_all_exited',
        'priority_cart_first_observed': priority_observed or 'N/A',
        'time_to_clear_from_first_motion_s': round(duration, 2) if duration is not None else 'not_cleared',
        'mean_commanded_stop_per_cart_s': round(avg_wait, 2) if avg_wait is not None else 'N/A',
        'run': run.name,
    }


def main():
    BASE.mkdir(parents=True, exist_ok=True)
    runs = sorted(p for p in BASE.glob('T*_trial*') if p.is_dir())
    results = []
    for run in runs:
        print(f'Analyzing {run.name}...', flush=True)
        result = analyze(run)
        if result is not None:
            results.append(result)

    if not results:
        print('No readable trial bags found.')
        return
    csv_path = BASE / 'comparison_metrics.csv'
    with csv_path.open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(results[0]))
        writer.writeheader()
        writer.writerows(results)

    print(f'\nSaved {len(results)} trials to {csv_path}')

    # Pool only the matched priority-designation conditions T2/T4/T6.
    # Different N across methods remains visible in the output.
    summary = []
    for method in ('no_comm', 'fcfs', 'proposed'):
        group = [r for r in results if r['method'] == method and
                 r['test'] in ('T2', 'T4', 'T6')]
        if not group:
            continue
        n = len(group)
        overlap_n = sum(r['rectangular_footprint_overlap_proxy'] == 'yes'
                        for r in group)
        cleared = [r for r in group if isinstance(r['time_to_clear_from_first_motion_s'], float)]
        waited = [r for r in cleared if isinstance(r['mean_commanded_stop_per_cart_s'], float)]
        prioritized = sum(r['priority_cart_first_observed'] == 'yes' for r in group)
        summary.append({
            'method': method,
            'trials_N': n,
            'footprint_overlap_trials_n_over_N': f'{overlap_n}/{n}',
            'priority_cart_first_observed_n_over_N': f'{prioritized}/{n}',
            'completed_trials_n_over_N': f'{len(cleared)}/{n}',
            'mean_time_to_clear_s_completed_only': (
                round(sum(r['time_to_clear_from_first_motion_s'] for r in cleared) / len(cleared), 2)
                if cleared else 'N/A'),
            'mean_commanded_stop_per_cart_s_completed_only': (
                round(sum(r['mean_commanded_stop_per_cart_s'] for r in waited) / len(waited), 2)
                if waited else 'N/A'),
            'mean_minimum_separation_m': round(
                sum(r['minimum_center_distance_m'] for r in group) / n, 4),
        })
    if summary:
        path = BASE / 'comparison_summary_T2_T4_T6.csv'
        with path.open('w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=list(summary[0]))
            writer.writeheader()
            writer.writerows(summary)
        print(f'Saved comparison summary to {path}')
    print('Important: footprint overlap = geometric proxy, not Gazebo contact sensor.')
    print('Time-to-clear = first motion until all carts beyond 1.5m exit lines and aligned.')
    print('Stopped time = near-zero commanded translation; not a direct human wait measurement.')


if __name__ == '__main__':
    main()
