#!/usr/bin/env python3
"""
tlog_to_vr_json.py - Convert Mission Planner .tlog files to Unity VR drone path planner JSON format.

Usage:
    python3 tools/tlog_to_vr_json.py <input.tlog> [-o output.json] [options]
"""

import argparse
import bisect
import json
import math
import os
import sys

COPTER_MODES = {
    0: 'STABILIZE', 2: 'ALT_HOLD', 3: 'AUTO', 4: 'GUIDED',
    5: 'LOITER', 6: 'RTL', 9: 'LAND', 16: 'POSHOLD'
}


def parse_args():
    parser = argparse.ArgumentParser(
        description='Convert Mission Planner .tlog to Unity VR drone path planner JSON.'
    )
    parser.add_argument('input_tlog', help='Path to Mission Planner .tlog file')
    parser.add_argument('-o', '--output', help='Output JSON file (default: <input_stem>_telemetry.json)')
    parser.add_argument('--downsample', type=float, default=0.1,
                        help='One sample per N seconds (default: 0.1 = 10Hz)')
    parser.add_argument('--start-time', type=float, default=None,
                        help='Only include after this time offset in seconds')
    parser.add_argument('--end-time', type=float, default=None,
                        help='Only include before this time offset in seconds')
    parser.add_argument('--include-attitude', action='store_true',
                        help='Add yaw (degrees) to each sample')
    parser.add_argument('--participant-id', default='unknown',
                        help='participantId for metadata (default: unknown)')
    parser.add_argument('--environment', default='unknown',
                        help='environment for metadata (default: unknown)')
    parser.add_argument('-v', '--verbose', action='store_true',
                        help='Show progress and message counts')
    return parser.parse_args()


def parse_tlog(input_path, verbose):
    try:
        from pymavlink import mavutil
    except ImportError:
        print('Error: pymavlink is not installed. Run: pip install pymavlink', file=sys.stderr)
        sys.exit(1)

    try:
        mlog = mavutil.mavlink_connection(input_path)
    except Exception as e:
        print(f'Error opening {input_path}: {e}', file=sys.stderr)
        sys.exit(1)

    pos_msgs = []    # list of (timestamp, x, y, z)
    att_msgs = []    # list of (timestamp, yaw_deg)
    mode_msgs = []   # list of (timestamp, mode_name)

    counts = {'LOCAL_POSITION_NED': 0, 'ATTITUDE': 0, 'HEARTBEAT': 0}
    last_mode = None

    while True:
        try:
            msg = mlog.recv_match(
                type=['LOCAL_POSITION_NED', 'ATTITUDE', 'HEARTBEAT'],
                blocking=False
            )
        except Exception:
            break
        if msg is None:
            break

        mtype = msg.get_type()
        t = getattr(msg, '_timestamp', 0)

        if mtype == 'LOCAL_POSITION_NED':
            counts['LOCAL_POSITION_NED'] += 1
            pos_msgs.append((t, msg.x, msg.y, msg.z))

        elif mtype == 'ATTITUDE':
            counts['ATTITUDE'] += 1
            yaw_deg = math.degrees(msg.yaw)
            att_msgs.append((t, yaw_deg))

        elif mtype == 'HEARTBEAT':
            counts['HEARTBEAT'] += 1
            custom_mode = getattr(msg, 'custom_mode', 0)
            mode_name = COPTER_MODES.get(custom_mode, str(custom_mode))
            if mode_name != last_mode:
                mode_msgs.append((t, mode_name))
                last_mode = mode_name

    if verbose:
        for k, v in counts.items():
            print(f'  {k}: {v} messages')

    return pos_msgs, att_msgs, mode_msgs


def find_nearest_yaw(att_msgs, att_timestamps, target_t):
    if not att_msgs:
        return 0.0
    idx = bisect.bisect_left(att_timestamps, target_t)
    if idx == 0:
        return att_msgs[0][1]
    if idx >= len(att_msgs):
        return att_msgs[-1][1]
    before = att_msgs[idx - 1]
    after = att_msgs[idx]
    if abs(target_t - before[0]) <= abs(target_t - after[0]):
        return before[1]
    return after[1]


def build_output(pos_msgs, att_msgs, mode_msgs, args):
    if not pos_msgs:
        print('Error: no LOCAL_POSITION_NED messages found in tlog.', file=sys.stderr)
        sys.exit(1)

    if not att_msgs and args.include_attitude:
        print('Warning: no ATTITUDE messages found; yaw will be 0.', file=sys.stderr)

    t0 = pos_msgs[0][0]
    att_timestamps = [a[0] for a in att_msgs]

    samples = []
    last_kept_time = None

    for (t, ned_x, ned_y, ned_z) in pos_msgs:
        rel_t = t - t0

        if args.start_time is not None and rel_t < args.start_time:
            continue
        if args.end_time is not None and rel_t > args.end_time:
            continue
        if last_kept_time is not None and (rel_t - last_kept_time) < args.downsample:
            continue

        last_kept_time = rel_t

        unity_x = ned_y    # NED East  → Unity X (East)
        unity_y = -ned_z   # NED Down  → Unity Y (Up)
        unity_z = ned_x    # NED North → Unity Z (North)

        sample = {
            'time': round(rel_t, 4),
            'position': {
                'x': round(unity_x, 4),
                'y': round(unity_y, 4),
                'z': round(unity_z, 4),
            }
        }

        if args.include_attitude:
            yaw_deg = find_nearest_yaw(att_msgs, att_timestamps, t)
            sample['yaw'] = round(yaw_deg, 2)

        samples.append(sample)

    if not samples:
        print('Error: no samples remain after filtering.', file=sys.stderr)
        sys.exit(1)

    duration = samples[-1]['time'] - samples[0]['time']

    # Path metrics
    total_3d = 0.0
    total_horiz = 0.0
    total_vert = 0.0
    for i in range(1, len(samples)):
        p0 = samples[i - 1]['position']
        p1 = samples[i]['position']
        dx = p1['x'] - p0['x']
        dy = p1['y'] - p0['y']
        dz = p1['z'] - p0['z']
        total_3d += math.sqrt(dx * dx + dy * dy + dz * dz)
        total_horiz += math.sqrt(dx * dx + dz * dz)
        total_vert += abs(dy)

    start_pos = samples[0]['position']
    end_pos = samples[-1]['position']

    # Flight modes — convert to relative time
    flight_modes = []
    for (t, mode_name) in mode_msgs:
        rel_t = t - t0
        if args.start_time is not None and rel_t < args.start_time:
            continue
        if args.end_time is not None and rel_t > args.end_time:
            continue
        flight_modes.append({'time': round(rel_t, 4), 'mode': mode_name})

    source_file = os.path.basename(args.input_tlog)

    output = {
        'metadata': {
            'participantId': args.participant_id,
            'taskVariant': 'flight_log',
            'environment': args.environment,
            'sourceFile': source_file,
            'durationSeconds': round(duration, 3),
            'coordinateSpace': 'modelLocal',
            'sampleCount': len(samples),
        },
        'pathMetrics': {
            'startPosition': start_pos,
            'endPosition': end_pos,
            'totalLength3D': round(total_3d, 4),
            'totalLengthHorizontal2D': round(total_horiz, 4),
            'totalVerticalChange': round(total_vert, 4),
        },
        'flightModes': flight_modes,
        'dronePlannedMovement': {
            'totalDistanceMoved': round(total_3d, 4),
            'samples': samples,
        },
    }

    return output, duration, total_3d, len(flight_modes)


def main():
    args = parse_args()

    if not os.path.isfile(args.input_tlog):
        print(f'Error: file not found: {args.input_tlog}', file=sys.stderr)
        sys.exit(1)

    if args.output:
        output_path = args.output
    else:
        stem = os.path.splitext(args.input_tlog)[0]
        output_path = stem + '_telemetry.json'

    if args.verbose:
        print(f'Parsing {args.input_tlog} ...')

    pos_msgs, att_msgs, mode_msgs = parse_tlog(args.input_tlog, args.verbose)

    if args.verbose:
        print('Building output ...')

    output, duration, total_dist, n_modes = build_output(pos_msgs, att_msgs, mode_msgs, args)

    with open(output_path, 'w') as f:
        json.dump(output, f, indent=2)

    n_samples = output['metadata']['sampleCount']
    print(f'Written: {output_path}')
    print(f'  Samples:       {n_samples}')
    print(f'  Duration:      {duration:.1f}s')
    print(f'  Total distance:{total_dist:.2f}m')
    print(f'  Mode changes:  {n_modes}')


if __name__ == '__main__':
    main()
