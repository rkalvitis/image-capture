#!/usr/bin/env python3
"""Live diagnostic: which Femto Mega clock is trustworthy for capture time?

Opens the same streams the recorder uses (MJPG color + Y16 depth, frame-sync
on) and, for every frame, logs the three clocks the SDK exposes plus the host
wall time at receipt:

  device  = frame.get_timestamp_us()         on-sensor capture clock
  arrival = frame.get_system_timestamp_us()  host time when the frame landed
  global  = frame.get_global_timestamp_us()  SDK device->host fitted clock
  host    = time.time()                       our wall clock at receipt

Then reports, per stream, whether each clock is monotonic, how many unique
values it produced, its inter-frame dt stats, and the (arrival - device)
spread — i.e. the USB transfer jitter we must not bake into the stamp.

Run (needs root to claim the USB device):
  sudo pixi run python scripts/diag_timestamps.py            # ~100 frames
  sudo pixi run python scripts/diag_timestamps.py --frames 300
"""

import argparse
import os
import sys
import time

import numpy as np

from pyorbbecsdk import Config, OBFormat, OBSensorType, Pipeline


def _say(msg):
    print(msg, flush=True)


def _pick(pipeline, sensor_type, w, h, fmt, fps, label):
    profiles = pipeline.get_stream_profile_list(sensor_type)
    try:
        return profiles.get_video_stream_profile(w, h, fmt, fps)
    except Exception:
        print(f'  {label}: requested {w}x{h}@{fps} unavailable, using default')
        return profiles.get_default_video_stream_profile()


def _clocks(frame):
    def g(fn):
        try:
            v = fn()
            return int(v)
        except Exception:
            return -1
    return (g(frame.get_timestamp_us),
            g(frame.get_system_timestamp_us),
            g(frame.get_global_timestamp_us))


def _report(name, dev, arr, glob, host):
    dev, arr, glob, host = map(np.array, (dev, arr, glob, host))
    print(f'\n===== {name}: {len(dev)} frames =====')
    for label, ts in (('device ', dev), ('arrival', arr), ('global ', glob)):
        d = np.diff(ts.astype(np.int64))
        mono = bool(np.all(d > 0))
        uniq = len(np.unique(ts))
        dt_ms = d / 1e3  # us -> ms
        print(f'  {label}: unique {uniq}/{len(ts)}  monotonic {mono}  '
              f'dt_ms[min/med/max] '
              f'{dt_ms.min():.2f}/{np.median(dt_ms):.2f}/{dt_ms.max():.2f}  '
              f'zero-dt {int((d == 0).sum())}')
    # transfer jitter: arrival - device, both us. Only meaningful if device ok.
    if np.all(dev > 0):
        lag = (arr.astype(np.int64) - dev.astype(np.int64)) / 1e3  # ms
        lag -= lag.min()  # relative to least-delayed frame
        print(f'  arrival-device jitter ms [med/95th/max]: '
              f'{np.median(lag):.2f}/{np.percentile(lag, 95):.2f}/{lag.max():.2f}'
              f'   (this is the lag a constant offset cannot fix)')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--frames', type=int, default=100)
    ap.add_argument('--color-width', type=int, default=1920)
    ap.add_argument('--color-height', type=int, default=1080)
    ap.add_argument('--fps', type=int, default=30)
    ap.add_argument('--csv', default='/tmp/orbbec_clocks.csv',
                    help='where to stream raw per-frame clocks')
    args = ap.parse_args()

    # Stream raw per-frame clocks to a CSV as we go, flushed each line, so a
    # crash in the SDK's teardown can't cost us the data.
    csv_path = args.csv
    csv = open(csv_path, 'w')
    csv.write('stream,device_us,arrival_us,global_us,host_us\n')

    _say('creating pipeline...')
    pipeline = Pipeline()
    try:
        pipeline.get_device().timer_sync_with_host()
        _say('timer_sync_with_host: ok')
    except Exception as e:
        _say(f'timer_sync_with_host failed: {e}')

    config = Config()
    config.enable_stream(_pick(pipeline, OBSensorType.COLOR_SENSOR,
                               args.color_width, args.color_height,
                               OBFormat.MJPG, args.fps, 'color'))
    config.enable_stream(_pick(pipeline, OBSensorType.DEPTH_SENSOR,
                               640, 576, OBFormat.Y16, args.fps, 'depth'))
    try:
        pipeline.enable_frame_sync()
    except Exception as e:
        _say(f'enable_frame_sync failed: {e}')

    pipeline.start(config)
    _say(f'streaming; collecting {args.frames} framesets...')

    c_dev = []; c_arr = []; c_glob = []; c_host = []
    d_dev = []; d_arr = []; d_glob = []; d_host = []
    n = 0
    while n < args.frames:
        fs = pipeline.wait_for_frames(200)
        if fs is None:
            continue
        host = time.time() * 1e6
        cf = fs.get_color_frame()
        if cf is not None:
            dv, ar, gl = _clocks(cf)
            c_dev.append(dv); c_arr.append(ar); c_glob.append(gl); c_host.append(host)
            csv.write(f'color,{dv},{ar},{gl},{host:.0f}\n')
        df = fs.get_depth_frame()
        if df is not None:
            dv, ar, gl = _clocks(df)
            d_dev.append(dv); d_arr.append(ar); d_glob.append(gl); d_host.append(host)
            csv.write(f'depth,{dv},{ar},{gl},{host:.0f}\n')
        csv.flush()
        n += 1
        if n % 30 == 0:
            _say(f'  ...{n} framesets')

    # Report BEFORE any teardown, so an SDK abort in stop()/atexit can't hide it.
    if c_dev:
        _report('COLOR', c_dev, c_arr, c_glob, c_host)
    if d_dev:
        _report('DEPTH', d_dev, d_arr, d_glob, d_host)
    _say('\nInterpretation:')
    _say('  - pick the clock that is monotonic + all-unique for the stamp;')
    _say('  - if device is good, stamp = device + robust(arrival-device) offset;')
    _say('  - if only arrival is good, stamp = arrival - min_transfer_latency.')
    _say(f'\nraw per-frame clocks written to {csv_path}')
    csv.flush(); csv.close()
    sys.stdout.flush(); sys.stderr.flush()
    # Skip the SDK's C++ teardown entirely — it aborts with a recursive_mutex
    # error on macOS at interpreter shutdown. We already have all the data.
    os._exit(0)


if __name__ == '__main__':
    main()
