#!/usr/bin/env python3
"""Fake robot PC for testing the coordinator without the arm.

Speaks exactly what panda_manual_poses.py speaks in `_confirm_via:=socket`
mode (see confirm_socket.py): on connect `hello`, then `reached` for
id 0 (initial pose), waits for `go`, `reached` 1..N each followed by a
`go`, then `done`. One process = one run (= one turntable step); use
`--cycles` to repeat, like run_cycles.sh does with rosrun.

    python fake_robot.py --port 5600 --poses 9 --cycles 36 --move-time 0.2
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from confirm_socket import ConfirmServer  # noqa: E402


def fake_joints(k):
    return [0.1 * k, -0.5, 0.0, -2.0, 0.0, 1.5, 0.8]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', type=int, default=5600)
    ap.add_argument('--poses', type=int, default=9)
    ap.add_argument('--cycles', type=int, default=1)
    ap.add_argument('--move-time', type=float, default=0.2,
                    help='simulated travel time between poses (s)')
    a = ap.parse_args()
    for c in range(1, a.cycles + 1):
        print(f'[fake_robot] === run {c}/{a.cycles} ===', flush=True)
        srv = ConfirmServer(port=a.port, poses=a.poses)
        try:
            time.sleep(a.move_time)
            if not srv.wait(0, fake_joints(0), 'at initial pose'):
                print('[fake_robot] aborted at initial pose'); srv.done(); continue
            aborted = False
            for k in range(1, a.poses + 1):
                time.sleep(a.move_time)
                if not srv.wait(k, fake_joints(k), f'pose {k}/{a.poses} reached'):
                    print(f'[fake_robot] aborted at pose {k}'); aborted = True; break
            time.sleep(a.move_time)   # return to initial
            print('[fake_robot] back at initial pose' + (' (after abort)' if aborted else ''))
            srv.done()
        finally:
            time.sleep(0.2)
            srv.close()
    print('[fake_robot] all cycles done')


if __name__ == '__main__':
    main()
