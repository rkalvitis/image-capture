#!/usr/bin/env python3
"""Dump every rigid body's marker constellation from Motive to YAML.

    pixi run dump-bodies          # needs Motive streaming on the lab LAN

Motive's NatNet data descriptions include, for each rigid body, the marker
positions RELATIVE TO THE BODY PIVOT — exactly the constellation the body
frame is built from. This script connects like the driver does, requests
the descriptions, converts marker offsets to the ROS convention used by the
whole pipeline (x, y, z)_ros = (x, -z, y)_motive, and writes
calibration/rigid_bodies.yaml. Consumed by scripts/plot_handeye.py to draw
the markers alongside the calibrated camera.

Re-run after any change to a rigid body in Motive (markers moved, pivot
moved, body re-created).
"""

import argparse
import asyncio
import logging
import os
import socket
from datetime import datetime

import yaml
from natnet_py import AsyncClient


def default_client_ip(server: str) -> str:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect((server, 1510))
        return s.getsockname()[0]
    except OSError:
        return '0.0.0.0'      # no route to the Motive PC (Mac was .245, robot PC .158): bind all
    finally:
        s.close()


async def run(server: str, client_ip: str, timeout: float):
    logging.basicConfig(level=logging.WARNING)
    client = AsyncClient(address=client_ip, queue=-1,
                         logger=logging.getLogger('natnet'))
    ok = await client.connect(discovery_address='', server_address=server,
                              timeout=timeout, start_listening_for_data=False)
    if not ok:
        raise SystemExit(f'could not connect to Motive at {server} '
                         f'(client bind {client_ip})')
    await client.update_description(timeout=timeout)
    desc = client.description
    await client.unconnect()
    if desc is None:
        raise SystemExit('Motive returned no data descriptions')
    return desc


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', default='calibration/rigid_bodies.yaml')
    ap.add_argument('--timeout', type=float, default=5.0)
    args = ap.parse_args()

    server = os.environ.get('OPTITRACK_PC_IP', '192.168.201.5')
    client_ip = os.environ.get('MY_IP') or default_client_ip(server)
    print(f'connecting to Motive at {server} (local bind {client_ip})')
    desc = asyncio.run(run(server, client_ip, args.timeout))

    bodies = {}
    for rb in desc.rigid_bodies:
        markers = []
        for i, m in enumerate(rb.markers):
            x, y, z = m.position                     # Motive frame (y-up)
            markers.append({
                'name': m.name or f'marker_{i + 1}',
                # ROS convention, matching the streamed body frame
                'position_m': [float(x), float(-z), float(y)],
            })
        bodies[rb.name] = {'id': rb.id, 'markers': markers}
        print(f'  {rb.name}: {len(markers)} markers')

    os.makedirs(os.path.dirname(args.out) or '.', exist_ok=True)
    with open(args.out, 'w') as f:
        yaml.safe_dump({
            'date': datetime.now().isoformat(timespec='seconds'),
            'note': 'marker positions relative to each body pivot, '
                    'ROS axes (x,y,z)_ros = (x,-z,y)_motive',
            'bodies': bodies,
        }, f, sort_keys=False)
    print(f'wrote {args.out}')


if __name__ == '__main__':
    main()
