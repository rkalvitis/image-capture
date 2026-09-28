"""Idempotently apply the upstream natnet_py bug fix.

Why this exists
---------------
`AsyncClient.connect()` in `jeguzzi/natnet_py` (commit e5deb0e7 and likely
later) forgets to update `self.cmd_protocol._server` in the direct-connect
branch — only the discovery branch does. As a result, the first
`sendto(ConnectRequest, ('', 1510))` goes to an empty-string destination
and macOS rejects it with `[Errno 65] No route to host`.

This script inserts the missing assignment right after the
`info("Connecting to ...")` log. Safe to run multiple times: if the file is
already patched, this prints a notice and exits 0.

Worth filing upstream when convenient.
"""

from __future__ import annotations

import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
TARGET = HERE.parent / "src" / "natnet_py" / "natnet_py" / "async_client.py"

NEEDLE = 'self.logger.info(f"Connecting to {server_address} ...")'
ADDITION = "            self.cmd_protocol._server = server"
PATCHED_MARKER = NEEDLE + "\n" + ADDITION


def main() -> int:
    if not TARGET.exists():
        print(f"  natnet_py source not found at {TARGET}", file=sys.stderr)
        print("  (clone the repo into src/natnet_py/ first, or run pixi install)",
              file=sys.stderr)
        return 1

    src = TARGET.read_text()
    if PATCHED_MARKER in src:
        print(f"  {TARGET.name}: already patched, skipping")
        return 0
    if NEEDLE not in src:
        print(
            f"  ERROR: marker not found in {TARGET}\n"
            "  natnet_py upstream may have changed — review this patch.",
            file=sys.stderr,
        )
        return 2

    TARGET.write_text(src.replace(NEEDLE, NEEDLE + "\n" + ADDITION, 1))
    print(f"  patched {TARGET.name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
