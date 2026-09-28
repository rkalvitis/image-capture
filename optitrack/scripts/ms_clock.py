#!/usr/bin/env python3
"""Millisecond wall-clock display for the session clock photo.

Run `pixi run ms-clock`, make the terminal window big, and photograph the
screen with the iPhone at the START and END of every photo session. Comparing
the displayed time against the photo's own EXIF capture time gives the
iPhone↔Mac clock offset (consumed by scripts/match_photos.py --clock-shows).

The display shows this Mac's system clock — the same clock that stamps the
rosbag. Accuracy of the method is bounded by the screen refresh (~60 Hz,
so ±20 ms) — far better than the stop-and-shoot protocol needs.

Ctrl-C to quit.
"""

import sys
import time

# 5-row block-character digits, readable in a photo across the room.
_GLYPHS = {
    '0': ('█████', '█   █', '█   █', '█   █', '█████'),
    '1': ('   █ ', '  ██ ', '   █ ', '   █ ', '  ███'),
    '2': ('█████', '    █', '█████', '█    ', '█████'),
    '3': ('█████', '    █', ' ████', '    █', '█████'),
    '4': ('█  █ ', '█  █ ', '█████', '   █ ', '   █ '),
    '5': ('█████', '█    ', '█████', '    █', '█████'),
    '6': ('█████', '█    ', '█████', '█   █', '█████'),
    '7': ('█████', '    █', '   █ ', '  █  ', '  █  '),
    '8': ('█████', '█   █', '█████', '█   █', '█████'),
    '9': ('█████', '█   █', '█████', '    █', '█████'),
    ':': ('     ', '  █  ', '     ', '  █  ', '     '),
    '.': ('     ', '     ', '     ', '     ', '  █  '),
}


def _render(text: str) -> str:
    rows = ['  '.join(_GLYPHS[c][r] for c in text) for r in range(5)]
    return '\n'.join(rows)


def main() -> None:
    sys.stdout.write('\x1b[2J\x1b[?25l')  # clear screen, hide cursor
    try:
        while True:
            now = time.time()
            local = time.localtime(now)
            ms = int((now % 1) * 1000)
            text = time.strftime('%H:%M:%S', local) + f'.{ms:03d}'
            frame = (
                '\x1b[H'  # cursor home (no flicker vs full clear)
                + '\n  Mac system clock — photograph this screen\n\n'
                + _render(text)
                + '\n\n  ' + time.strftime('%Y-%m-%d', local)
                + f'   (unix {now:.3f})\n'
            )
            sys.stdout.write(frame)
            sys.stdout.flush()
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        sys.stdout.write('\x1b[?25h\n')  # show cursor again


if __name__ == '__main__':
    main()
