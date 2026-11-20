#!/usr/bin/env python3
"""Find which joystick axis carries each stick channel of an RC transmitter.

Walks through the four stick movements and prints the axis mapping part of an
SDL_GAMECONTROLLERCONFIG line for fly.sh (mode 2: left stick = throttle/yaw,
right stick = pitch/roll).

usage: tools/calibrate.py [/dev/input/jsN]
"""
import os, sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from jsread import read_state  # noqa: E402

DEV = sys.argv[1] if len(sys.argv) > 1 else '/dev/input/js0'

# stick move -> (SDL gamepad axis it is mapped to, prompt)
# fpv.cfg expects each of these moves to read as a positive value on that axis.
STEPS = (
    ('throttle', 'lefty', 'Throttle fully UP (other sticks centred).'),
    ('roll', 'rightx', 'Throttle back LOW. RIGHT stick fully RIGHT (roll right).'),
    ('pitch', 'righty', 'RIGHT stick fully UP / forward (pitch forward).'),
    ('yaw', 'leftx', 'LEFT stick fully RIGHT (yaw right), throttle still low.'),
)


def wait(prompt):
    input(f'\n>> {prompt}\n   Hold it there and press Enter... ')


def main():
    wait('Throttle LOW, other sticks CENTRED.')
    name, base, _ = read_state(DEV)
    print(f'   "{name}", {len(base)} axes. Rest position: {base}')

    mapping = []
    used = set()
    for channel, sdl_axis, prompt in STEPS:
        wait(prompt)
        _, now, _ = read_state(DEV)
        deltas = [n - b for b, n in zip(base, now)]
        axis = max((i for i in range(len(deltas)) if i not in used), key=lambda i: abs(deltas[i]))
        if abs(deltas[axis]) < 8000:
            print(f'   {channel}: no axis moved much (biggest change {deltas[axis]:+d}); check the radio is in USB joystick mode')
            return 1
        used.add(axis)
        invert = '~' if deltas[axis] < 0 else ''
        mapping.append(f'{sdl_axis}:a{axis}{invert}')
        print(f'   {channel}: axis {axis}{" (reversed)" if invert else ""}')

    print('\nAxis mapping for SDL_GAMECONTROLLERCONFIG (see fly.sh):')
    print('   ' + ','.join(mapping))
    return 0


if __name__ == '__main__':
    sys.exit(main())
