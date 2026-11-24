#!/usr/bin/env python3
"""Find which joystick axis carries each stick channel of an RC transmitter.

Walks through the four stick movements and prints the axis mapping part of an
SDL_GAMECONTROLLERCONFIG line for fly.sh (mode 2: left stick = throttle/yaw,
right stick = pitch/roll). Then, optionally, finds the switches you want to
fire with and reset with (controls/radio.cfg binds them to "a" and "b").

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


def find_switch(dev, what, used):
    """Ask for a switch to be held on; return its button number, or None to skip."""
    _, _, before = read_state(dev)
    answer = input(f'\n>> Switch for {what}: hold it ON and press Enter (or type s and Enter to skip)... ')
    if answer.strip().lower().startswith('s'):
        return None
    _, _, now = read_state(dev)
    changed = [i for i, (b, n) in enumerate(zip(before, now)) if b != n and i not in used]
    if len(changed) != 1:
        print(f'   {"no button changed" if not changed else f"buttons {changed} all changed"}; skipped')
        return None
    print(f'   {what}: button {changed[0]}')
    return changed[0]


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

    used = set()
    for sdl_button, what in (('a', 'fire (a momentary switch works best)'), ('b', 'reset')):
        button = find_switch(DEV, what, used)
        if button is not None:
            used.add(button)
            mapping.append(f'{sdl_button}:b{button}')

    print('\nMapping for SDL_GAMECONTROLLERCONFIG (see fly.sh):')
    print('   ' + ','.join(mapping))
    return 0


if __name__ == '__main__':
    sys.exit(main())
