#!/usr/bin/env python3
"""Read a Linux joystick (/dev/input/jsN): name, axis and button counts, current values."""
import array, fcntl, os, struct, sys, time

JSIOCGAXES = 0x80016a11
JSIOCGBUTTONS = 0x80016a12
JSIOCGNAME = 0x80806a13
JS_EVENT_BUTTON, JS_EVENT_AXIS, JS_EVENT_INIT = 0x01, 0x02, 0x80


def read_state(dev='/dev/input/js0', settle=0.3):
    """Return (name, axes, buttons) with the device's current values.

    The device is opened fresh each time: the driver only reports the full
    state right after open, and later just the values that change.
    """
    fd = os.open(dev, os.O_RDONLY | os.O_NONBLOCK)
    try:
        buf = array.array('B', [0]); fcntl.ioctl(fd, JSIOCGAXES, buf); naxes = buf[0]
        buf = array.array('B', [0]); fcntl.ioctl(fd, JSIOCGBUTTONS, buf); nbuttons = buf[0]
        name = array.array('B', [0] * 128); fcntl.ioctl(fd, JSIOCGNAME, name)
        name = name.tobytes().split(b'\0')[0].decode(errors='replace')

        axes, buttons = [0] * naxes, [0] * nbuttons
        end = time.time() + settle
        while time.time() < end:
            try:
                ev = os.read(fd, 8)
            except BlockingIOError:
                time.sleep(0.005)
                continue
            _, value, kind, number = struct.unpack('IhBB', ev)
            kind &= ~JS_EVENT_INIT
            if kind == JS_EVENT_AXIS and number < naxes:
                axes[number] = value
            elif kind == JS_EVENT_BUTTON and number < nbuttons:
                buttons[number] = value
        return name, axes, buttons
    finally:
        os.close(fd)


if __name__ == '__main__':
    name, axes, buttons = read_state(sys.argv[1] if len(sys.argv) > 1 else '/dev/input/js0')
    print(f'{name}: {len(axes)} axes, {len(buttons)} buttons')
    print('axes   ', ' '.join(f'{i}:{v:+6d}' for i, v in enumerate(axes)))
    print('buttons', ''.join(str(b) for b in buttons))
