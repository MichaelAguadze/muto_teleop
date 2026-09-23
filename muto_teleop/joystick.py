"""Linux joystick reader.

Reads ``/dev/input/js*`` directly rather than going through the ``joy`` package,
which is not installed in this robot's ROS container and cannot be added durably:
``apt`` installs are lost with the container, while only the bind-mounted
workspaces survive.

The device layer is small enough that this is a fair trade. What it does need to
handle is the receiver disappearing -- a wireless handle's dongle can be
unplugged mid-drive, and the robot must not keep walking on the last stick
values it saw.
"""

import os
import select
import struct
import threading
import time

# struct js_event: __u32 time, __s16 value, __u8 type, __u8 number
EVENT_FORMAT = '<IhBB'
EVENT_SIZE = struct.calcsize(EVENT_FORMAT)

JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
# Set on the state dump the kernel sends when the device is opened. Those carry
# real values and are worth keeping -- they prime the initial state -- but they
# are not evidence the handle is awake.
JS_EVENT_INIT = 0x80

AXIS_MAX = 32767.0

# How long to wait before retrying a device that is missing or failed to open.
REOPEN_INTERVAL = 1.0


class Joystick:
    """Background reader exposing the latest axis and button state.

    Axes are floats in -1.0..1.0. Buttons are bools. Both are read under a lock,
    so a caller always sees a consistent snapshot rather than a half-updated one.

    ``connected`` reflects whether the device is currently open. ``last_event``
    is the monotonic time of the last real (non-init) event, which is what tells
    a paired-but-asleep handle from an active one.
    """

    def __init__(self, device='/dev/input/js0', num_axes=8, num_buttons=16):
        self.device = device
        self._lock = threading.Lock()
        self._axes = [0.0] * num_axes
        self._buttons = [False] * num_buttons
        self._connected = False
        self._last_event = 0.0
        self._running = False
        self._thread = None

    # -- lifecycle --------------------------------------------------------

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    # -- state ------------------------------------------------------------

    @property
    def connected(self):
        with self._lock:
            return self._connected

    @property
    def last_event(self):
        with self._lock:
            return self._last_event

    def snapshot(self):
        """Return (axes, buttons, connected) as copies taken together."""
        with self._lock:
            return list(self._axes), list(self._buttons), self._connected

    def axis(self, index, default=0.0):
        with self._lock:
            return self._axes[index] if 0 <= index < len(self._axes) else default

    def button(self, index, default=False):
        with self._lock:
            return self._buttons[index] if 0 <= index < len(self._buttons) else default

    # -- reader -----------------------------------------------------------

    def _clear(self):
        """Zero every control. Called whenever the device goes away.

        Holding the last-seen values would leave a stick 'pushed' forever if the
        dongle were pulled mid-command.
        """
        with self._lock:
            self._axes = [0.0] * len(self._axes)
            self._buttons = [False] * len(self._buttons)
            self._connected = False

    def _loop(self):
        fd = None
        while self._running:
            if fd is None:
                try:
                    fd = os.open(self.device, os.O_RDONLY | os.O_NONBLOCK)
                    with self._lock:
                        self._connected = True
                except OSError:
                    self._clear()
                    time.sleep(REOPEN_INTERVAL)
                    continue

            try:
                if not select.select([fd], [], [], 0.1)[0]:
                    continue
                data = os.read(fd, EVENT_SIZE * 64)
                if not data:
                    raise OSError('device returned no data')
            except (OSError, ValueError):
                # Receiver unplugged, or the node vanished. Drop everything and
                # wait for it to come back.
                try:
                    os.close(fd)
                except OSError:
                    pass
                fd = None
                self._clear()
                time.sleep(REOPEN_INTERVAL)
                continue

            self._ingest(data)

        if fd is not None:
            try:
                os.close(fd)
            except OSError:
                pass
        self._clear()

    def _ingest(self, data):
        now = time.monotonic()
        with self._lock:
            for off in range(0, len(data) - EVENT_SIZE + 1, EVENT_SIZE):
                _, value, ev_type, number = struct.unpack_from(EVENT_FORMAT, data, off)
                is_init = bool(ev_type & JS_EVENT_INIT)

                if ev_type & JS_EVENT_AXIS:
                    if 0 <= number < len(self._axes):
                        self._axes[number] = max(-1.0, min(1.0, value / AXIS_MAX))
                elif ev_type & JS_EVENT_BUTTON:
                    if 0 <= number < len(self._buttons):
                        self._buttons[number] = bool(value)

                if not is_init:
                    self._last_event = now
