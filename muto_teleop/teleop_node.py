"""ROS 2 node owning the Muto expansion board.

This node replaces ``muto_driver``. Both cannot run: the serial port takes one
writer, and nothing enforces that, so running both produces interleaved frames
rather than an error.

Why one node rather than a driver plus a separate gimbal node: the port is
exclusive, so camera control has to live in whichever process holds it. That is
the whole reason this exists.

Interface
---------
``/cmd_vel``        geometry_msgs/Twist   -- mapped to firmware gaits
``/muto/gimbal``    geometry_msgs/Vector3 -- x = pan, y = tilt, absolute
``/Buzzer``         std_msgs/Bool

``/cmd_vel`` here is coarse by nature. The board offers six discrete gaits and
five speeds, so a Twist picks the dominant axis and a speed band -- it cannot
blend translation and rotation. Anything wanting smooth velocity control wants
the software gait, which is not reachable from this container.
"""

import threading
import time

import rclpy
from geometry_msgs.msg import Twist, Vector3
from rclpy.node import Node
from std_msgs.msg import Bool

from muto_teleop.muto_board import (
    AUTO_PORT, GIMBAL_HOLD, PAN_MAX, PAN_MIN, SPEED_MAX, SPEED_MIN, TILT_MAX,
    TILT_MIN,
    MutoBoard, clamp,
)

# Below this the stick or velocity command counts as centred. Without it,
# analogue noise keeps restarting the gait.
DEFAULT_DEADZONE = 0.05


class MutoTeleopNode(Node):

    def __init__(self):
        super().__init__('muto_teleop')

        self.declare_parameter('port', AUTO_PORT)
        self.declare_parameter('tick_hz', 20.0)
        self.declare_parameter('watchdog_timeout', 0.5)
        self.declare_parameter('gait_step', 25)
        self.declare_parameter('deadzone', DEFAULT_DEADZONE)
        # Home is forward and about 10 degrees below level on this bracket.
        self.declare_parameter('home_pan', 90)
        self.declare_parameter('home_tilt', 4)
        self.declare_parameter('pan_min', PAN_MIN)
        self.declare_parameter('pan_max', PAN_MAX)
        self.declare_parameter('tilt_min', TILT_MIN)
        self.declare_parameter('tilt_max', TILT_MAX)
        self.declare_parameter('centre_gimbal_on_start', True)

        port = self.get_parameter('port').value
        self.watchdog_timeout = float(self.get_parameter('watchdog_timeout').value)
        self.gait_step = int(self.get_parameter('gait_step').value)
        self.deadzone = float(self.get_parameter('deadzone').value)

        try:
            self.board = MutoBoard(port)
        except Exception as exc:
            self.get_logger().fatal(
                f'cannot open {port}: {exc}. '
                'Another process may hold it -- check with: fuser -v /dev/ttyUSB0'
            )
            raise

        # Log what was resolved, not what was requested -- 'auto' tells nobody
        # which device this process actually has open.
        self.get_logger().info(f'muto_teleop holding {self.board.port}')

        # Desired state, written by callbacks and read by the writer thread.
        # Guarded because a serial frame assembled from half-updated state is a
        # frame that commands something nobody asked for.
        self._state_lock = threading.Lock()
        self._gait = None            # direction name, or None for stopped
        self._speed = SPEED_MAX
        self._pan = int(self.get_parameter('home_pan').value)
        self._tilt = int(self.get_parameter('home_tilt').value)
        self._last_command = time.monotonic()

        # Last values actually transmitted, so unchanged state costs no frames.
        self._sent_gait = None
        self._sent_speed = None
        self._sent_pan = None
        self._sent_tilt = None

        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 1)
        self.create_subscription(Vector3, 'muto/gimbal', self.on_gimbal, 1)
        self.create_subscription(Bool, 'Buzzer', self.on_buzzer, 1)

        if self.get_parameter('centre_gimbal_on_start').value:
            # The servos have no feedback, so the first command snaps at full
            # speed from an unknown position. Home is mid-range and safe.
            self.board.gimbal(self._pan, self._tilt)
            self._sent_pan, self._sent_tilt = self._pan, self._tilt
            self.get_logger().info(f'gimbal homed to pan={self._pan} tilt={self._tilt}')

        self._running = True
        self._writer = threading.Thread(target=self._writer_loop, daemon=True)
        self._writer.start()

    # -- subscriptions ----------------------------------------------------

    def on_cmd_vel(self, msg):
        """Map a Twist onto one firmware gait.

        The board cannot blend, so the dominant axis wins: rotation if yaw
        exceeds both translation components, otherwise whichever of x and y is
        larger. Speed comes from the magnitude of that same axis.
        """
        x, y, yaw = msg.linear.x, msg.linear.y, msg.angular.z

        if max(abs(x), abs(y), abs(yaw)) < self.deadzone:
            self.set_gait(None)
            return

        if abs(yaw) >= max(abs(x), abs(y)):
            direction = 'turn_left' if yaw > 0 else 'turn_right'
            magnitude = abs(yaw)
        elif abs(x) >= abs(y):
            direction = 'forward' if x > 0 else 'backward'
            magnitude = abs(x)
        else:
            # +y is the robot's left, matching the ROS convention.
            direction = 'left' if y > 0 else 'right'
            magnitude = abs(y)

        self.set_gait(direction, self._speed_from(magnitude))

    def on_gimbal(self, msg):
        """Absolute aim. x = pan, y = tilt, in board units, not degrees."""
        with self._state_lock:
            if msg.x != GIMBAL_HOLD:
                self._pan = int(clamp(msg.x,
                                      self.get_parameter('pan_min').value,
                                      self.get_parameter('pan_max').value))
            if msg.y != GIMBAL_HOLD:
                self._tilt = int(clamp(msg.y,
                                       self.get_parameter('tilt_min').value,
                                       self.get_parameter('tilt_max').value))
            # Aiming the camera is not a drive command and must not feed the
            # watchdog -- otherwise panning the camera keeps a walking robot
            # walking after its driver has gone quiet.

    def on_buzzer(self, msg):
        self.board.buzzer(255 if msg.data else 0)

    # -- state ------------------------------------------------------------

    def _speed_from(self, magnitude):
        """Map 0..1 onto the board's five speed levels."""
        magnitude = clamp(magnitude, 0.0, 1.0)
        return int(clamp(round(SPEED_MIN + magnitude * (SPEED_MAX - SPEED_MIN)),
                         SPEED_MIN, SPEED_MAX))

    def set_gait(self, direction, speed=None):
        with self._state_lock:
            self._gait = direction
            if speed is not None:
                self._speed = speed
            self._last_command = time.monotonic()

    # -- writer -----------------------------------------------------------

    def _writer_loop(self):
        """The only thread that talks to the board.

        Callbacks record intent; this sends it. Keeping a single writer means
        the serial ordering is whatever this loop decides, rather than whichever
        callback happened to fire first.
        """
        period = 1.0 / float(self.get_parameter('tick_hz').value)
        while self._running:
            try:
                self._tick()
            except Exception as exc:  # a dead serial port must not kill the loop
                self.get_logger().error(f'write failed: {exc}')
            time.sleep(period)

    def _tick(self):
        with self._state_lock:
            gait, speed = self._gait, self._speed
            pan, tilt = self._pan, self._tilt
            stale = time.monotonic() - self._last_command > self.watchdog_timeout

        # A firmware gait runs until told otherwise. If commands stop arriving --
        # the joystick unplugged, the sender crashed, wifi dropped -- the robot
        # would keep walking. Stop it.
        if stale and gait is not None:
            self.get_logger().warn(
                f'no command for {self.watchdog_timeout:g}s, stopping')
            with self._state_lock:
                self._gait = None
            gait = None

        # Gimbal first: it is one frame and the camera should not wait behind a
        # gait change.
        if (pan, tilt) != (self._sent_pan, self._sent_tilt):
            self.board.gimbal(pan, tilt)
            self._sent_pan, self._sent_tilt = pan, tilt

        if gait is None:
            if self._sent_gait is not None:
                self.board.stay_put()
                self._sent_gait = None
            return

        if speed != self._sent_speed:
            self.board.speed(speed)
            self._sent_speed = speed

        if gait != self._sent_gait:
            self.board.gait(gait, self.gait_step)
            self._sent_gait = gait

    # -- shutdown ---------------------------------------------------------

    def shutdown(self):
        """Stop the robot before dropping the port.

        Without this the board keeps running its last gait after the node exits,
        and nothing is left holding the port to stop it.
        """
        self._running = False
        if self._writer.is_alive():
            self._writer.join(timeout=1.0)
        try:
            self.board.stay_put()
            self.board.close()
        except Exception as exc:
            self.get_logger().error(f'shutdown failed: {exc}')


def main():
    rclpy.init()
    node = MutoTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
