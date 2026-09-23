# muto_teleop

Gamepad drive and camera pan/tilt control for the Raspberry Pi / ROS 2 Kilted Yahboom Muto
hexapod.

This package talks to the expansion board directly over serial. It depends on neither `MutoLib`
nor `muto_hexapod_lib`, for reasons in [Why no vendor library](#why-no-vendor-library) below.

> **Targets one robot.** The Pi / Kilted unit, which has a depth camera on a two-servo pan/tilt
> bracket. The older Jetson / Foxy Muto has a fixed camera and different driver code under
> identical filenames. Nothing here has been run on it.

## Status

| Piece | State |
|---|---|
| Serial frame layer (`muto_board.py`) | Working — frames verified byte-for-byte against the vendor implementation |
| Teleop node, gimbal control | **Working on hardware** — commanded through ROS, confirmed by camera |
| Teleop node, `/cmd_vel` → gaits | Implemented, **not yet driven on hardware** |
| Joystick input | Not yet built |

## Interface

| Topic | Type | Meaning |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Dominant axis picks a firmware gait; magnitude picks a speed |
| `/muto/gimbal` | `geometry_msgs/Vector3` | `x` = pan, `y` = tilt, absolute, in board units. `-1` holds an axis |
| `/Buzzer` | `std_msgs/Bool` | On is continuous |

### Gimbal units are not degrees

Pan is 0–180 and tilt 0–115, but these are servo units: tilt measures about **1.6° per unit**.
Home is **pan 90, tilt 4** — forward, roughly 10° below level.

| | 0 | Middle | Max |
|---|---|---|---|
| Pan | robot's **right** | 90 = forward | 180 = left |
| Tilt | **down** | ~9 = level | 115 = up (57 points at the ceiling) |

Increasing pan turns **left**, matching the ROS sign convention — which means a joystick's
right-is-positive axis has to be inverted to feel right.

The gimbal is write-only. There is no position feedback, so a command snaps to its target at full
speed from wherever the servo happens to be, and nothing in software can detect a collision. The
first command after power-up should be mid-range.

## Running

```bash
ros2 launch muto_teleop teleop.launch.py
```

The port defaults to `auto`, which finds the CH340 (`1a86:7523`) by USB id. Don't hard-code a
path: `/dev/myserial` is a udev symlink that exists on the host but **not inside the ROS
container**, and raw `ttyUSB` numbering moves when devices are replugged.

### Do not run this alongside anything else that wants the board

The serial port takes one writer, and **nothing enforces that**. pyserial opens a device another
process already holds, and the two interleave bytes into frames the board silently discards. The
symptom is commands that do nothing, not an error.

Two things commonly hold it:

```bash
sudo fuser -v /dev/ttyUSB0          # who has it
```

- `muto_driver` — this node replaces it
- `/home/pi/muto/app_muto/app_muto.py` — the vendor phone-app server, running on the **host**, not
  in a container and not a systemd service. Restart it with
  `python3 /home/pi/muto/app_muto/app_muto.py` when you're done.

## Why no vendor library

`muto_driver` on this robot imports `muto_hexapod_lib.core.MutoLibCore` from the muto-llm 2.0
framework — not `MutoLib`, whose import line is commented out. That library lives at
`/home/pi/muto-llm-2.0/`, which the ROS container does not mount, so it is not importable from
inside ROS at all. Depending on it would mean depending on a path that is absent where the code
runs.

The frames are therefore built here, and `test/test_muto_board.py` pins them against bytes
captured from the vendor implementation.

## Why firmware gaits instead of `move()`

The vendor software gait (`move()`) solves inverse kinematics on the Pi and writes 360 serial
frames per cycle, each followed by a 1 ms sleep — about 0.65 s per call, blocking, for one gait
cycle. The camera could only be updated between cycles, roughly 1.5 Hz, which is visibly jerky.

The firmware gaits are a single frame that the board's own firmware executes. The Pi stays free,
so the camera keeps moving smoothly while the robot walks. The cost is that the gaits are discrete
— six directions, five speeds, three heights — with no blending between them.

`move()` is unreachable from the container anyway, for the reason above.

### `/cmd_vel` is coarse here

A `Twist` cannot be honoured faithfully by a board with six discrete gaits, so the node picks the
dominant axis and maps its magnitude to a speed level. It cannot translate and rotate at once.

For reference, the vendor `muto_driver` has a sharper edge: it forces any **nonzero** yaw to at
least level 10, so through its `/cmd_vel` there is no gentle turn at all.

## Safety

- A firmware gait **runs until stopped**. There is no board-side timeout.
- The node stops the robot if no command arrives within `watchdog_timeout` (default 0.5 s), and on
  shutdown. Camera commands deliberately do **not** feed the watchdog — panning should not keep a
  walking robot walking after its driver has gone silent.
- Torque off leaves nothing holding the robot up. Support the body first.

## Development

Source lives on the development machine; the robot's workspace is a bind mount, so deployment is a
copy:

```bash
rsync -a --exclude='.git' --exclude='__pycache__' ./ \
  pi@<robot>:/home/pi/yahboomcar_ros2_ws/yahboomcar_ws/src/muto_teleop/
```

Then, inside the container:

```bash
source /opt/ros/kilted/setup.bash
cd /root/yahboomcar_ros2_ws/yahboomcar_ws
colcon build --packages-select muto_teleop
python3 -m pytest src/muto_teleop/test/ -q
```

Both workspaces are host-backed bind mounts, so builds survive `docker rm`. Anything `apt`
installed into the container does not — which is why this package adds no dependencies beyond
what the image already carries (`rclpy`, `pyserial`, the standard message packages).

Useful detail: the D435's colour stream is on `/dev/video4` on the **host**, so a frame can be
grabbed without ROS or the container at all:

```bash
ffmpeg -f v4l2 -input_format yuyv422 -video_size 640x480 -i /dev/video4 -frames:v 8 -y /tmp/f_%02d.jpg
```

Discard the first few frames — auto-exposure needs them.
