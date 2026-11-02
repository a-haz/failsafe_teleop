# failsafe_teleop

A ROS 2 package with two small teleop nodes for driving a mobile robot by
hand, both built to stop the robot when something goes wrong. `joy_teleop`
reads a gamepad straight from the Linux joystick device and only drives while
a deadman button is held. `web_teleop` serves a touch joystick page for a
phone on the same network and stops when the phone goes quiet. Both publish
`geometry_msgs/Twist` on a configurable topic. I wrote them for driving a
small rover during my own research with a thermal camera.

## Behaviour

Gamepad (`joy_teleop`):

- Hold-to-drive: the robot moves only while the deadman button is held.
  Releasing it publishes zero.
- After the pad is opened or reopened, motion stays inhibited until the
  deadman has been released and pressed again, so a button that was already
  held when the pad connected never starts the robot.
- A lost or misbehaving pad (unplugged, I/O error, end of file, a short or
  interrupted read, or a flood of events) publishes zero in the same cycle,
  closes the device and retries every 0.5 s.
- With `override: true` the node publishes only while the deadman is held,
  plus one zero on release or loss, so a planner can drive while the pad is
  idle and holding the deadman with the stick centred is a hard stop.

Phone (`web_teleop`):

- The page sends commands only while a finger or the mouse is on the pad,
  about 15 times a second. Arrow keys or WASD also work, and the space bar
  toggles the e-stop.
- A watchdog publishes zero when no command has arrived for 0.4 s, which
  covers a dropped WiFi link, a locked phone and a closed tab.
- E-STOP latches zero until it is reset on the page. After a reset the robot
  stays still until a new command arrives.
- A token protects the page and the API: open `http://<robot>:8080/?t=<token>`.
  Set it with the `token` parameter; when it is empty, the node makes a
  random one at startup and logs the full URL.

Both nodes publish one last zero when they are stopped with Ctrl-C or
SIGTERM, before rclpy shuts down.

The decision logic lives in `failsafe_teleop/joystick.py` and
`failsafe_teleop/web.py`, which do not import ROS. The nodes only read that
logic on a timer (20 Hz by default) and publish the result.

## Hardware

Any robot base that takes `geometry_msgs/Twist`. The gamepad node works with
any pad the Linux kernel exposes as `/dev/input/jsN`; the defaults match an
Xbox 360 pad (left stick, X button as deadman). The web node needs only a
phone browser on the same network. `config/mega_rover.yaml` is an example
for a Vstone Mega Rover 3, whose base listens on `/rover_twist`.

## Install

In a ROS 2 workspace (written for Humble or later):

```
cd ~/ros2_ws/src
git clone https://github.com/a-haz/failsafe_teleop.git
cd ~/ros2_ws
colcon build --packages-select failsafe_teleop
source install/setup.bash
```

The user running the gamepad node needs read access to `/dev/input/jsN`
(usually the `input` group).

## Usage

```
ros2 run failsafe_teleop joy_teleop --ros-args -p cmd_vel_topic:=/cmd_vel -p max_linear:=0.5
ros2 run failsafe_teleop web_teleop --ros-args -p port:=8080 -p token:=pick-something
ros2 launch failsafe_teleop joy_teleop.launch.py params:=$(ros2 pkg prefix failsafe_teleop)/share/failsafe_teleop/config/mega_rover.yaml
```

Run one of the two at a time. If both publish to the same topic, the last
message wins.

Parameters of `joy_teleop`:

| name | default | meaning |
|---|---|---|
| device | /dev/input/js0 | joystick device |
| cmd_vel_topic | cmd_vel | output topic |
| max_linear | 0.3 | m/s at full stick |
| max_angular | 0.6 | rad/s at full stick |
| deadman_button | 2 | button index to hold |
| axis_linear, axis_angular | 1, 0 | stick axes |
| invert_linear, invert_angular | true, true | stick up and stick left are negative on most pads |
| deadzone | 0.15 | fraction of full stick ignored |
| rate_hz | 20.0 | publish rate |
| reopen_s | 0.5 | retry interval after the pad is lost |
| max_events | 64 | events read per cycle before the stream counts as faulty |
| override | false | publish only while the deadman is held |

Parameters of `web_teleop`:

| name | default | meaning |
|---|---|---|
| host, port | 0.0.0.0, 8080 | where the page is served |
| cmd_vel_topic | cmd_vel | output topic |
| max_linear | 0.3 | m/s at full deflection |
| max_angular | 0.6 | rad/s at full deflection |
| watchdog_s | 0.4 | zero after this long without a command |
| deadzone | 0.12 | applied in the page |
| token | (empty) | required in the URL; empty = a random one per run, shown in the log |
| battery_topic | (empty) | optional sensor_msgs/BatteryState shown on the page |
| rate_hz | 20.0 | publish rate |

The page is plain HTTP, so the token travels in clear text. Use it on a
network you trust.

## Running tests

The tests need only Python 3 and pytest. ROS does not need to be installed
to run them. From the repository root, optionally in a fresh virtual
environment:

```bash
python3 -m venv ~/venvs/failsafe_teleop
. ~/venvs/failsafe_teleop/bin/activate
python3 -m pip install pytest
python3 -m pytest test
```

They cover the pad logic through a real pipe and through a scripted device
(loss, reopen, re-arming, interrupted reads, floods, override mode), the
watchdog and e-stop latch, the HTTP API, and the node wiring against small
stand-ins for rclpy.

## License

MIT, see LICENSE.
