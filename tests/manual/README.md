# Manual G1 safety tests

These tests exercise the path from an arm policy through the ROS command checker
and hardware motor gate to the G1. The checker validates proposed commands, and
the gate watches command expiry and robot feedback before writing motor targets.
A separate controller monitor broadcasts Start aborts. The hardware gate also
reads Start and F3 directly from SDK telemetry, independently of the policy.

The [safety installer](../../install_safety_stops.sh) runs that hardware gate as
the background user service `g1-safety-stops.service`. It builds the ROS overlay,
starts immediately, starts again at boot with user lingering, and restarts on
process failure. It does not launch an arm test. See the
[installation and service reference](../../robot_safety/README.md#background-safety-watchdog)
for dependencies, configuration and removal.

Double presses require press–release–press within 0.5 seconds. Double **F3**
requests the local GR00T lower-body balance policy and holds the captured arm
pose. Double **Start** requests stock whole-body damping (`SetFsmId(1)`), which
can fold the robot. Ordinary test completion or faults after arm ownership
normally hold the measured arm pose, with damping if that hold cannot continue.
These are different outcomes: completion alone does not test either button.

## Prepare the robot and background service

Use the supported gantry/harness setup, with feet bearing weight and room for
arm movement and balance corrections. This setup targets the 29-DOF G1 EDU,
machine type 5, seven joints per arm, no dexterous hands and no waist fixing
pieces. Begin in locked standing/ready mode (FSM 4), with joints near stationary
and Start and F3 released. Locked standing describes the operating mode.
Physical stop behavior remains under validation following a reported joint
shutdown during a T-pose stop.

Run all commands on the robot host from this repository. The defaults are SDK
interface `eth0`, SDK domain 0 and ROS localhost domain 97. The service currently
uses [the lenient profile](../../robot_safety/profiles/g1_type5.lenient.json),
not [the stricter template](../../robot_safety/profiles/g1_type5.template.json).
Review the selected [profile and limits](../../robot_safety/profiles/README.md)
before hardware testing. F3 also requires the balance ONNX model and its runtime.

```bash
./install_safety_stops.sh install
systemctl --user is-enabled g1-safety-stops.service
systemctl --user status g1-safety-stops.service
loginctl show-user "$USER" -p Linger
journalctl --user -u g1-safety-stops.service -f
```

Expect `enabled`, an active service, `Linger=yes`, and eventually the gate's
`Hardware output enabled` message after the build. Resolve build, SDK, telemetry
or `balance network failed to load` errors before starting motion. Systemd can
report the launcher as active while it is still building. To check persistence,
repeat these checks after a planned reboot before starting a test.

## Run an arm test through the systemd gate

The convenience launcher `run_unitree_arm_test.sh --hardware` starts its own
hardware gate. **Do not run it alongside the service.** To test the installed
watchdog, leave the service running and start only the other four processes as
shown below. Do not run another test graph or SDK motor publisher concurrently.

In each test terminal, prepare the same environment, using the service's overlay
path if you have overridden it:

```bash
cd /home/unitree/him-sim2real-management
source /opt/ros/foxy/setup.bash
source "$HOME/.him-safety/ros-install/setup.bash"
export ROS_DOMAIN_ID=97 ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export CYCLONEDDS_HOME=/home/unitree/cyclonedds_ws/install/cyclonedds
export LD_LIBRARY_PATH="$CYCLONEDDS_HOME/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="$PWD:${PYTHONPATH:-}"
```

Start telemetry in the first terminal:

```bash
ros2 run him_safety_nodes safety_node feedback --ros-args -p sdk_interface:=eth0 -p sdk_domain:=0
```

Once telemetry is flowing, start the checker in a second terminal and the
controller monitor in a third:

```bash
# Terminal 2
ros2 run him_safety_nodes safety_node checker
```

```bash
# Terminal 3
ros2 run him_safety_nodes safety_node controller_stop --ros-args -p startup_timeout_secs:=5.0
```

Start the arm producer in a fourth terminal:

```bash
/usr/bin/python3 -m tests.manual.arm_raise_node --ros-args -p test_policy:=unitree_arm7 -p handover_timeout_secs:=5.0
```

Wait for `Ready for explicit start request`. In another prepared terminal,
check readiness and request one run:

```bash
ros2 service call /safety/motor_gate/ready std_srvs/srv/Trigger '{}'
ros2 service call /safety/arm_raise/start std_srvs/srv/Trigger '{}'
```

Readiness must succeed. The start request repeats readiness and coordinates
arming. The producer holds captured arm/waist positions during the default
3-second ownership ramp and waits for a full-weight SDK write. It then runs an
18-second trajectory: targets approach zero for 3 seconds, both arms raise for
6 seconds, then return toward zero for 9 seconds. The stock controller owns the
legs during the arm trajectory. A full-weight write is not proof of takeover.

For a smaller first motion, use `test_policy:=small_elbow` instead. After handover,
the left elbow target decreases by 0.05 rad over 10 seconds and holds for
2 seconds. Completion reports `small_elbow_complete` if the measured elbow is
within 0.02 rad of the target, or `small_elbow_did_not_follow` otherwise.

## Verify both stops

Keep the service journal visible throughout the test. While the arms are moving,
double press F3. Confirm that the trajectory stops, the arms hold their captured
pose, and the lower-body balance policy takes over. The robot may make balance
corrections. The service status output should show
`hardware_controller_double_f3` and `walk_stop: true`. That flag records the
request, so also check for model errors and observe the physical response.

Use a fresh test run to double press Start during arm movement. Confirm that
motion stops and the robot enters whole-body damping. Look for
`hardware_controller_double_start`, a successful `damping_rpc_code` of `0` and
`reported_fsm_id` of `1` in the service journal. Status can subsequently clear
after damping is confirmed. RPC success and reported mode alone do not prove
physical damping. Double Start should also override an active F3 balance stop.

For each trial, record the gesture, visible robot response and journal output.
Check that a single press or a held button does not trigger this double-press
handler. Seeing the stop in the systemd service's journal while it is the only
hardware gate establishes that this test exercised the background process.
An `active` service or a completed arm trajectory alone is insufficient.

After a stop or completion, the test graph remains latched. With the robot
supported, exit all four foreground processes, restart the service with
`systemctl --user restart g1-safety-stops.service`, restore ready mode, and
start fresh processes for the next trial. Restarting only the gate leaves
retained aborts in the other nodes. Gate shutdown may request damping. After
the final test, use the same coordinated cleanup and leave the service running
without the test processes for normal background monitoring.

## Diagnostics and standalone tests

In a prepared terminal, inspect `/safety/hardware_status` or joint extrema:

```bash
ros2 topic echo /safety/hardware_status std_msgs/msg/String
ros2 topic echo /safety/joint_diagnostics std_msgs/msg/String
```

Joint diagnostics report tracking error, target step and measured velocity for
the 17 controlled joints, including rejected proposals. They reset on process
restart and are not proof of executed motion. The gate logs final extrema on
abort, which places them in the service journal for the procedure above.

For standalone testing, first stop the service with the robot supported. Then
`./tests/manual/run_unitree_arm_test.sh --hardware` launches the complete graph,
and `./tests/manual/run_unitree_arm_test.sh --start` requests a run from another
terminal. Add `--small-step` before `--hardware` for the elbow test. This tests
the foreground gate, not the systemd process. Exit the complete graph before
restarting the service.

With the service stopped, `./tests/manual/run_arm_raise_dry_run.sh` and its
`--start` command exercise a diagnostic 0.15 rad shoulder raise over 10 seconds
using real telemetry without SDK motor writes. This does not validate physical
stopping. Automated ROS tests use localhost domain 191, separate from manual
domain 97.
