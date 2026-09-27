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

The hardware test uses the existing `g1-safety-stops.service`. It checks that
this **user service** is active before starting any test nodes. It does not
start a second hardware gate or stop/restart the service. The service owns the
hardware profile and performs its own warmup.

If a previous run stopped or completed, follow [Recovery between trials](#recovery-between-trials)
before launching again. Restarting the test alone does not clear a latched stop.

```bash
cd /home/unitree/him-sim2real-management
systemctl --user is-active g1-safety-stops.service
./tests/manual/run_unitree_arm_test.sh --hardware
```

If the service is inactive, start it first and wait for warmup:
`systemctl --user start g1-safety-stops.service`.
Do not run another test graph or SDK motor publisher concurrently.

Wait for `Ready for explicit start request`. In another terminal:

```bash
cd /home/unitree/him-sim2real-management
./tests/manual/run_unitree_arm_test.sh --check-ready
./tests/manual/run_unitree_arm_test.sh --start
```

The foreground launch contains telemetry, checker, controller monitor and arm
producer only. Ctrl+C closes these processes and leaves the service running.
The handover deadline defaults to five seconds. For a longer service ownership
ramp, pass `handover_timeout_secs:=SECONDS` with at least one second of margin.

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

## Recovery between trials

An immediate `Abort latched: hardware_controller_double_f3` on a new test can
come from a previous F3 stop, without a new button press. While latched, the
service repeats the abort on `/safety/abort`. This topic also retains the last
message for new subscribers (`TRANSIENT_LOCAL` durability). Stopping and
relaunching only the test therefore does not clear the stop.

**Double-Start damping is the normal recovery path while keeping the service
running.** F3 balance remains a stopped state and does not permit another arm
test. After a stop or completion:

1. Support the robot and exit all foreground test processes with Ctrl+C.
   Old test nodes can rebroadcast their own latched aborts if left running.
2. Double-press Start. Verify physical damping and check the service journal
   for `damping_rpc_code: 0`, `reported_fsm_id: 1`, and `arm_release_written: true`.
   The service clears its old session and retained abort only after those
   software checks succeed. If damping or arm release fails, the stop remains
   latched. Software status alone does not prove physical damping.
3. Restore ready mode, launch a fresh test, and explicitly run `--check-ready`
   and `--start` as above. Clearing the stop does not automatically rearm.

A service restart also removes its in-memory state, but is not the normal
between-trial procedure and leaves a gap in monitoring. Leave the service
running after the final test.

## Diagnostics and standalone tests

Source ROS and the test overlay, then inspect status or joint extrema:

```bash
source /opt/ros/foxy/setup.bash
source /tmp/him-safety-ros-install/setup.bash
export ROS_DOMAIN_ID=97 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 topic echo /safety/hardware_status std_msgs/msg/String
ros2 topic echo /safety/joint_diagnostics std_msgs/msg/String
```

Joint diagnostics report tracking error, target step and measured velocity for
the 17 controlled joints, including rejected proposals. They reset on process
rearm and are not proof of executed motion. The gate logs final extrema on
abort, which places them in the service journal for the procedure above.

Use `./tests/manual/run_unitree_arm_test.sh --small-step --hardware` for the
elbow test through the same service, followed by `--start` in another terminal.
The launcher does not clear an existing abort or reset a used session. The
coordinated recovery described above is still required after a stopped run.

With the service stopped, `./tests/manual/run_arm_raise_dry_run.sh` and its
`--start` command exercise a diagnostic 0.15 rad shoulder raise over 10 seconds
using real telemetry without SDK motor writes. This does not validate physical
stopping. Automated ROS tests use localhost domain 191, separate from manual
domain 97.
