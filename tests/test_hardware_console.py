"""Operator output filtering and passive button detection with fake telemetry."""

import json
from types import SimpleNamespace

from robot_safety.console_output import HardwareConsole
from tests.test_startup_hardware import rig


def record(count=0, stamp=None):
    return json.dumps(dict(state=dict(left_b_count=count, left_b_at_secs=stamp),
                           command=dict(q=[0.] * 29), inference_secs=0.001)) + '\n'


def test_telemetry_is_quiet_and_each_new_press_prints_once():
    console = HardwareConsole(started=10)
    assert console.line(record()) is None
    assert console.line(record(1, 11)) == 'D-pad Left + B detected\n'
    for _ in range(100):
        assert console.line(record(1, 11)) is None
    assert console.line(record(2, 12)) == 'D-pad Left + B detected\n'


def test_old_press_and_restart_do_not_replay_notifications():
    console = HardwareConsole(started=10)
    assert console.line(record(5, 9)) is None
    assert console.line(record()) is None
    assert console.line(record(1, 11)) == 'D-pad Left + B detected\n'


def test_bundle_chatter_is_hidden_but_faults_and_tracebacks_remain():
    console = HardwareConsole(started=10)
    for line in ('Bundle hashes: {\n', '  "policy": "hash"\n', '}\n', '\n',
                 'Checking hardware readiness before arming\n'):
        assert console.line(line) is None
    for line in ('Policy abort: joint_torque_limit\n', 'Traceback (most recent call last):\n',
                 '  File "/example/client.py", line 83\n', 'RuntimeError: arming rejected\n',
                 '{"error": "test", "state": {}}\n'):
        assert console.line(line) == line
    assert console.line('Bundle hashes: {\n') is None
    assert console.line('RuntimeError: failed\n') == 'RuntimeError: failed\n'


def test_left_b_notice_counts_existing_edges_without_taking_control(rig):
    def sample(pressed, tick):
        rig.now += 0.001
        remote = [0] * 40
        remote[3] = ((1 << 7) | (1 << 1)) if pressed else 0
        rig.adapter.receive_state(SimpleNamespace(
            tick=tick, mode_machine=5, wireless_remote=remote,
            motor_state=[SimpleNamespace(q=0., dq=0., temperature=(25, 25), tau_est=0.) for _ in range(29)],
            imu_state=SimpleNamespace(quaternion=(1., 0., 0., 0.), gyroscope=(0., 0., 0.))))

    sample(True, 1)
    assert rig.adapter.left_b_count == 1
    for tick in range(2, 30):
        sample(True, tick)
    assert rig.adapter.left_b_count == 1
    sample(False, 29)
    sample(True, 29)
    assert rig.adapter.left_b_count == 1
    sample(False, 30)
    sample(True, 31)
    assert rig.adapter.left_b_count == 2
    assert not rig.adapter.armed
    assert not rig.adapter.launch_requested
    assert rig.adapter.startup.handoff is None
    assert rig.transport.release_calls == rig.transport.stop_calls == 0
    rig.transport.write_body.assert_not_called()
