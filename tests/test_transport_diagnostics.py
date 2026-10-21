"""Transport diagnostics use fake writers and readers only."""

from threading import Event, Lock
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from robot_safety.sdk_latest import LatestSdkPoller
from robot_safety.unitree_transport import UnitreeArmTransport


def transport():
    instance = UnitreeArmTransport.__new__(UnitreeArmTransport)
    instance.output_enabled = True
    instance.write_lock = Lock()
    instance._body_ready = False
    instance._body_message = None
    instance.lowcmd = SimpleNamespace(Write=Mock(return_value=True))
    instance.crc = SimpleNamespace(Crc=lambda _: 123)
    instance._encode_body = lambda targets, machine: SimpleNamespace(
        targets=targets, mode_machine=machine, crc=123)
    return instance


def targets():
    return {i: dict(q=.1, dq=0., kp=20., kd=1., tau=0.) for i in range(29)}


def test_write_diagnostics_count_generations_success_and_clear():
    instance = transport()
    instance.set_diagnostics_enabled(True)
    assert instance.write_body(targets(), 5)
    first = instance.diagnostics_snapshot()['body']
    assert instance.repeat_body(5)
    second = instance.diagnostics_snapshot()['body']
    assert first['generation'] == second['generation'] == 1
    assert first['write_count'] == 1
    assert second['write_count'] == second['success_count'] == 2
    assert second['fresh_count'] == second['repeat_count'] == 1
    assert second['last_write_finished_s'] >= second['last_write_started_s']
    assert second['last_lock_wait_s'] >= 0
    assert second['last_lock_held_s'] >= second['last_write_duration_s']
    instance.clear_body()
    assert instance.repeat_body(5) is None
    cleared = instance.diagnostics_snapshot()['body']
    assert cleared['clear_count'] == 1
    assert not cleared['ready']
    assert instance.write_body(targets(), 5)
    assert instance.diagnostics_snapshot()['body']['generation'] == 2
    assert first['write_count'] == 1


@pytest.mark.parametrize('failure', [False, RuntimeError('fake write failure')])
def test_failed_write_diagnostics_preserve_success_timestamp(failure):
    instance = transport()
    instance.set_diagnostics_enabled(True)
    instance.write_body(targets(), 5)
    success = instance.diagnostics_snapshot()['body']['last_success_s']
    if isinstance(failure, Exception):
        instance.lowcmd.Write.side_effect = failure
        with pytest.raises(RuntimeError, match='fake write failure'):
            instance.repeat_body(5)
    else:
        instance.lowcmd.Write.return_value = failure
        assert instance.repeat_body(5) is False
    snapshot = instance.diagnostics_snapshot()['body']
    assert snapshot['write_count'] == 2
    assert snapshot['success_count'] == 1
    assert not snapshot['last_write_success']
    assert snapshot['last_success_s'] == success


def test_disabled_diagnostics_leave_writes_uninstrumented():
    instance = transport()
    assert instance.write_body(targets(), 5)
    assert instance.repeat_body(5)
    instance.clear_body()
    assert instance.diagnostics_snapshot() == dict(body={}, state_poller={})


def test_poller_exposes_decode_timing_before_handler():
    completed = Event()
    observed = []
    sample = SimpleNamespace(tick=42, sample_info=SimpleNamespace(source_timestamp=123456))
    reader = SimpleNamespace(topic='offline', take_latest=lambda: sample)

    def handler(value):
        observed.append((value, poller.diagnostics))
        poller.stopped.set()
        completed.set()

    poller = LatestSdkPoller(reader, handler, .005)
    poller.diagnostics_enabled = True
    poller.start()
    assert completed.wait(1.)
    poller.stop()
    value, entry = observed[0]
    assert value is sample
    assert entry['tick'] == 42
    assert entry['source_timestamp_ns'] == 123456
    assert entry['take_finished_s'] >= entry['take_started_s']
    assert 'handler_duration_s' not in entry
    assert poller.diagnostics['handler_duration_s'] >= 0
