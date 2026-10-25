"""Exercise real SDK packets with an offline writer."""

from threading import Lock
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from robot_safety.unitree_transport import UnitreeArmTransport
from tests.test_transport_diagnostics import targets

sdk = pytest.importorskip("unitree_sdk2py.idl.default")
CRC = pytest.importorskip("unitree_sdk2py.utils.crc").CRC


@pytest.fixture
def output():
    transport = UnitreeArmTransport.__new__(UnitreeArmTransport)
    transport.output_enabled = True
    transport.write_lock = Lock()
    transport.make_command = sdk.unitree_hg_msg_dds__LowCmd_
    transport.crc = SimpleNamespace(Crc=Mock(wraps=CRC().Crc))
    packets = []
    transport.lowcmd = SimpleNamespace(
        Write=lambda message: packets.append(message.serialize()) or True,
    )
    return transport, packets


def test_resends_reuse_crc_until_machine_mode_changes(output):
    transport, packets = output
    assert transport.write_body(targets(), 5)
    for _ in range(3):
        assert transport.repeat_body(5)
    assert packets == [packets[0]] * 4
    assert transport.crc.Crc.call_count == 1

    assert transport.repeat_body(7)
    assert transport.repeat_body(7)
    assert packets[-1] == packets[-2] != packets[0]
    message = type(transport.make_command()).deserialize(packets[-1])
    assert message.mode_machine == 7
    assert message.crc == CRC().Crc(message)
    assert transport.crc.Crc.call_count == 2


def test_new_commands_replace_resends_and_clearing_stops_them(output):
    transport, packets = output
    command = targets()
    assert transport.write_body(command, 5)
    command[4] = dict(q=.2, dq=.3, kp=25., kd=2., tau=.5)
    assert transport.write_body(command, 5)
    assert transport.repeat_body(5)
    assert packets[-1] == packets[-2] != packets[0]
    message = type(transport.make_command()).deserialize(packets[-1])
    for field, value in command[4].items():
        assert getattr(message.motor_cmd[4], field) == pytest.approx(value)
    assert message.crc == CRC().Crc(message)
    assert transport.crc.Crc.call_count == 2

    transport.clear_body()
    count = len(packets)
    assert transport.repeat_body(5) is None
    assert len(packets) == count
    assert transport.write_body(targets(), 5)
    assert transport.repeat_body(5)
    assert packets[-1] == packets[-2] == packets[0]
