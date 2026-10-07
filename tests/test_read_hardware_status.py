"""Status-reader behavior with an inert ROS subscription."""

import json
import sys
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from robot_safety.read_hardware_status import main, read_status


@pytest.fixture
def ros(monkeypatch):
    node = Mock()
    api = Mock(create_node=Mock(return_value=node))
    callback = []
    node.create_subscription.side_effect = lambda kind, topic, fn, depth: callback.append(fn)
    monkeypatch.setitem(sys.modules, 'rclpy', api)
    monkeypatch.setitem(sys.modules, 'std_msgs', SimpleNamespace())
    monkeypatch.setitem(sys.modules, 'std_msgs.msg', SimpleNamespace(String=object))
    return api, node, callback


def test_reads_complete_json_without_publishers(ros):
    api, node, callback = ros
    status = dict(startup_balance_required=True, armed=False, abort_reason=None,
                  hardware_profile_path='/a long path/' + 'x' * 300)
    api.spin_once.side_effect = lambda *a, **kw: callback[0](SimpleNamespace(data=json.dumps(status)))
    assert read_status() == status
    node.create_publisher.assert_not_called()
    node.create_client.assert_not_called()
    node.destroy_node.assert_called_once()
    api.shutdown.assert_called_once()


def test_timeout_cleans_up_and_does_not_publish(ros, monkeypatch):
    api, node, _ = ros
    times = iter([0.0, 0.1, 3.1])
    monkeypatch.setattr('robot_safety.read_hardware_status.time.monotonic', lambda: next(times))
    with pytest.raises(TimeoutError, match='No hardware status'):
        read_status()
    node.create_publisher.assert_not_called()
    node.destroy_node.assert_called_once()
    api.shutdown.assert_called_once()


@pytest.mark.parametrize('payload', ['not json', '[]'])
def test_malformed_status_is_reported(ros, capsys, payload):
    api, _, callback = ros
    api.spin_once.side_effect = lambda *a, **kw: callback[0](SimpleNamespace(data=payload))
    assert main() == 1
    output = capsys.readouterr()
    assert not output.out
    assert 'Cannot read safety-service status:' in output.err
    api.shutdown.assert_called_once()
