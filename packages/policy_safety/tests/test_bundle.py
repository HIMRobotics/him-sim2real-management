"""Policy selection alongside velocity-command YAML files."""

import shutil
from pathlib import Path

import pytest
from policy_safety.bundle import read_policy_yaml

EXAMPLES = Path(__file__).resolve().parents[3] / "examples"
WALKER = EXAMPLES / "policies/walker"


def test_walker_policy_is_selected_alongside_command_files(tmp_path):
    for source in WALKER.iterdir():
        if source.suffix in (".yaml", ".onnx"):
            shutil.copy2(source, tmp_path / source.name)
    shutil.copy2(EXAMPLES / "commands/walker/stand.yaml", tmp_path / "stand.yaml")
    config, model = read_policy_yaml(tmp_path)
    assert config["contract"] == "nvidia-decoupled-wbc-516-v1"
    assert model == tmp_path / "walker.onnx"


def test_multiple_policy_files_are_rejected(tmp_path):
    for name in ("walker.yaml", "policy.yaml"):
        shutil.copy2(WALKER / "walker.yaml", tmp_path / name)
    with pytest.raises(ValueError, match="exactly one policy YAML"):
        read_policy_yaml(tmp_path)


def test_command_files_alone_are_not_a_policy(tmp_path):
    shutil.copy2(EXAMPLES / "commands/walker/stand.yaml", tmp_path / "stand.yaml")
    with pytest.raises(ValueError, match="exactly one policy YAML"):
        read_policy_yaml(tmp_path)
