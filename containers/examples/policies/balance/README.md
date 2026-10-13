# GR00T balance model

`GR00T-WholeBodyControl-Balance.onnx` is the standing-balance network, separate
from the standalone walking checkpoint. The NVIDIA walker bundle also includes this Balance checkpoint for near-zero commands. The shared adapter in `robot_safety/balance_policy.py`
uses six 86-value observation frames and returns 15 leg/waist targets. The
simulation example holds the remaining arm joints at their initial pose.

`./sim.sh --example balance` uses this committed model by default.
`HIM_BALANCE_ONNX` can select another copy. The hardware safety service continues
to use its separately configured cache path.
