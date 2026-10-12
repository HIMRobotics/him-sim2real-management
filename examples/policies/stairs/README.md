# Walker model bundle

`walker.onnx`, `walker.onnx.data`, and `walker.yaml` form one policy bundle.
Keep the external weights beside the ONNX graph. The network consumes 99 values
and produces 29 raw joint actions, decoded using the YAML specification.

The ONNX graph and weights were retained from this repository's
`examples/sample_submissions/unitree_g1_stairs/policies` before commit `d541eb4`.
These weights are temporary. A yaw command of 0 still turns: walking forward at
0.5 m/s yaws about +0.08 rad/s. A yaw command of about −0.127 rad/s cancels that
turn in this sim. The neural contract is described in the
[offline screening guide](../../../packages/policy_safety/README.md#what-the-first-adapter-tests).

The graph uses ONNX IR 9 and opset 17 for the Python 3.8 ARM64 container's
ONNX Runtime 1.16.3. The original IR 10 export was resaved after removing its
`pkg.onnxscript.optimizer.folded_from` value-info annotation. Computation,
tensor values, and the external weights are unchanged.

The shared client and scripted stair controller live in
[containers/examples](../../../containers/examples/README.md). Run this bundle
with `./submit.sh --mode sim --example stairs`. This directory contains no launch code.

From the repository root, prepare an offline screening bundle with:

```bash
policy-safety prepare-g1 --submission examples/policies/stairs --output /tmp/g1-screening-bundle
```

This legacy 99-input bundle is used only by `--example stairs` and legacy
adapter contract tests. The official NVIDIA walker is in `../walker/`.
