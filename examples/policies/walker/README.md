# Walker model bundle

`walker.onnx`, `walker.onnx.data`, and `walker.yaml` form one policy bundle.
Keep the external weights beside the ONNX graph. The network consumes 99 values
and produces 29 raw joint actions, decoded using the YAML specification.

The ONNX graph and weights were retained from this repository's
`examples/sample_submissions/unitree_g1_stairs/policies` before commit `d541eb4`.
The complete contract is documented in [ONNX_BUNDLE.md](../../../ONNX_BUNDLE.md).

The graph uses ONNX IR 9 and opset 17 for the Python 3.8 ARM64 container's
ONNX Runtime 1.16.3. The original IR 10 export was resaved after removing its
`pkg.onnxscript.optimizer.folded_from` value-info annotation. Computation,
tensor values, and the external weights are unchanged.

Velocity commands live in `examples/commands/walker/`. The submission client
lives in `examples/submissions/walker/`. This directory contains no launch code.

From the repository root, prepare an offline screening bundle with:

```bash
policy-safety prepare-g1 --submission examples/policies/walker --output /tmp/g1-screening-bundle
```
