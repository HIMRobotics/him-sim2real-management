# Example policy

`examples/` is one policy bundle: a YAML file plus the ONNX files that YAML names. The network here is a Unitree G1 walker. Its contract id is `g1-walker-99-v1`. Another policy replaces these files. It does not get its own subdirectory.

| File | Role |
| --- | --- |
| `policy.yaml` | Observation layout, joint order, defaults, action scales, gains, timing |
| `walker.onnx` | 99-input, 29-output network |
| `walker.onnx.data` | External weights. Keep this beside `walker.onnx`. |

`walker.onnx` and `walker.onnx.data` are a third-party locomotion prior from [luckyrobots/g1-manipulation-challenge](https://github.com/luckyrobots/g1-manipulation-challenge). See that repository for license terms.

This folder does not launch the robot. `policy-safety prepare-g1` reads `policy.yaml`, attaches this repo's MuJoCo G1, and writes `robot.json` for screening. Contract notes are in [ONNX_BUNDLE.md](../ONNX_BUNDLE.md).

```bash
policy-safety prepare-g1 \
  --submission examples \
  --output /tmp/g1-screening-bundle
```
