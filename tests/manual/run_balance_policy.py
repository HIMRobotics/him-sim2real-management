#!/usr/bin/env python3
"""Try the Gear WBC standing network on the lower body.

With no flag this only reads ``rt/lowstate`` and prints the first warmed
targets. ``--apply`` holds the measured pose, releases the stock motion
service, blends to those targets, and keeps publishing them until double
Start or Ctrl+C. It then reselects the sport service and enters stock damp.

Double Start during ``--apply`` does that restore immediately. Leave the
hardware adapter running so its double Start still calls ``SetFsmId(DAMPING_FSM_ID)`` as
well.
"""

import argparse
import sys
import threading
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from robot_safety.balance_policy import (  # noqa: E402
    LEG_KD,
    LEG_KP,
    NUM_ACTIONS,
    NUM_JOINTS,
    BalancePolicy,
)
from robot_safety.hardware_constants import (  # noqa: E402
    DAMPING_FSM_ID,
    LOW_COMMAND_TOPIC,
    LOW_STATE_TOPIC,
    LOWER_JOINT_NAMES,
)
from robot_safety.controller import (  # noqa: E402
    StartDoublePressDetector,
    decode_start_pressed,
)

MODEL_PATH = (
    Path(__file__).resolve().parents[2]
    / ".cache"
    / "groot_balance"
    / "GR00T-WholeBodyControl-Balance.onnx"
)
CONTROL_DT = 0.005
POLICY_EVERY = 4
WARMUP_SECS = 0.4
HOLD_AFTER_RELEASE_SECS = 0.5
BLEND_SECS = 3.0
ARM_KP = 40.0
ARM_KD = 1.0
MAX_RELEASE_DELTA_RAD = 1.0
MAX_JOINT_SPEED = 50.0


def _sample(message):
    motors = message.motor_state
    q = np.array([motors[i].q for i in range(NUM_JOINTS)], dtype=np.float32)
    dq = np.array([motors[i].dq for i in range(NUM_JOINTS)], dtype=np.float32)
    quat = np.array(message.imu_state.quaternion, dtype=np.float32)
    gyro = np.array(message.imu_state.gyroscope, dtype=np.float32)
    remote = [int(value) for value in message.wireless_remote]
    return q, dq, quat, gyro, remote, int(message.mode_machine)


def _print_delta(q, targets):
    delta = targets - q[:NUM_ACTIONS]
    print(f"max_abs_delta_rad {float(np.max(np.abs(delta))):.4f}")
    for name, measured, target, change in zip(
        LOWER_JOINT_NAMES, q[:NUM_ACTIONS], targets, delta
    ):
        print(
            f"  {name:20} q {measured:8.4f}  target {target:8.4f}  "
            f"delta {change:8.4f}"
        )
    return float(np.max(np.abs(delta)))


def _restore(switcher, locomotion, saved_name):
    """Reselect the sport service, then damp. The service needs time to return."""
    name = saved_name or "ai"
    switcher.SetTimeout(5.0)
    locomotion.SetTimeout(5.0)
    select_code = None
    for _ in range(5):
        try:
            select_code, _ = switcher.SelectMode(name)
        except Exception as exc:  # noqa: BLE001
            select_code = None
            print(f"restore_select failed {exc}")
        print(f"restore_select code {select_code} name {name}")
        if select_code == 0:
            break
        time.sleep(0.5)
    time.sleep(1.0)
    damp_code = None
    for _ in range(5):
        try:
            damp_code = locomotion.SetFsmId(DAMPING_FSM_ID)
        except Exception as exc:  # noqa: BLE001
            damp_code = None
            print(f"restore_damp failed {exc}")
        print(f"restore_damp code {damp_code}")
        if damp_code == 0:
            break
        time.sleep(0.5)
    try:
        print("fsm", locomotion.GetFsmId())
    except Exception as exc:  # noqa: BLE001
        print(f"fsm read failed {exc}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--apply", action="store_true")
    parser.add_argument(
        "--force",
        action="store_true",
        help="release even when a target is more than 1 rad from the measured pose",
    )
    parser.add_argument("--interface", default="eth0")
    parser.add_argument("--domain", type=int, default=0)
    args = parser.parse_args()
    if not MODEL_PATH.is_file():
        raise SystemExit(f"missing balance network at {MODEL_PATH}")

    from unitree_sdk2py.core.channel import (
        ChannelFactoryInitialize,
        ChannelPublisher,
        ChannelSubscriber,
    )
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
    from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import (
        MotionSwitcherClient,
    )
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
    from unitree_sdk2py.utils.crc import CRC

    latest = {}
    lock = threading.Lock()

    def on_state(message):
        with lock:
            latest["sample"] = _sample(message)

    ChannelFactoryInitialize(args.domain, args.interface)
    subscriber = ChannelSubscriber(LOW_STATE_TOPIC, LowState_)
    subscriber.Init(on_state, 10)
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline and "sample" not in latest:
        time.sleep(0.01)
    if "sample" not in latest:
        raise SystemExit("no lowstate")

    policy = BalancePolicy(str(MODEL_PATH))
    detector = StartDoublePressDetector(0.5, 0.5)
    warmed = None
    warmup_deadline = time.monotonic() + WARMUP_SECS
    next_policy = 0.0
    while time.monotonic() < warmup_deadline:
        now = time.monotonic()
        if now >= next_policy:
            with lock:
                sample = latest["sample"]
            q, dq, quat, gyro, remote, machine = sample
            if detector.update(decode_start_pressed(remote), now):
                raise SystemExit("double Start during warmup, nothing released")
            warmed = policy.update(q, dq, quat, gyro)
            next_policy = now + CONTROL_DT * POLICY_EVERY
        time.sleep(0.001)
    with lock:
        q, dq, quat, gyro, remote, machine = latest["sample"]
    print(f"mode_machine {machine}")
    delta = _print_delta(q, warmed)
    if not args.apply:
        subscriber.Close()
        return
    if delta > MAX_RELEASE_DELTA_RAD and not args.force:
        subscriber.Close()
        raise SystemExit(
            f"refusing release, max delta {delta:.3f} rad exceeds "
            f"{MAX_RELEASE_DELTA_RAD:.3f}"
        )
    if delta > MAX_RELEASE_DELTA_RAD:
        print(
            f"delta gate overridden, max delta {delta:.3f} rad; "
            "double Start restores stock damp"
        )

    from robot_safety.unitree_transport import MotorWriterLock

    writer_lock = MotorWriterLock()
    publisher = ChannelPublisher(LOW_COMMAND_TOPIC, LowCmd_)
    publisher.Init()
    crc = CRC()
    stop = threading.Event()
    phase = {"name": "hold", "since": time.monotonic(), "hold_q": q.copy()}
    reason = {"text": "completed"}

    def publish():
        tick = 0
        next_tick = time.monotonic()
        while not stop.is_set():
            next_tick += CONTROL_DT
            with lock:
                sample = latest.get("sample")
            if sample is None:
                continue
            measured, speeds, quat, gyro, remote, machine = sample
            now = time.monotonic()
            try:
                if detector.update(decode_start_pressed(remote), now):
                    reason["text"] = "double_start"
                    stop.set()
                    break
            except ValueError:
                pass
            if float(np.max(np.abs(speeds[:NUM_ACTIONS]))) > MAX_JOINT_SPEED:
                reason["text"] = "joint_speed"
                stop.set()
                break
            if tick % POLICY_EVERY == 0:
                try:
                    phase["targets"] = policy.update(measured, speeds, quat, gyro)
                except ValueError as exc:
                    reason["text"] = str(exc)
                    stop.set()
                    break
            targets = phase.get("targets")
            if targets is None:
                continue
            alpha = 0.0
            if phase["name"] == "blend":
                alpha = min(1.0, (now - phase["since"]) / BLEND_SECS)
            elif phase["name"] == "run":
                alpha = 1.0
            lower = (1.0 - alpha) * phase["hold_q"][:NUM_ACTIONS] + alpha * targets
            command = unitree_hg_msg_dds__LowCmd_()
            command.mode_pr = 0
            command.mode_machine = machine
            for index in range(NUM_JOINTS):
                motor = command.motor_cmd[index]
                motor.mode = 1
                motor.dq = 0.0
                motor.tau = 0.0
                if index < NUM_ACTIONS:
                    motor.q = float(lower[index])
                    motor.kp = float(LEG_KP[index])
                    motor.kd = float(LEG_KD[index])
                else:
                    motor.q = float(phase["hold_q"][index])
                    motor.kp = ARM_KP
                    motor.kd = ARM_KD
            command.crc = crc.Crc(command)
            publisher.Write(command)
            tick += 1
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)

    worker = threading.Thread(target=publish, daemon=True)
    worker.start()
    time.sleep(0.2)
    switcher = MotionSwitcherClient()
    switcher.SetTimeout(2.0)
    switcher.Init()
    locomotion = LocoClient()
    locomotion.SetTimeout(1.0)
    locomotion.Init()
    code, mode = switcher.CheckMode()
    saved_name = mode.get("name") if isinstance(mode, dict) else None
    print(f"motion_mode code {code} body {mode}")
    release_code, _ = switcher.ReleaseMode()
    print(f"release_code {release_code}")
    if release_code != 0:
        stop.set()
        worker.join(timeout=1.0)
        _restore(switcher, locomotion, saved_name)
        raise SystemExit("release failed, stock mode restored")
    try:
        time.sleep(HOLD_AFTER_RELEASE_SECS)
        phase["since"] = time.monotonic()
        phase["name"] = "blend"
        print("blending")
        time.sleep(BLEND_SECS)
        phase["since"] = time.monotonic()
        phase["name"] = "run"
        print("balancing until double Start")
        while not stop.wait(0.2):
            pass
    except KeyboardInterrupt:
        reason["text"] = "interrupt"
        stop.set()
    finally:
        stop.set()
        worker.join(timeout=1.0)
        print(f"stop_reason {reason['text']}")
        _restore(switcher, locomotion, saved_name)
        subscriber.Close()
        if not worker.is_alive():
            writer_lock.close()


if __name__ == "__main__":
    main()
