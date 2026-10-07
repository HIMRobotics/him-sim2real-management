"""Encode arm SDK frames and request stock-controller whole-body damping."""

import json
import time
from threading import Lock, Thread

from .hardware_constants import (
    ARM_INDICES,
    ARM_SDK_TOPIC,
    ARM_WEIGHT_INDEX,
    DAMPING_FSM_ID,
    ENGLISH_SPEAKER_ID,
    LOW_COMMAND_TOPIC,
    LOW_STATE_TOPIC,
    LOWER_BODY_INDICES,
    MOTION_SWITCHER_SET_SILENT,
    NUM_JOINTS,
    SDK_RPC_TIMEOUT_CODE,
)
from .hardware_profile import (
    DAMPING_RPC_TIMEOUT_SECS,
    READINESS_QUERY_ATTEMPTS,
    READINESS_RPC_TIMEOUT_SECS,
)

SOFT_RESET_PHRASE = "Soft Reset"


class MotorWriterLock:
    """Exclude other repository motor writers on this host."""

    def __init__(self, path="/tmp/g1-motor-writer.lock"):
        import fcntl
        import os

        self.fd = os.open(path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
        try:
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            self.close()
            raise RuntimeError("another G1 motor writer is active") from None

    def close(self):
        import os

        if self.fd is not None:
            os.close(self.fd)
            self.fd = None

    def __del__(self):
        if getattr(self, "fd", None) is not None:
            self.close()


class UnitreeArmTransport:
    """Own arm_sdk publication and a separate locomotion RPC client."""

    def __init__(self, interface, domain, on_state, on_battery):
        self.writer_lock = MotorWriterLock()
        self.write_lock = Lock()
        from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import (
            MotionSwitcherClient,
        )
        from unitree_sdk2py.core.channel import (
            ChannelFactoryInitialize,
            ChannelPublisher,
            ChannelSubscriber,
        )
        from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import BmsState_, LowCmd_, LowState_
        from unitree_sdk2py.utils.crc import CRC

        ChannelFactoryInitialize(domain, interface)
        self.make_command = unitree_hg_msg_dds__LowCmd_
        self.crc = CRC()
        self.publisher = ChannelPublisher(ARM_SDK_TOPIC, LowCmd_)
        self.publisher.Init()
        self.lowcmd = ChannelPublisher(LOW_COMMAND_TOPIC, LowCmd_)
        self.lowcmd.Init()
        self.audio = None
        self.saved_motion_name = None
        self.motion_restored = False
        self.locomotion = LocoClient()
        self.locomotion.SetTimeout(DAMPING_RPC_TIMEOUT_SECS)
        self.locomotion.Init()
        # A slow read-only startup query must not change stop-worker RPC timing.
        self.readiness_client = LocoClient()
        self.readiness_client.SetTimeout(READINESS_RPC_TIMEOUT_SECS)
        self.readiness_client.Init()
        self.readiness_query_results = []
        # Each SDK client constructor includes a 0.5 s discovery wait.
        self.switcher = MotionSwitcherClient()
        self.switcher.SetTimeout(5.0)
        self.switcher.Init()
        try:
            self._audio()
        except Exception as error:  # noqa: BLE001
            print(f"Optional speech initialization failed: {error}", flush=True)
        self.subscriber = ChannelSubscriber(LOW_STATE_TOPIC, LowState_)
        # No SDK queue that could conceal old samples behind new receipt timestamps.
        self.subscriber.Init(on_state)
        self.bms_subscriber = ChannelSubscriber("rt/lf/bmsstate", BmsState_)
        self.bms_subscriber.Init(on_battery)

    def write_arm(self, targets, weight):
        """Send only explicitly mapped upper-body targets, plus SDK ownership weight."""
        message = self.make_command()
        for index, values in targets.items():
            motor = message.motor_cmd[index]
            for field, value in values.items():
                setattr(motor, field, value)
        message.motor_cmd[ARM_WEIGHT_INDEX].q = weight
        message.crc = self.crc.Crc(message)
        with self.write_lock:
            return self.publisher.Write(message)

    def release_motion(self):
        """Hand the motors to rt/lowcmd. The sport service will not damp until restored."""
        switcher = self.switcher
        _code, mode = switcher.CheckMode()
        if isinstance(mode, dict) and mode.get("name"):
            self.saved_motion_name = mode["name"]
        self._set_motion_silent(True)
        release_code, _ = switcher.ReleaseMode()
        self.motion_restored = False
        if release_code == 0:
            # Speech must not block the first balance command.
            self._speech_thread = Thread(target=self._say_soft_reset, daemon=True)
            self._speech_thread.start()
        else:
            self._set_motion_silent(False)
        return release_code

    def restore_motion(self):
        """Reselect the sport service after a release. 7002 means it is still busy."""
        if self.motion_restored:
            return 0
        self._set_motion_silent(False)
        switcher = self.switcher
        switcher.SetTimeout(5.0)
        name = self.saved_motion_name or "ai"
        code, _ = switcher.SelectMode(name)
        if code == 0:
            self.motion_restored = True
        return code

    def write_lower_body(self, lower_q, arm_q, leg_kp, leg_kd, machine):
        """Command 15 lower-body targets and hold the captured arm pose."""
        targets = {}
        for index in range(NUM_JOINTS):
            lower = index in LOWER_BODY_INDICES
            targets[index] = {
                "q": float(lower_q[index] if lower else arm_q[index - ARM_INDICES[0]]),
                "dq": 0.0, "tau": 0.0,
                "kp": float(leg_kp[index]) if lower else 40.0,
                "kd": float(leg_kd[index]) if lower else 1.0,
            }
        return self.write_body(targets, machine)

    def write_body(self, targets, machine):
        """Encode a complete, already checked 29-joint command."""
        if set(targets) != set(range(NUM_JOINTS)):
            raise ValueError("whole-body output requires all 29 joints")
        message = self.make_command()
        message.mode_pr = 0
        message.mode_machine = machine
        for index, values in targets.items():
            motor = message.motor_cmd[index]
            motor.mode = 1
            for field in ("q", "dq", "kp", "kd", "tau"):
                setattr(motor, field, float(values[field]))
        message.crc = self.crc.Crc(message)
        with self.write_lock:
            return self.lowcmd.Write(message)

    def _set_motion_silent(self, silent):
        """Mute the firmware mode announcement. ReleaseMode otherwise says developer mode."""
        try:
            switcher = self.switcher
            switcher.SetTimeout(0.2)
            switcher._Call(MOTION_SWITCHER_SET_SILENT, json.dumps({"silent": bool(silent)}))
            switcher.SetTimeout(5.0)
        except Exception:  # noqa: BLE001
            return

    def _say_soft_reset(self):
        """Speak after the stock line is muted. A voice failure must not block the release."""
        try:
            audio = self._audio()
            audio.tts_index = max(audio.tts_index, 1)
            audio.TtsMaker(SOFT_RESET_PHRASE, ENGLISH_SPEAKER_ID)
        except Exception:  # noqa: BLE001
            return

    def _audio(self):
        if getattr(self, "audio", None) is None:
            from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient

            self.audio = AudioClient()
            self.audio.SetTimeout(3.0)
            self.audio.Init()
        return self.audio

    def request_damping(self):
        """Use Damp's underlying RPC because Damp() discards its return code."""
        return self.locomotion.SetFsmId(DAMPING_FSM_ID)

    def read_ready_fsm(self):
        """Retry one timed-out startup query, requiring a real successful reply."""
        self.readiness_query_results = []
        for _ in range(READINESS_QUERY_ATTEMPTS):
            started = time.monotonic()
            code, fsm_id = self.readiness_client.GetFsmId()
            self.readiness_query_results.append(
                {
                    "rpc_code": code,
                    "fsm_id": fsm_id,
                    "elapsed_secs": round(time.monotonic() - started, 3),
                }
            )
            if code != SDK_RPC_TIMEOUT_CODE:
                return code, fsm_id
        return code, fsm_id

    def read_fsm(self):
        return self.locomotion.GetFsmId()

    def close(self):
        self.subscriber.Close()
        self.bms_subscriber.Close()
        self.publisher.Close()
        self.lowcmd.Close()
        self.writer_lock.close()
