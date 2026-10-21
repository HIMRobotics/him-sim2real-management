"""Encode arm SDK frames and request stock-controller whole-body damping."""

import json
import time
from threading import Lock, Thread

from .hardware_constants import (
    ARM_SDK_TOPIC,
    ARM_WEIGHT_INDEX,
    DAMPING_FSM_ID,
    ENGLISH_SPEAKER_ID,
    LOW_COMMAND_TOPIC,
    LOW_STATE_TOPIC,
    MOTION_SWITCHER_SET_SILENT,
    NUM_JOINTS,
    SDK_RPC_TIMEOUT_CODE,
)
from .hardware_profile import (
    DAMPING_RPC_TIMEOUT_SECS,
    READINESS_QUERY_ATTEMPTS,
    READINESS_RPC_TIMEOUT_SECS,
    STATE_POLL_SECS,
)
from .sdk_latest import LatestSdkPoller, LatestSdkReader

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

    def __init__(self, interface, domain, on_state, on_battery, *, output_enabled=True):
        self.output_enabled = output_enabled
        self.writer_lock = MotorWriterLock() if output_enabled else None
        self.write_lock = Lock()
        self.diagnostics_enabled = False
        self._body_diagnostics = {}
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
        self.publisher = self.lowcmd = None
        if output_enabled:
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
        self._body_targets = None
        self._body_message = None
        self._body_ready = False
        self.bms_subscriber = ChannelSubscriber("rt/lf/bmsstate", BmsState_)
        self.bms_subscriber.Init(on_battery)
        # Newest sample only, on a fixed schedule. A per-message callback
        # decodes 1 kHz in Python and starves the policy and 2 ms threads.
        self.state_reader = LatestSdkReader(LOW_STATE_TOPIC, LowState_)
        self.state_poller = LatestSdkPoller(self.state_reader, on_state, STATE_POLL_SECS)
        self.state_poller.start()

    def write_arm(self, targets, weight):
        """Send only explicitly mapped upper-body targets, plus SDK ownership weight."""
        if not self.output_enabled:
            return True
        message = self.make_command()
        for index, values in targets.items():
            motor = message.motor_cmd[index]
            for field, value in values.items():
                setattr(motor, field, value)
        message.motor_cmd[ARM_WEIGHT_INDEX].q = weight
        message.crc = self.crc.Crc(message)
        with self.write_lock:
            return self.publisher.Write(message)

    def release_motion(self, *, announce_soft_reset=False):
        """Hand the motors to rt/lowcmd. The sport service will not damp until restored."""
        if not self.output_enabled:
            return 0
        switcher = self.switcher
        _code, mode = switcher.CheckMode()
        if isinstance(mode, dict) and mode.get("name"):
            self.saved_motion_name = mode["name"]
        self._set_motion_silent(True)
        release_code, _ = switcher.ReleaseMode()
        self.motion_restored = False
        if release_code == 0 and announce_soft_reset:
            # Speech must not block the first balance command.
            self._speech_thread = Thread(target=self._say_soft_reset, daemon=True)
            self._speech_thread.start()
        elif release_code != 0:
            self._set_motion_silent(False)
        return release_code

    def restore_motion(self):
        """Reselect the sport service after a release. 7002 means it is still busy."""
        if not self.output_enabled:
            return 0
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

    def write_body(self, targets, machine):
        """Encode a complete, already checked 29-joint command."""
        if set(targets) != set(range(NUM_JOINTS)):
            raise ValueError("whole-body output requires all 29 joints")
        if not self.output_enabled:
            return True
        stored = {
            index: {field: float(values[field]) for field in ("q", "dq", "kp", "kd", "tau")}
            for index, values in targets.items()
        }
        message = self._encode_body(stored, machine)
        waiting_since = time.monotonic() if getattr(self, "diagnostics_enabled", False) else None
        with self.write_lock:
            acquired_at = time.monotonic() if waiting_since is not None else None
            self._body_targets = stored
            self._body_message = message
            self._body_ready = True
            return self._write_body_message(message, "fresh", waiting_since, acquired_at)

    def repeat_body(self, machine):
        """Resend the latched whole-body command with a fresh mode and CRC.

        Returns None when no command is latched. Joint targets stay the ones
        write_body checked. This does not rebuild the motor command or run
        another safety check, so the 2 ms thread stays off the policy tick.
        """
        if not self.output_enabled:
            return True
        waiting_since = time.monotonic() if getattr(self, "diagnostics_enabled", False) else None
        with self.write_lock:
            acquired_at = time.monotonic() if waiting_since is not None else None
            message = getattr(self, "_body_message", None)
            if not getattr(self, "_body_ready", False) or message is None:
                return None
            message.mode_machine = machine
            message.crc = self.crc.Crc(message)
            if not self._body_ready:
                return None
            return self._write_body_message(message, "repeat", waiting_since, acquired_at)

    def clear_body(self):
        """Drop the latched whole-body command so the 2 ms thread stops."""
        with self.write_lock:
            self._body_ready = False
            if getattr(self, "diagnostics_enabled", False):
                previous = self._body_diagnostics
                self._body_diagnostics = dict(
                    previous, clear_count=previous.get("clear_count", 0) + 1,
                    last_clear_s=time.monotonic(), ready=False,
                )

    def set_diagnostics_enabled(self, enabled):
        """Enable local timing snapshots without publishing extra messages."""
        with self.write_lock:
            if enabled and not getattr(self, "diagnostics_enabled", False):
                self._body_diagnostics = {}
            self.diagnostics_enabled = bool(enabled)
        poller = getattr(self, "state_poller", None)
        if poller is not None:
            poller.diagnostics_enabled = bool(enabled)

    def diagnostics_snapshot(self):
        """Copy immutable write and reader timing snapshots."""
        poller = getattr(self, "state_poller", None)
        return {
            "body": dict(getattr(self, "_body_diagnostics", {})),
            "state_poller": dict(getattr(poller, "diagnostics", {})),
        }

    def _write_body_message(self, message, kind, waiting_since, acquired_at):
        """Write under the caller's lock, optionally recording local timing."""
        if waiting_since is None:
            return self.lowcmd.Write(message)
        started = time.monotonic()
        succeeded = False
        try:
            result = self.lowcmd.Write(message)
            succeeded = bool(result)
            return result
        finally:
            finished = time.monotonic()
            previous = self._body_diagnostics
            generation = previous.get("generation", 0) + int(kind == "fresh")
            update = dict(
                previous,
                generation=generation,
                write_count=previous.get("write_count", 0) + 1,
                success_count=previous.get("success_count", 0) + int(succeeded),
                fresh_count=previous.get("fresh_count", 0) + int(kind == "fresh"),
                repeat_count=previous.get("repeat_count", 0) + int(kind == "repeat"),
                last_write_kind=kind,
                last_write_started_s=started,
                last_write_finished_s=finished,
                last_write_duration_s=finished - started,
                last_lock_wait_s=acquired_at - waiting_since,
                last_lock_held_s=finished - acquired_at,
                max_lock_wait_s=max(previous.get("max_lock_wait_s", 0), acquired_at - waiting_since),
                max_write_duration_s=max(previous.get("max_write_duration_s", 0), finished - started),
                last_write_success=succeeded,
                ready=self._body_ready,
            )
            if succeeded:
                update["last_success_s"] = finished
                update["last_success_generation"] = generation
                if kind == "fresh":
                    update["last_fresh_success_s"] = finished
            self._body_diagnostics = update

    def _encode_body(self, targets, machine):
        message = self.make_command()
        message.mode_pr = 0
        message.mode_machine = machine
        for index, values in targets.items():
            motor = message.motor_cmd[index]
            motor.mode = 1
            for field in ("q", "dq", "kp", "kd", "tau"):
                setattr(motor, field, float(values[field]))
        message.crc = self.crc.Crc(message)
        return message

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
        if not self.output_enabled:
            return 0
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
        self.state_poller.stop()
        self.state_reader.close()
        self.bms_subscriber.Close()
        if self.output_enabled:
            self.publisher.Close()
            self.lowcmd.Close()
            self.writer_lock.close()
