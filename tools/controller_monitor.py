"""Read-only SDK monitor for Start button edges and double-press events."""

import argparse
import queue
import time
from threading import Event

from robot_safety.config import Config
from robot_safety.controller import StartDoublePressDetector, decode_start_pressed


def main():
    """Subscribe to G1 state without creating publishers or motion clients."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", required=True)
    parser.add_argument("--domain", type=int, default=0)
    parser.add_argument("--topic", default="rt/lowstate")
    parser.add_argument(
        "--window-secs", type=float, default=Config().double_press_window_secs
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="show button bytes, sample gaps and press timing",
    )
    args = parser.parse_args()

    # Lazy imports let help and offline tests run without the native SDK.
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

    config = Config(double_press_window_secs=args.window_secs)
    detector = StartDoublePressDetector(
        config.double_press_window_secs, config.robot_timeout
    )
    input_fault = Event()
    samples = queue.Queue(maxsize=config.observation_history_size)

    def receive(message):
        try:
            samples.put_nowait(
                (time.monotonic(), message.tick, bytes(message.wireless_remote))
            )
        except (queue.Full, TypeError, ValueError):
            input_fault.set()

    print(
        "Read-only monitor. No motor commands. Stock firmware may react to Start.",
        flush=True,
    )
    print("Release Start, then press-release-press. Ctrl+C exits.", flush=True)
    ChannelFactoryInitialize(args.domain, args.interface)
    subscriber = ChannelSubscriber(args.topic, LowState_)
    subscriber.Init(receive, config.observation_history_size)
    time_last_debug_secs = None
    previous_button_bytes = None
    previous_pressed = None
    stale_reported = False
    try:
        while True:
            if input_fault.is_set():
                input_fault.clear()
                detector.reset()
                previous_pressed = None
                while True:
                    try:
                        samples.get_nowait()
                    except queue.Empty:
                        break
                print("Input dropped. Gesture reset, release Start again.", flush=True)
            try:
                received_at, tick, payload = samples.get(timeout=config.robot_timeout)
            except queue.Empty:
                detector.reset()
                previous_pressed = None
                if not stale_reported:
                    print(
                        "No fresh state. Gesture reset. Radio health is unknown.",
                        flush=True,
                    )
                    stale_reported = True
                continue
            if time.monotonic() - received_at >= config.robot_timeout:
                detector.reset()
                if args.debug:
                    print("Gesture reset: queued sample exceeded 200 ms.", flush=True)
                continue
            # Robot tick semantics are unverified, so do not filter by tick here.
            button_bytes = payload[2:4]
            sample_gap = (
                None
                if detector.time_last_sample_secs is None
                else received_at - detector.time_last_sample_secs
            )
            if args.debug and (
                button_bytes != previous_button_bytes
                or time_last_debug_secs is None
                or received_at - time_last_debug_secs >= 1.0
            ):
                print(
                    f"tick={tick} buttons={button_bytes.hex()} sample_gap_secs={sample_gap}",
                    flush=True,
                )
                time_last_debug_secs = received_at
                previous_button_bytes = button_bytes
            stale_reported = False
            try:
                pressed = decode_start_pressed(payload)
                if args.debug:
                    if sample_gap is not None and sample_gap >= config.robot_timeout:
                        print(
                            "Gesture reset: controller sample gap exceeded timeout.",
                            flush=True,
                        )
                    if pressed and detector.previous_start_pressed is False:
                        first_press = detector.time_first_press_secs
                        interval = (
                            None if first_press is None else received_at - first_press
                        )
                        print(
                            f"Start rising edge: interval_secs={interval}, window_secs={config.double_press_window_secs}",
                            flush=True,
                        )
                stop_requested = detector.update(pressed, received_at)
            except ValueError as error:
                detector.reset()
                print(f"Invalid controller sample: {error}", flush=True)
                continue
            if pressed != previous_pressed:
                print(f"Start {'pressed' if pressed else 'released'}", flush=True)
                previous_pressed = pressed
            if stop_requested:
                print(
                    "STOP REQUESTED: Start double press detected. No command sent.",
                    flush=True,
                )
    except KeyboardInterrupt:
        print("Monitor stopped.", flush=True)
    finally:
        subscriber.Close()


if __name__ == "__main__":
    main()
