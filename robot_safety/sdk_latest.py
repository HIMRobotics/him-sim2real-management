"""Take only the newest SDK sample, on the caller's schedule.

The G1 publishes ``rt/lowstate`` at 1 kHz. A Python callback decodes every
message under the interpreter lock: about 0.7 ms each on the Jetson, or
most of a core. That starves the policy step and the 2 ms command refresh.
Samples here stay serialized in DDS until ``take_latest`` decodes one.
"""

import threading
import time

from .timing_profile import profiled, scope, timed_wait


def _new_reader(topic, message_type, depth):
    from cyclonedds.qos import Policy, Qos
    from cyclonedds.sub import DataReader
    from cyclonedds.topic import Topic
    from unitree_sdk2py.core.channel import ChannelFactory

    factory = ChannelFactory()
    participant = getattr(type(factory), "_ChannelFactory__participant", None)
    if participant is None:
        raise RuntimeError("ChannelFactoryInitialize must run before reading " + topic)
    qos = Qos(Policy.History.KeepLast(depth))
    return DataReader(participant, Topic(participant, topic, message_type), qos)


def _invalid_sample_type():
    try:
        from cyclonedds.internal import InvalidSample
    except ImportError:
        return ()
    return InvalidSample


class LatestSdkReader:
    """Non-blocking reader that keeps only the newest sample."""

    def __init__(self, topic, message_type, depth=1, reader=None):
        self.topic = topic
        self.depth = depth
        self.reader = reader if reader is not None else _new_reader(topic, message_type, depth)

    @profiled("sdk_latest.take_latest")
    def take_latest(self):
        """Decode the newest waiting sample, or return None."""
        invalid = _invalid_sample_type()
        samples = self.reader.take(N=self.depth)
        for sample in reversed(samples or ()):
            if not isinstance(sample, invalid):
                return sample
        return None

    def close(self):
        reader = self.reader
        self.reader = None
        if reader is not None:
            del reader


def poll_deadline(next_tick, now, period):
    """Return the sleep and the next deadline. A late tick waits a full period."""
    delay = next_tick - now
    if delay <= 0:
        return period, now + period
    return delay, next_tick


class LatestSdkPoller:
    """Hand the newest sample to ``handler`` once per ``period``."""

    def __init__(self, reader, handler, period):
        self.reader = reader
        self.handler = handler
        self.period = period
        self.stopped = threading.Event()
        self.thread = threading.Thread(target=self._run, name="lowstate_poll", daemon=True)
        self.diagnostics_enabled = False
        self.diagnostics = {}

    def start(self):
        self.thread.start()

    def stop(self):
        self.stopped.set()
        if self.thread.is_alive() and threading.current_thread() is not self.thread:
            self.thread.join(timeout=1.0)

    def _run(self):
        next_tick = time.monotonic()
        while not self.stopped.is_set():
            diagnostic = self.diagnostics_enabled
            if diagnostic:
                take_started = time.monotonic()
            try:
                sample = self.reader.take_latest()
            except Exception as error:  # noqa: BLE001 - feedback timeout handles silence
                print("{0} read failed: {1}".format(self.reader.topic, error), flush=True)
                sample = None
            if diagnostic:
                take_finished = time.monotonic()
                info = getattr(sample, "sample_info", None)
                self.diagnostics = {
                    "tick": getattr(sample, "tick", None),
                    "source_timestamp_ns": getattr(info, "source_timestamp", None),
                    "take_started_s": take_started,
                    "take_finished_s": take_finished,
                    "take_duration_s": take_finished - take_started,
                    "has_sample": sample is not None,
                }
            if sample is not None:
                with scope(self, "feedback_handler"):
                    self.handler(sample)
                if diagnostic:
                    finished = time.monotonic()
                    self.diagnostics = dict(
                        self.diagnostics,
                        handler_finished_s=finished,
                        handler_duration_s=finished - take_finished,
                    )
            deadline = next_tick + self.period
            delay, next_tick = poll_deadline(deadline, time.monotonic(), self.period)
            if timed_wait(self, self.stopped, delay, deadline, "feedback_timer"):
                return
