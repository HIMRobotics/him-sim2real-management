#!/usr/bin/env python3
"""Set the G1 speaker volume (0-100)."""

import argparse
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--volume", type=int, default=20, help="target percent, 0-100")
    parser.add_argument("--interface", default="eth0")
    parser.add_argument("--domain", type=int, default=0)
    args = parser.parse_args()
    if not 0 <= args.volume <= 100:
        sys.exit("volume must be an integer from 0 to 100")

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient

    ChannelFactoryInitialize(args.domain, args.interface)
    audio = AudioClient()
    audio.SetTimeout(3.0)
    audio.Init()

    code, data = audio.GetVolume()
    if code != 0:
        sys.exit(f"GetVolume failed: {code}")
    current = data.get("volume") if isinstance(data, dict) else data
    print(f"current volume: {current}", flush=True)

    code = audio.SetVolume(args.volume)
    if code != 0:
        sys.exit(f"SetVolume failed: {code}")

    code, data = audio.GetVolume()
    if code != 0:
        sys.exit(f"volume write returned 0, but readback failed: {code}")
    print(f"volume now: {data}", flush=True)


if __name__ == "__main__":
    main()
