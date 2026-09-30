#!/usr/bin/env bash
set -e
source /opt/ros/foxy/setup.bash
source /example/install/setup.bash
exec python3 /example/client.py
