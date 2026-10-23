#!/usr/bin/env bash
set -e
source /opt/ros/foxy/setup.bash
source /submission/install/setup.bash
exec python3 /submission/node.py
