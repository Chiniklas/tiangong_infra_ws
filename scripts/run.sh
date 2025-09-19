#!/usr/bin/env bash
set -e

# Allow container GUI apps to use host X11
xhost +si:localuser:$USER 1>/dev/null 2>&1 || true

# Start a persistent container named "tiangong"
docker compose run --name tiangong_dev tiangong bash

