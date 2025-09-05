#!/usr/bin/env bash
set -e
# Allow container GUI apps to use host X11
xhost +si:localuser:$USER 1>/dev/null 2>&1 || true
docker compose run --rm tiangong bash

