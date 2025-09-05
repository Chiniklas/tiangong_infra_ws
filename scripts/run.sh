#!/usr/bin/env bash
set -e
xhost +si:localuser:$USER 1>/dev/null 2>&1 || true
docker compose run --rm tiangong bash

