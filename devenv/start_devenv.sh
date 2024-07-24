#!/bin/bash
set -e
export MY_UID=$(id -u)
export MY_GID=$(id -g)
export MY_USER=${USER}
docker compose down
docker compose up --build -d --wait
