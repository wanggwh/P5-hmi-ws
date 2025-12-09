#!/bin/bash
echo "Remember to run docker compose up alone the first time"
docker compose up -d
xhost +local:root
sleep 0.1
docker exec -it p5-hmi-ws-p5_hmi-1 bash
source kivy_venv/bin/activate

