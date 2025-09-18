#!/bin/bash
set -e

SRC="./vision.service"
DST="/etc/systemd/system/vision.service"

if [ ! -f "$SRC" ]; then
  echo "找不到 $SRC，退出" >&2
  exit 1
fi

sudo cp "$SRC" "$DST"
sudo chmod 644 "$DST"
sudo systemctl daemon-reload
sudo systemctl restart vision.service
sudo systemctl status vision.service --no-pager
