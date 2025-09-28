#!/bin/bash
if mount | grep -q "/home/at_vision"; then
    echo "[OK] /home/at_vision 已挂载："
    mount | grep "/home/at_vision"
else
    echo "[WARN] /home/at_vision 没有挂载"
fi