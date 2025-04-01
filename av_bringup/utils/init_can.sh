#!/bin/bash

sudo pkill -f slcand

tty_device=$(ls /dev/ttyACM* | head -n 1)
if [ -z "$tty_device" ]; then
    echo "No ttyACM device found."
    exit 1
fi
echo "Using tty device: $tty_device"

sudo slcand -o -c -s8 $tty_device
sudo ip link set can0 up type can bitrate 1000000
sudo ip link show can0

