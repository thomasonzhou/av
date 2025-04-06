#!/bin/bash

sudo pkill -f slcand

tty_device=$(ls /dev/ttyACM* | head -n 1)
if [ -z "$tty_device" ]; then
    echo "No ttyACM device found."
    exit 1
fi
echo "Using tty device: $tty_device"

# jetson uses can0 and and can1 by default,set to can3 to avoid conflicts
can_interface=can3

sudo slcand -o -c -s8 $tty_device $can_interface
sudo ip link set $can_interface up type can bitrate 1000000
sudo ip link show $can_interface