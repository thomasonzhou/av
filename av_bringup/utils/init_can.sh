#!/bin/bash

sudo pkill -f slcand

tty_device=$(ls /dev/ttyACM* | head -n 1)
if [ -z "$tty_device" ]; then
    echo "No ttyACM device found."
    exit 1
fi
echo "Using tty device: $tty_device"

# jetson uses can0 and and can1 by default,set to can1 to avoid conflicts with USB to CAN adapters
can_interface=can0

sudo slcand -o -c -s8 $tty_device $can_interface
sudo ip link set $can_interface up type can bitrate 1000000
sudo ip link show $can_interface