#!/usr/bin/env bash

ip link set can0 up type can bitrate 500000
sleep 2
ip link set can0 up
