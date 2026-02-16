#!/bin/bash
# Script to start the Micro-XRCE-DDS Agent
#!/usr/bin/env bash


# ==============================
# ---- FOR SIMULATION (UDP) ----
# ==============================

echo "Starting Micro XRCE-DDS Agent on UDP port 8888..."
MicroXRCEAgent udp4 -p 8888



# ==============================
# ---- FOR REAL DRONE (SERIAL) -
# ==============================
# Uncomment below and comment UDP above when using real hardware
#
# MicroXRCEAgent serial --dev /dev/ttyACM0 -b 921600
#
# If using Jetson UART:
# MicroXRCEAgent serial --dev /dev/ttyTHS1 -b 921600