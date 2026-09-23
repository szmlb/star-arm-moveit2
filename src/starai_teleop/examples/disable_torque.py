#!/usr/bin/env python3
"""Release a StarAI arm's servo torque directly via the SDK, WITHOUT touching
the multi-turn zero reference (unlike relaunching robo_driver, which always
resets it on init). Use this when you want the arm to go limp/free-movable
without disturbing whatever zero calibration is currently in effect -- e.g.
before hand-posing the arm to capture a rest pose (see
docs/starai-arm-ros2.md, "レスト姿勢の取得").

Usage: /usr/bin/python3 disable_torque.py [port]   (default: /dev/ttyUSB0)
"""
import sys
import time

import fashionstar_uart_sdk as uservo
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
BAUDRATE = 1000000

uart = serial.Serial(
    port=PORT, baudrate=BAUDRATE, parity=serial.PARITY_NONE,
    stopbits=1, bytesize=8, timeout=0,
)
manager = uservo.UartServoManager(uart)
manager.disable_torque(0xFF)
time.sleep(0.1)
uart.close()
print(f"Done: torque released on {PORT}, zero reference untouched.")
