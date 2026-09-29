#!/usr/bin/env python3
# Copyright (C) 2023-2026 Ola Gatner
# SPDX-License-Identifier: AGPL-3.0-only OR LicenseRef-AmyCore-Commercial
# Bench test for the enable wave: sends T1 (wave without 24V relay), then K every 10 ms.
#   scope_test.py [port]            wave test
#   scope_test.py --relay [port]    toggle the 24V relay (P1/P0) every RELAY_PERIOD_S, wave off
#                                   ONLY with nothing on the relay contacts
# Reconnects (and sends T1 again) if the board resets or is replugged.
# Stop with Ctrl+C / kill -> the ESP32 watchdog stops the wave 50 ms later.
import serial, sys, time

args = sys.argv[1:]
relayMode = "--relay" in args
args = [a for a in args if a != "--relay"]
port = args[0] if args else "/dev/amy-esp32"
RELAY_PERIOD_S = 0.5


def run(s):
    time.sleep(0.2)
    s.reset_input_buffer()
    s.write(b"P0\r" if relayMode else b"T1\r")
    nextStatus = 0
    nextToggle = 0
    relayOn = False
    while True:
        s.write(b"K\r")
        now = time.monotonic()
        if relayMode and now >= nextToggle:
            relayOn = not relayOn
            s.write(b"P1\r" if relayOn else b"P0\r")
            nextToggle = now + RELAY_PERIOD_S
        if now >= nextStatus:
            s.write(b"S\r")
            nextStatus = now + 1.0
        data = s.read(512)
        if data:
            sys.stdout.write(data.decode(errors="replace"))
            sys.stdout.flush()
        time.sleep(0.01)


while True:
    s = None
    try:
        s = serial.Serial(port, 115200, timeout=0)
        print(f"connected {port}", flush=True)
        run(s)
    except KeyboardInterrupt:
        if s:
            s.write(b"P0\r" if relayMode else b"T0\r")
        break
    except (serial.SerialException, OSError) as e:
        print(f"lost {port}: {e}, retrying", flush=True)
        if s:
            try:
                s.close()
            except Exception:
                pass
        time.sleep(0.5)
