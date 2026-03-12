#!/usr/bin/env python3
import serial.tools.list_ports

ports = list(serial.tools.list_ports.comports())
if not ports:
    print("No serial ports found.")
else:
    for p in ports:
        print(f"device={p.device} description={p.description} hwid={p.hwid}")