# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

# Simple demo of the TSL2591 sensor.  Will print the detected light value
# every second.
import time

import board

import adafruit_tsl2591

# Create sensor object, communicating over the board's default I2C bus
i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller

# Initialize the sensor.
sensor = adafruit_tsl2591.TSL2591(i2c)
# Read the total lux, IR, and visible light levels and print it every second.
while True:
    # Read and calculate the light level in lux.
    lux = sensor.lux
    print(f"Total light: {lux}lux")
    # You can also read the raw infrared and visible light levels.
    # These are unsigned, the higher the number the more light of that type.
    # There are no units like lux.
    # Infrared levels range from 0-65535 (16-bit)
    infrared = sensor.infrared
    print(f"Infrared light: {infrared}")
    # Visible-only levels range from 0-2147483647 (32-bit)
    visible = sensor.visible
    print(f"Visible light: {visible}")
    # Full spectrum (visible + IR) also range from 0-2147483647 (32-bit)
    full_spectrum = sensor.full_spectrum
    print(f"Full spectrum (IR + visible) light: {full_spectrum}")
    time.sleep(1.0)
