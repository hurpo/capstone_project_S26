#!/usr/bin/env python3
"""
Continuous-rotation servo control via PCA9685 on Raspberry Pi (I2C).
- Uses channel 0 (module output 0) by default.
- Terminal commands: center/calibrate, forward/backward with speed, stop, quit.

Wiring notes:
- PCA9685 VCC -> Pi 3.3V (logic), GND -> Pi GND
- PCA9685 SDA/SCL -> Pi SDA/SCL (GPIO2/GPIO3 on Pi 4B)
- Servo power: use external 5-6V supply to PCA9685 V+ and GND (share ground with Pi).
"""

import sys
import time

try:
    from adafruit_pca9685 import PCA9685
    import board
    import busio
except ImportError:
    print("Missing libraries. Install with:")
    print("  sudo pip3 install adafruit-circuitpython-pca9685")
    sys.exit(1)


# ---------------------------
# Configuration
# ---------------------------
I2C_ADDRESS = 0x40       # default PCA9685 address; change if you changed A0-A5 jumpers
SERVO_CHANNEL = 4        # "module 0" output
PWM_FREQUENCY_HZ = 50    # standard servo frequency

# Continuous servo calibration (in microseconds)
# Many continuous servos: ~1500us stop, <1500 one direction, >1500 the other.
STOP_US = 1525 # true stop_us

# How far from STOP_US counts as "full speed".
# Typical safe range is about +/- 400us, but your servo may vary (e.g., +/- 300..500).
RANGE_US = 400

# Safety clamp so we never exceed reasonable servo pulse widths
MIN_US = 1000
MAX_US = 2000


# ---------------------------
# Helper functions
# ---------------------------
def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))

def hard_off(self):
    # No pulses (output low). Some servos may still drift if they lose signal.
    self.out.duty_cycle = 0

def us_to_duty_16bit(microseconds: float, freq_hz: float) -> int:
    """
    Convert pulse width in microseconds to 16-bit duty_cycle value expected by CircuitPython PCA9685.
    duty_cycle: 0..65535
    """
    period_us = 1_000_000.0 / freq_hz
    duty_fraction = microseconds / period_us
    duty_fraction = clamp(duty_fraction, 0.0, 1.0)
    return int(duty_fraction * 65535.0)


# ---------------------------
# Servo control
# ---------------------------
class ContinuousServoPCA9685:
    def __init__(self, address=I2C_ADDRESS, channel=SERVO_CHANNEL, freq_hz=PWM_FREQUENCY_HZ):
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c, address=address)
        self.pca.frequency = freq_hz

        self.freq_hz = freq_hz
        self.channel = channel
        self.out = self.pca.channels[channel]

        # Set to stop on start
        self.set_pulse_us(STOP_US)

    def set_pulse_us(self, pulse_us: float):
        pulse_us = clamp(pulse_us, MIN_US, MAX_US)
        self.out.duty_cycle = us_to_duty_16bit(pulse_us, self.freq_hz)

    def stop(self):
        self.set_pulse_us(STOP_US)

    def forward(self, speed: float = 0.5):
        """
        speed: 0.0..1.0
        For this script: forward = pulse > STOP_US
        """
        speed = clamp(speed, 0.0, 1.0)
        pulse = STOP_US + speed * RANGE_US
        self.set_pulse_us(pulse)

    def backward(self, speed: float = 0.5):
        """
        speed: 0.0..1.0
        For this script: backward = pulse < STOP_US
        """
        speed = clamp(speed, 0.0, 1.0)
        pulse = STOP_US - speed * RANGE_US
        self.set_pulse_us(pulse)

    def center(self, stop_us: float):
        """
        Recalibrate stop pulse (useful because continuous servos vary).
        """
        global STOP_US
        STOP_US = int(stop_us)
        self.stop()

    def deinit(self):
        try:
            self.stop()
            time.sleep(0.1)
        finally:
            self.pca.deinit()


# ---------------------------
# Simple terminal command loop
# ---------------------------
def print_help():
    print("""
Commands:
  forward [speed]      -> rotate forward (speed 0.0..1.0, default 0.5)
  backward [speed]     -> rotate backward (speed 0.0..1.0, default 0.5)
  stop                 -> stop servo
  center <us>          -> set stop/neutral pulse width (e.g., center 1500)
  pulse <us>           -> directly set a pulse width in microseconds (debug)
  help                 -> show this help
  quit / exit          -> stop and exit

Examples:
  forward 0.3
  backward 1
  center 1490
  stop
""")


def main():
    servo = ContinuousServoPCA9685()

    print("PCA9685 continuous servo control (channel 0). Type 'help' for commands.")
    try:
        while True:
            line = input("> ").strip()
            if not line:
                continue

            parts = line.split()
            cmd = parts[0].lower()

            if cmd in ("quit", "exit"):
                servo.stop()
                time.sleep(0.2)
                servo.hard_off()
                break

            elif cmd == "help":
                print_help()

            elif cmd == "stop":
                servo.stop()
                print("Stopped.")

            elif cmd == "forward":
                speed = float(parts[1]) if len(parts) > 1 else 0.5
                servo.forward(speed)
                print(f"Forward, speed={clamp(speed,0.0,1.0):.2f}")

            elif cmd == "backward":
                speed = float(parts[1]) if len(parts) > 1 else 0.5
                servo.backward(speed)
                print(f"Backward, speed={clamp(speed,0.0,1.0):.2f}")

            elif cmd == "center":
                if len(parts) < 2:
                    print("Usage: center <microseconds>  (e.g., center 1500)")
                    continue
                us = float(parts[1])
                servo.center(us)
                print(f"Set neutral/stop to {int(us)} us and stopped.")

            elif cmd == "pulse":
                if len(parts) < 2:
                    print("Usage: pulse <microseconds>  (e.g., pulse 1600)")
                    continue
                us = float(parts[1])
                servo.set_pulse_us(us)
                print(f"Set pulse to {int(us)} us")

            else:
                print("Unknown command. Type 'help'.")

    except KeyboardInterrupt:
        servo.stop()
        print("\nStopped (Ctrl+C).")
    finally:
        servo.deinit()


if __name__ == "__main__":
    main()