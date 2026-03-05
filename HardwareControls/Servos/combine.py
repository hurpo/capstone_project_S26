#!/usr/bin/env python3
"""
Two continuous-rotation servos (TD-8135MG) via PCA9685 on Raspberry Pi (I2C).
- Uses channels 3 and 4.
- Default "run" command spins them at full speed in opposite directions:
    ch3 forward full, ch4 backward full

Terminal commands:
  run                  -> both servos full speed opposite directions
  stop                 -> both servos to STOP_US
  center <us>           -> set stop/neutral pulse width (applies to both)
  swap                 -> swap which channel goes which direction
  pulse3 <us> / pulse4 <us> -> direct pulse debug per channel
  off                  -> duty_cycle=0 on both channels (no pulses)
  help
  quit / exit
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
I2C_ADDRESS = 0x40
PWM_FREQUENCY_HZ = 50

SERVO_CH_A = 3  # TD-8135MG #1
SERVO_CH_B = 4  # TD-8135MG #2

# Your measured neutral
STOP_US = 1525

# Full-speed offset from neutral (tune if needed)
RANGE_US = 400

# Safety clamp
MIN_US = 1000
MAX_US = 2000


# ---------------------------
# Helper functions
# ---------------------------
def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def us_to_duty_16bit(microseconds: float, freq_hz: float) -> int:
    period_us = 1_000_000.0 / freq_hz
    duty_fraction = clamp(microseconds / period_us, 0.0, 1.0)
    return int(duty_fraction * 65535.0)


# ---------------------------
# Servo control (single channel wrapper)
# ---------------------------
class ContinuousServoChannel:
    def __init__(self, pca: PCA9685, channel: int, freq_hz: int):
        self.pca = pca
        self.freq_hz = freq_hz
        self.channel = channel
        self.out = pca.channels[channel]

        # Stop on init
        self.set_pulse_us(STOP_US)

    def set_pulse_us(self, pulse_us: float):
        pulse_us = clamp(pulse_us, MIN_US, MAX_US)
        self.out.duty_cycle = us_to_duty_16bit(pulse_us, self.freq_hz)

    def hard_off(self):
        # No pulses (output low). Some servos may drift if they lose signal.
        self.out.duty_cycle = 0

    def stop(self):
        self.set_pulse_us(STOP_US)

    def forward(self, speed: float = 1.0):
        speed = clamp(speed, 0.0, 1.0)
        self.set_pulse_us(STOP_US + speed * RANGE_US)

    def backward(self, speed: float = 1.0):
        speed = clamp(speed, 0.0, 1.0)
        self.set_pulse_us(STOP_US - speed * RANGE_US)


# ---------------------------
# Dual-servo controller
# ---------------------------
class DualContinuousServos:
    def __init__(self, address=I2C_ADDRESS, ch_a=SERVO_CH_A, ch_b=SERVO_CH_B, freq_hz=PWM_FREQUENCY_HZ):
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c, address=address)
        self.pca.frequency = freq_hz

        self.a = ContinuousServoChannel(self.pca, ch_a, freq_hz)
        self.b = ContinuousServoChannel(self.pca, ch_b, freq_hz)

        # Direction mapping: if True => A forward, B backward. If False => swapped.
        self.a_forward_b_backward = True

    def run_opposite_full(self):
        if self.a_forward_b_backward:
            self.a.forward(1.0)
            self.b.backward(1.0)
        else:
            self.a.backward(1.0)
            self.b.forward(1.0)

    def stop_all(self):
        self.a.stop()
        self.b.stop()

    def hard_off_all(self):
        self.a.hard_off()
        self.b.hard_off()

    def swap(self):
        self.a_forward_b_backward = not self.a_forward_b_backward

    def center(self, stop_us: float):
        global STOP_US
        STOP_US = int(stop_us)
        self.stop_all()

    def deinit(self):
        try:
            self.stop_all()
            time.sleep(0.2)
        finally:
            self.pca.deinit()


# ---------------------------
# Terminal CLI
# ---------------------------
def print_help():
    print(f"""
Channels: A={SERVO_CH_A}, B={SERVO_CH_B}
Neutral STOP_US={STOP_US}  RANGE_US={RANGE_US}

Commands:
  run                 -> both servos full speed opposite directions
  stop                -> both servos to neutral (STOP_US)
  center <us>         -> set new neutral for both (e.g., center 1525)
  swap                -> swap which channel goes which direction
  pulse3 <us>         -> set channel 3 pulse (debug)
  pulse4 <us>         -> set channel 4 pulse (debug)
  off                 -> duty_cycle=0 on both channels (no pulses)
  help
  quit / exit         -> stop + off + exit
""")


def main():
    servos = DualContinuousServos()
    print("Dual TD-8135MG control on PCA9685 channels 3 & 4. Type 'help'.")

    try:
        while True:
            line = input("> ").strip()
            if not line:
                continue

            parts = line.split()
            cmd = parts[0].lower()

            if cmd in ("quit", "exit"):
                servos.stop_all()
                time.sleep(0.2)
                servos.hard_off_all()
                break

            elif cmd == "help":
                print_help()

            elif cmd == "run":
                servos.run_opposite_full()
                mapping = "A forward, B backward" if servos.a_forward_b_backward else "A backward, B forward"
                print(f"Running opposite full speed ({mapping}).")

            elif cmd == "stop":
                servos.stop_all()
                print("Stopped (neutral pulses).")

            elif cmd == "off":
                servos.hard_off_all()
                print("Outputs off (duty_cycle=0).")

            elif cmd == "swap":
                servos.swap()
                mapping = "A forward, B backward" if servos.a_forward_b_backward else "A backward, B forward"
                print(f"Swapped direction mapping: {mapping}")

            elif cmd == "center":
                if len(parts) < 2:
                    print("Usage: center <microseconds>  (e.g., center 1525)")
                    continue
                us = float(parts[1])
                servos.center(us)
                print(f"Set neutral/stop to {int(us)} us (applies to both).")

            elif cmd == "pulse3":
                if len(parts) < 2:
                    print("Usage: pulse3 <microseconds>")
                    continue
                us = float(parts[1])
                servos.a.set_pulse_us(us) if SERVO_CH_A == 3 else servos.b.set_pulse_us(us)
                print(f"Set channel 3 pulse to {int(us)} us")

            elif cmd == "pulse4":
                if len(parts) < 2:
                    print("Usage: pulse4 <microseconds>")
                    continue
                us = float(parts[1])
                servos.a.set_pulse_us(us) if SERVO_CH_A == 4 else servos.b.set_pulse_us(us)
                print(f"Set channel 4 pulse to {int(us)} us")

            else:
                print("Unknown command. Type 'help'.")

    except KeyboardInterrupt:
        servos.stop_all()
        print("\nStopped (Ctrl+C).")
    finally:
        servos.deinit()


if __name__ == "__main__":
    main()