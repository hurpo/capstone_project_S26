from smbus2 import SMBus, i2c_msg
import struct
import time
from statistics import mean, pstdev

ADDR = 0x24
BUS = 1
REG_ENC_BASE = 0x30  # M1 starts at 0x30, each motor uses 4 bytes


def read_encoder(bus: SMBus, motor_index: int) -> int:
    """Read signed int32 encoder count for motor 1..4 (BIG-ENDIAN verified)."""
    if motor_index not in (1, 2, 3, 4):
        raise ValueError("motor_index must be 1..4")

    reg = REG_ENC_BASE + 4 * (motor_index - 1)
    w = i2c_msg.write(ADDR, [reg])
    r = i2c_msg.read(ADDR, 4)
    bus.i2c_rdwr(w, r)
    raw = bytes(list(r))
    return struct.unpack(">i", raw)[0]  # BIG-ENDIAN


def main():
    print("\nM5Stack 4EncoderMotor - Counts Per Wheel Revolution Calibration (Averaged)")
    print("--------------------------------------------------------------------------")
    print("Goal: measure COUNTS_PER_REV_OUT (encoder counts per 1 wheel revolution).")
    print("Make sure motors are NOT powered/driving. Rotate wheel by hand.\n")

    # Motor selection
    try:
        motor = int(input("Which motor are you calibrating? Enter 1-4: ").strip())
        if motor not in (1, 2, 3, 4):
            raise ValueError
    except ValueError:
        print("Invalid motor selection.")
        return

    # Number of trials
    try:
        trials = int(input("How many trials? (recommended 3-5): ").strip())
        if trials <= 0:
            raise ValueError
    except ValueError:
        print("Invalid trials count.")
        return

    deltas = []

    with SMBus(BUS) as bus:
        time.sleep(0.05)

        print("\nInstructions for each trial:")
        print("1) Wait for the prompt.")
        print("2) Rotate the wheel EXACTLY 1 full revolution.")
        print("3) Press Enter.\n")

        for k in range(1, trials + 1):
            input(f"Trial {k}/{trials}: Press Enter to capture START count... ")
            start = read_encoder(bus, motor)
            print(f"  START = {start}")

            input("  Rotate wheel 1 full rev, then press Enter to capture END... ")
            end = read_encoder(bus, motor)
            delta = end - start
            adelta = abs(delta)
            deltas.append(adelta)

            print(f"  END   = {end}")
            print(f"  Δ     = {delta}   (abs -> {adelta})\n")

    avg = mean(deltas)
    sd = pstdev(deltas) if len(deltas) > 1 else 0.0

    # Suggest a usable integer CPR
    suggested = int(round(avg))

    print("Results")
    print("-------")
    print(f"Abs deltas: {deltas}")
    print(f"Average CPR (counts per 1 wheel rev): {avg:.3f}")
    print(f"Std dev (population):                {sd:.3f}")
    print(f"\nSuggested COUNTS_PER_REV_OUT = {suggested}")

    # Quick sanity check guidance
    if suggested == 0:
        print("\nWARNING: CPR resolved to 0. Encoder wiring/pinout likely incorrect.")
    elif sd > 0 and (sd / avg) > 0.05:
        print("\nNote: Variation >5%. Try slower/more precise 1-rev turns, or do more trials.")

    print("\nUse COUNTS_PER_REV_OUT in your distance/RPM calculations.")
    print("Direction sign doesn’t matter for CPR; you can handle direction separately.")


if __name__ == "__main__":
    main()
