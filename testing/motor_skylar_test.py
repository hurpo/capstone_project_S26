from M5MotorController import M5Module4EncoderMotor
import time
from smbus3 import SMBus


I2C_ADDR = 0x24

def check_i2c_device(addr):
    print(f"Checking for device at 0x{addr:02X}...")
    try:
        with SMBus(1) as bus:
            bus.write_quick(addr)
        print("✓ Device detected on I2C bus")
        return True
    except Exception as e:
        print("✗ Device NOT found:", e)
        return False


def main():
    # Step 1: Check raw I2C connection
    if not check_i2c_device(I2C_ADDR):
        return

    # Step 2: Create motor object
    motor = M5Module4EncoderMotor(module_addr=I2C_ADDR)

    try:
        print("\nReading encoder values...")
        encoders = motor.getEncoderValues()
        print("Encoder values:", encoders)

        print("\nSpinning motors slowly for 2 seconds...")
        motor.setMotorSpeeds([50, 50, 50, 50])  # small PWM value

        time.sleep(4)

        print("Stopping motors...")
        motor.setMotorSpeeds([0, 0, 0, 0])

        print("\nReading encoder values again...")
        encoders = motor.getEncoderValues()
        print("Encoder values:", encoders)

        print("\nTest complete ✓")

    except Exception as e:
        print("Error communicating with motor module:", e)


if __name__ == "__main__":
    main()