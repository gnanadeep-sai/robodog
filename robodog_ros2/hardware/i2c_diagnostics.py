"""
Standalone I2C/PCA9685 diagnostic CLI.

Refactor of the legacy `tools/i2c_test.py`. Verifies the I2C bus and the
connected PCA9685 driver address, and lets you manually jog each of the
12 servo channels. Uses only `pigpio` -- no CircuitPython/`adafruit_*`
dependency, and no ROS dependency (run this directly on the Pi, outside
of `colcon`/`ros2 run`, for bring-up/debugging).

Usage:
    python3 -m robodog_ros2.hardware.i2c_diagnostics [--bus 1] [--address 0x40]
"""
import argparse
import time

from .pca9685_driver import PCA9685Driver

SPEED_DEG_PER_SEC = 20.0
UPDATE_RATE_HZ = 50.0


def _angle_to_duty_cycle(angle_deg: float) -> int:
    pulse_ms = 1.0 + (angle_deg / 180.0)
    return int((pulse_ms / 20.0) * 65535)


def move_servo_smooth(pca: PCA9685Driver, channel: int, start_angle: float, target_angle: float):
    if start_angle == target_angle:
        return

    delta_angle = target_angle - start_angle
    duration = abs(delta_angle) / SPEED_DEG_PER_SEC
    steps = max(1, int(duration * UPDATE_RATE_HZ))
    sleep_time = 1.0 / UPDATE_RATE_HZ

    for i in range(1, steps + 1):
        current = start_angle + (delta_angle * i / steps)
        pca.set_channel_duty_cycle(channel, _angle_to_duty_cycle(current))
        time.sleep(sleep_time)


def test_all(pca: PCA9685Driver):
    print("\nTesting all 12 servos...")
    for i in range(12):
        print(f"Moving servo {i} to 90 degrees (neutral)...")
        pca.set_channel_duty_cycle(i, _angle_to_duty_cycle(90.0))
        time.sleep(0.2)


def main():
    parser = argparse.ArgumentParser(description="PCA9685 I2C diagnostics (raw pigpio)")
    parser.add_argument("--bus", type=int, default=1, help="I2C bus number")
    parser.add_argument("--address", type=lambda x: int(x, 0), default=0x40, help="PCA9685 I2C address")
    args = parser.parse_args()

    print("Initializing I2C and PCA9685 via pigpio...")
    try:
        pca = PCA9685Driver(i2c_bus=args.bus, i2c_address=args.address, frequency_hz=50.0)
    except Exception as e:
        print(f"Error initializing I2C/PCA9685: {e}")
        print(
            "Make sure the pigpio daemon is running (`sudo pigpiod`), I2C is "
            "enabled on your Raspberry Pi, and the PCA9685 is connected."
        )
        return

    current_angles = [90.0] * 12
    print("\nPCA9685 12-Servo Manual Test (pigpio)")
    print("--------------------------------------")
    print("This script allows you to test each of the 12 servos individually.")

    while True:
        try:
            print("\nOptions:")
            print("  0-11: Select servo channel to test")
            print("  a: Test all 12 servos sequentially")
            print("  q: Quit")

            user_input = input("Enter option: ").strip().lower()

            if user_input == "q":
                break
            elif user_input == "a":
                test_all(pca)
                continue

            channel = int(user_input)
            if not (0 <= channel <= 11):
                print("Invalid channel. Please enter a number between 0 and 11.")
                continue

            angle_input = input(
                f"Enter angle for servo {channel} (0-180) or 'n' for neutral (90): "
            ).strip().lower()
            angle = 90.0 if angle_input == "n" else float(angle_input)

            if not (0 <= angle <= 180):
                print("Invalid angle. Please enter a number between 0 and 180.")
                continue

            print(
                f"Moving servo {channel} from {current_angles[channel]:.1f} to "
                f"{angle:.1f} degrees smoothly..."
            )
            move_servo_smooth(pca, channel, current_angles[channel], angle)
            current_angles[channel] = angle

            print(f"Servo {channel} set to {angle} degrees.")

        except ValueError:
            print("Invalid input.")
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"Error communicating with PCA9685: {e}")

    pca.close()
    print("\nExiting and deactivating PCA9685 channels.")


if __name__ == "__main__":
    main()