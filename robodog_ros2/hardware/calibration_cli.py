"""
Interactive servo neutral-angle calibration CLI.

Refactor of the legacy `calibrate_servos.py`. No ROS dependencies --
talks straight to `servo_driver.ServoDriver` for headless tuning on the
bench. Calibration math (offsets, sign conventions per axis) is
preserved exactly.

Where the legacy script regenerated `pupper/ServoCalibration.py`, this
version prints a YAML snippet you paste into
`config/pupper_params.yaml` under `hardware_node.ros__parameters`,
since calibration is now a ROS 2 parameter rather than a generated
Python module.

Usage:
    python3 -m robodog_ros2.hardware.calibration_cli [--bus 1] [--address 0x40]
"""
import argparse

import numpy as np

from ..core.config import PWMParams, ServoParams
from .servo_driver import ServoDriver


def get_motor_name(i, j):
    motor_type = {0: "abduction", 1: "inner", 2: "outer"}  # Top  # Bottom
    leg_pos = {0: "front-right", 1: "front-left", 2: "back-right", 3: "back-left"}
    return motor_type[i] + " " + leg_pos[j]


def get_motor_setpoint(i, j):
    data = np.array([[0, 0, 0, 0], [45, 45, 45, 45], [45, 45, 45, 45]])
    return data[i, j]


def degrees_to_radians(input_array):
    return input_array * np.pi / 180.0


def radians_to_degrees(input_array):
    return input_array * 180.0 / np.pi


def step_until(servo_driver, axis, leg, set_point):
    """Returns the angle offset needed to correct a given link by asking the user for input."""
    found_position = False
    set_names = ["horizontal", "horizontal", "vertical"]
    offset = 0
    while not found_position:
        move_input = str(
            input(
                "Enter 'a' or 'b' to move the link until it is **"
                + set_names[axis]
                + "**. Enter 'd' when done. Input: "
            )
        )
        if move_input == "a":
            offset += 1.0
            servo_driver.set_actuator_position(
                degrees_to_radians(set_point + offset), axis, leg
            )
        elif move_input == "b":
            offset -= 1.0
            servo_driver.set_actuator_position(
                degrees_to_radians(set_point + offset), axis, leg
            )
        elif move_input == "d":
            found_position = True
        print("Offset: ", offset)

    return offset


def calibrate_angle_offset(servo_driver):
    """Calibrate the angle offset for the twelve motors on the robot.
    Note that servo_driver.servo_params is modified in-place.
    """
    print(
        "The scaling constant for your servo represents how much you have to "
        "increase\nthe pwm pulse width (in microseconds) to rotate the servo "
        "output 1 degree."
    )
    print(
        "This value is currently set to: {:.3f}".format(
            degrees_to_radians(servo_driver.servo_params.micros_per_rad)
        )
    )
    print("For newer CLS6336 and CLS6327 servos the value should be 11.333.")
    ks = input("Press <Enter> to keep the current value, or enter a new value: ")
    if ks != "":
        k = float(ks)
        servo_driver.servo_params.micros_per_rad = k * 180 / np.pi

    servo_driver.servo_params.neutral_angle_degrees = np.zeros((3, 4))

    for leg_index in range(4):
        for axis in range(3):
            completed = False
            while not completed:
                motor_name = get_motor_name(axis, leg_index)
                print("\n\nCalibrating the **" + motor_name + " motor **")
                set_point = get_motor_setpoint(axis, leg_index)

                servo_driver.servo_params.neutral_angle_degrees[axis, leg_index] = 0

                servo_driver.set_actuator_position(
                    degrees_to_radians(set_point), axis, leg_index
                )

                offset = step_until(servo_driver, axis, leg_index, set_point)
                print("Final offset: ", offset)

                # The upper leg link has a different equation because we're
                # calibrating to make it horizontal, not vertical
                if axis == 1:
                    servo_driver.servo_params.neutral_angle_degrees[axis, leg_index] = (
                        set_point - offset
                    )
                else:
                    servo_driver.servo_params.neutral_angle_degrees[axis, leg_index] = -(
                        set_point + offset
                    )
                print(
                    "Calibrated neutral angle: ",
                    servo_driver.servo_params.neutral_angle_degrees[axis, leg_index],
                )

                servo_driver.set_actuator_position(
                    degrees_to_radians([0, 45, -45][axis]), axis, leg_index
                )
                okay = ""
                prompt = (
                    "The leg should be at exactly **"
                    + ["horizontal", "45 degrees", "45 degrees"][axis]
                    + "**. Are you satisfied? Enter 'yes' or 'no': "
                )
                while okay not in ["y", "n", "yes", "no"]:
                    okay = str(input(prompt))
                completed = okay in ("y", "yes")


def print_yaml_snippet(servo_params):
    matrix = servo_params.neutral_angle_degrees.tolist()
    print("\n# --- paste into config/pupper_params.yaml under hardware_node.ros__parameters ---")
    print(
        "micros_per_rad: {:.3f}".format(
            degrees_to_radians(servo_params.micros_per_rad)
        )
    )
    print("neutral_angle_degrees:")
    for row in matrix:
        print("  - [{}]".format(", ".join(f"{v:.2f}" for v in row)))


def main():
    parser = argparse.ArgumentParser(description="Headless servo calibration (no ROS)")
    parser.add_argument("--bus", type=int, default=1)
    parser.add_argument("--address", type=lambda x: int(x, 0), default=0x40)
    args = parser.parse_args()

    pwm_params = PWMParams()
    servo_params = ServoParams()
    servo_driver = ServoDriver(
        pwm_params, servo_params, i2c_bus=args.bus, i2c_address=args.address
    )

    try:
        calibrate_angle_offset(servo_driver)
        print("\n\n CALIBRATION COMPLETE!\n")
        print("Calibrated neutral angles:")
        print(servo_driver.servo_params.neutral_angle_degrees)
        print_yaml_snippet(servo_driver.servo_params)
    finally:
        servo_driver.close()


if __name__ == "__main__":
    main()