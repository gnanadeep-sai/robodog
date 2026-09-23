"""
Servo driver -- hardware abstraction layer, decoupled from ROS.

The angle -> PWM -> duty-cycle conversion functions below are extracted
verbatim (surgically, per the migration spec) from the legacy
`pupper/HardwareInterface.py`. Only the transport underneath changed:
instead of `adafruit_pca9685`/CircuitPython, channel duty cycles are
now written through `PCA9685Driver`, which talks raw PCA9685 registers
over `pigpio`.
"""
from .pca9685_driver import PCA9685Driver


def pwm_to_duty_cycle(pulsewidth_micros, pwm_params):
    """Converts a pwm signal (measured in microseconds) to a corresponding duty cycle on the gpio pwm pin

    Parameters
    ----------
    pulsewidth_micros : float
        Width of the pwm signal in microseconds
    pwm_params : PWMParams
        PWMParams object

    Returns
    -------
    int
        PWM duty cycle corresponding to the pulse width
    """
    return int(pulsewidth_micros / 1e6 * pwm_params.freq * pwm_params.range)


def angle_to_pwm(angle, servo_params, axis_index, leg_index):
    """Converts a desired servo angle into the corresponding PWM command

    Parameters
    ----------
    angle : float
        Desired servo angle, relative to the vertical (z) axis
    servo_params : ServoParams
        ServoParams object
    axis_index : int
        Specifies which joint of leg to control. 0 is abduction servo, 1 is inner hip servo, 2 is outer hip servo.
    leg_index : int
        Specifies which leg to control. 0 is front-right, 1 is front-left, 2 is back-right, 3 is back-left.

    Returns
    -------
    float
        PWM width in microseconds
    """
    angle_deviation = (
        angle - servo_params.neutral_angles[axis_index, leg_index]
    ) * servo_params.servo_multipliers[axis_index, leg_index]
    pulse_width_micros = (
        servo_params.neutral_position_pwm
        + servo_params.micros_per_rad * angle_deviation
    )
    return pulse_width_micros


def angle_to_duty_cycle(angle, pwm_params, servo_params, axis_index, leg_index):
    return pwm_to_duty_cycle(
        angle_to_pwm(angle, servo_params, axis_index, leg_index), pwm_params
    )


class ServoDriver:
    """Owns the PCA9685Driver plus the calibration parameters, and
    exposes the same `set_actuator_position(s)` surface the legacy
    `HardwareInterface` had, so `hardware_node.py` and
    `calibration_cli.py` can share this one implementation.
    """

    def __init__(self, pwm_params, servo_params, i2c_bus: int = 1, i2c_address: int = 0x40):
        self.pwm_params = pwm_params
        self.servo_params = servo_params
        self.pca = PCA9685Driver(
            i2c_bus=i2c_bus, i2c_address=i2c_address, frequency_hz=pwm_params.freq
        )

    def set_actuator_positions(self, joint_angles):
        """joint_angles : numpy array (3, 4) [axis_index, leg_index], radians"""
        for leg_index in range(4):
            for axis_index in range(3):
                self.set_actuator_position(
                    joint_angles[axis_index, leg_index], axis_index, leg_index
                )

    def set_actuator_position(self, joint_angle, axis_index, leg_index):
        duty_cycle = angle_to_duty_cycle(
            joint_angle, self.pwm_params, self.servo_params, axis_index, leg_index
        )
        channel = int(self.pwm_params.pins[axis_index, leg_index])
        self.pca.set_channel_duty_cycle(channel, duty_cycle)

    def deactivate_all(self):
        """Zero PWM torque on all 12 channels -- used at startup and by
        the hardware_node's safety watchdog.
        """
        self.pca.all_off()

    def close(self):
        self.pca.close()