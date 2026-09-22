import board
import busio
from adafruit_pca9685 import PCA9685
from pupper.Config import ServoParams, PWMParams


class HardwareInterface:
    def __init__(self):
        self.pwm_params = PWMParams()
        self.servo_params = ServoParams()
        
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = self.pwm_params.freq
        
        initialize_pwm(self.pca, self.pwm_params)

    def set_actuator_postions(self, joint_angles):
        send_servo_commands(self.pca, self.pwm_params, self.servo_params, joint_angles)
    
    def set_actuator_position(self, joint_angle, axis, leg):
        send_servo_command(self.pca, self.pwm_params, self.servo_params, joint_angle, axis, leg)


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


def initialize_pwm(pca, pwm_params):
    # PCA9685 frequency is set globally in __init__
    # We can optionally set all duty cycles to 0 here to ensure they are off initially
    for leg_index in range(4):
        for axis_index in range(3):
            channel = pwm_params.pins[axis_index, leg_index]
            pca.channels[channel].duty_cycle = 0


def send_servo_commands(pca, pwm_params, servo_params, joint_angles):
    for leg_index in range(4):
        for axis_index in range(3):
            duty_cycle = angle_to_duty_cycle(
                joint_angles[axis_index, leg_index],
                pwm_params,
                servo_params,
                axis_index,
                leg_index,
            )
            channel = pwm_params.pins[axis_index, leg_index]
            pca.channels[channel].duty_cycle = duty_cycle


def send_servo_command(pca, pwm_params, servo_params, joint_angle, axis, leg):
    duty_cycle = angle_to_duty_cycle(joint_angle, pwm_params, servo_params, axis, leg)
    channel = pwm_params.pins[axis, leg]
    pca.channels[channel].duty_cycle = duty_cycle


def deactivate_servos(pca, pwm_params):
    for leg_index in range(4):
        for axis_index in range(3):
            channel = pwm_params.pins[axis_index, leg_index]
            pca.channels[channel].duty_cycle = 0
