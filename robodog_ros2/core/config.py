"""
Robot configuration objects.

This module is intentionally free of any ROS imports so it stays unit
testable in isolation (per the migration requirement that `core/` has
zero `rclpy` imports). The `nodes/` layer is responsible for reading
ROS 2 YAML parameters and handing them to these classes as plain dicts.

All default numeric values below are copied verbatim from the legacy
`pupper/Config.py`, `pupper/HardwareConfig.py` and
`pupper/ServoCalibration.py` files to preserve the exact gait,
inverse-kinematics and servo-calibration math.
"""
import numpy as np

# ----------------------------------------------------------------------
# Defaults migrated verbatim from pupper/ServoCalibration.py
# ----------------------------------------------------------------------
DEFAULT_MICROS_PER_RAD = 11.333 * 180.0 / np.pi
DEFAULT_NEUTRAL_ANGLE_DEGREES = np.array(
    [[-11.0, -9.0, -12.0, -9.0],
     [58.0, 48.0, 54.0, 40.0],
     [-45.0, -22.0, -42.0, -38.0]]
)

# ----------------------------------------------------------------------
# Defaults migrated verbatim from pupper/HardwareConfig.py
# ----------------------------------------------------------------------
DEFAULT_PS4_COLOR = {"red": 0, "blue": 0, "green": 255}
DEFAULT_PS4_DEACTIVATED_COLOR = {"red": 0, "blue": 0, "green": 50}


def _as_matrix(value, shape):
    """Accept either a nested list (as it comes back from a ROS 2 YAML
    param) or a numpy array, and always return a numpy array of `shape`.
    """
    arr = np.array(value, dtype=float)
    return arr.reshape(shape)


class PWMParams:
    """PCA9685 channel map and PWM signal parameters."""

    def __init__(self, params: dict = None):
        params = params or {}
        # 3 rows (Abduction, Inner Hip, Outer Hip) x 4 columns (FR, FL, BR, BL)
        self.pins = _as_matrix(
            params.get("pins", [[0, 1, 2, 3], [4, 5, 6, 7], [8, 9, 10, 11]]),
            (3, 4),
        ).astype(int)
        self.range = int(params.get("range", 65535))
        self.freq = float(params.get("freq", 50))


class ServoParams:
    """Servo calibration: converts joint angles (radians) to PWM pulse
    widths (microseconds). Math preserved exactly from
    pupper/Config.py::ServoParams.
    """

    def __init__(self, params: dict = None):
        params = params or {}
        self.neutral_position_pwm = float(params.get("neutral_position_pwm", 1500))
        self.micros_per_rad = float(params.get("micros_per_rad", DEFAULT_MICROS_PER_RAD))

        # The neutral angle of the joint relative to the modeled zero-angle
        # in degrees, for each joint (3 x 4: axis x leg)
        self.neutral_angle_degrees = _as_matrix(
            params.get("neutral_angle_degrees", DEFAULT_NEUTRAL_ANGLE_DEGREES), (3, 4)
        )

        self.servo_multipliers = _as_matrix(
            params.get(
                "servo_multipliers",
                [[1, 1, 1, 1], [-1, 1, -1, 1], [1, -1, 1, -1]],
            ),
            (3, 4),
        )

    @property
    def neutral_angles(self):
        return self.neutral_angle_degrees * np.pi / 180.0  # Convert to radians


class Configuration:
    """Gait, stance, swing and geometry parameters. Numeric defaults and
    derived-property math preserved exactly from pupper/Config.py.
    """

    def __init__(self, params: dict = None):
        params = params or {}

        ################# CONTROLLER BASE COLOR ##############
        self.ps4_color = params.get("ps4_color", DEFAULT_PS4_COLOR)
        self.ps4_deactivated_color = params.get(
            "ps4_deactivated_color", DEFAULT_PS4_DEACTIVATED_COLOR
        )

        #################### COMMANDS ####################
        self.max_x_velocity = float(params.get("max_x_velocity", 0.4))
        self.max_y_velocity = float(params.get("max_y_velocity", 0.3))
        self.max_yaw_rate = float(params.get("max_yaw_rate", 2.0))
        self.max_pitch = float(params.get("max_pitch", 30.0 * np.pi / 180.0))

        #################### MOVEMENT PARAMS ####################
        self.z_time_constant = float(params.get("z_time_constant", 0.02))
        self.z_speed = float(params.get("z_speed", 0.03))  # maximum speed [m/s]
        self.pitch_deadband = float(params.get("pitch_deadband", 0.02))
        self.pitch_time_constant = float(params.get("pitch_time_constant", 0.25))
        self.max_pitch_rate = float(params.get("max_pitch_rate", 0.15))
        self.roll_speed = float(params.get("roll_speed", 0.16))  # max roll rate [rad/s]
        self.yaw_time_constant = float(params.get("yaw_time_constant", 0.3))
        self.max_stance_yaw = float(params.get("max_stance_yaw", 1.2))
        self.max_stance_yaw_rate = float(params.get("max_stance_yaw_rate", 2.0))

        #################### STANCE ####################
        self.delta_x = float(params.get("delta_x", 0.1))
        self.delta_y = float(params.get("delta_y", 0.09))
        self.x_shift = float(params.get("x_shift", 0.0))
        self.default_z_ref = float(params.get("default_z_ref", -0.16))

        #################### SWING ######################
        self.z_coeffs = None
        self.z_clearance = float(params.get("z_clearance", 0.07))
        # Ratio between touchdown distance and total horizontal stance movement
        self.alpha = float(params.get("alpha", 0.5))
        self.beta = float(params.get("beta", 0.5))

        #################### GAIT #######################
        self.dt = float(params.get("dt", 0.01))
        self.num_phases = int(params.get("num_phases", 4))
        self.contact_phases = _as_matrix(
            params.get(
                "contact_phases",
                [[1, 1, 1, 0], [1, 0, 1, 1], [1, 0, 1, 1], [1, 1, 1, 0]],
            ),
            (4, 4),
        )
        # duration of the phase where all four feet are on the ground
        self.overlap_time = float(params.get("overlap_time", 0.10))
        # duration of the phase when only two feet are on the ground
        self.swing_time = float(params.get("swing_time", 0.15))

        ######################## GEOMETRY ######################
        self.LEG_FB = float(params.get("leg_fb", 0.10))
        self.LEG_LR = float(params.get("leg_lr", 0.04))
        self.LEG_L2 = float(params.get("leg_l2", 0.115))
        self.LEG_L1 = float(params.get("leg_l1", 0.1235))
        self.ABDUCTION_OFFSET = float(params.get("abduction_offset", 0.03))
        self.FOOT_RADIUS = float(params.get("foot_radius", 0.01))

        self.HIP_L = float(params.get("hip_l", 0.0394))
        self.HIP_W = float(params.get("hip_w", 0.0744))
        self.HIP_T = float(params.get("hip_t", 0.0214))
        self.HIP_OFFSET = float(params.get("hip_offset", 0.0132))

        self.L = float(params.get("body_l", 0.276))
        self.W = float(params.get("body_w", 0.100))
        self.T = float(params.get("body_t", 0.050))

        self.LEG_ORIGINS = np.array(
            [
                [self.LEG_FB, self.LEG_FB, -self.LEG_FB, -self.LEG_FB],
                [-self.LEG_LR, self.LEG_LR, -self.LEG_LR, self.LEG_LR],
                [0, 0, 0, 0],
            ]
        )

        self.ABDUCTION_OFFSETS = np.array(
            [
                -self.ABDUCTION_OFFSET,
                self.ABDUCTION_OFFSET,
                -self.ABDUCTION_OFFSET,
                self.ABDUCTION_OFFSET,
            ]
        )

        ################### INERTIAL ####################
        self.FRAME_MASS = float(params.get("frame_mass", 0.560))  # kg
        self.MODULE_MASS = float(params.get("module_mass", 0.080))  # kg
        self.LEG_MASS = float(params.get("leg_mass", 0.030))  # kg
        self.MASS = self.FRAME_MASS + (self.MODULE_MASS + self.LEG_MASS) * 4

        # Compensation factor of 3 because the inertia measurement was just
        # of the carbon fiber and plastic parts of the frame and did not
        # include the hip servos and electronics
        self.FRAME_INERTIA = tuple(
            map(lambda x: 3.0 * x, (1.844e-4, 1.254e-3, 1.337e-3))
        )
        self.MODULE_INERTIA = (3.698e-5, 7.127e-6, 4.075e-5)

        leg_z = 1e-6
        leg_mass = 0.010
        leg_x = 1 / 12 * self.LEG_L1 ** 2 * leg_mass
        leg_y = leg_x
        self.LEG_INERTIA = (leg_x, leg_y, leg_z)

    @property
    def default_stance(self):
        return np.array(
            [
                [
                    self.delta_x + self.x_shift,
                    self.delta_x + self.x_shift,
                    -self.delta_x + self.x_shift,
                    -self.delta_x + self.x_shift,
                ],
                [-self.delta_y, self.delta_y, -self.delta_y, self.delta_y],
                [0, 0, 0, 0],
            ]
        )

    ################## SWING ###########################
    @property
    def z_clearance(self):
        return self.__z_clearance

    @z_clearance.setter
    def z_clearance(self, z):
        self.__z_clearance = z

    ########################### GAIT ####################
    @property
    def overlap_ticks(self):
        return int(self.overlap_time / self.dt)

    @property
    def swing_ticks(self):
        return int(self.swing_time / self.dt)

    @property
    def stance_ticks(self):
        return 2 * self.overlap_ticks + self.swing_ticks

    @property
    def phase_ticks(self):
        return np.array(
            [self.overlap_ticks, self.swing_ticks, self.overlap_ticks, self.swing_ticks]
        )

    @property
    def phase_length(self):
        return 2 * self.overlap_ticks + 2 * self.swing_ticks