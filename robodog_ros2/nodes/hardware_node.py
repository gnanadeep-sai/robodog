"""
The hardware abstraction layer node.

Replaces the legacy `pupper/HardwareInterface.py` + the tail end of
`run_robot_x.py`'s loop (`hardware_interface.set_actuator_postions(...)`).
That stack depended on the Adafruit Blinka/CircuitPython ecosystem
(`board`, `busio`, `adafruit_pca9685`), which is incompatible with the
`pigpiod`-only requirement here. `hardware/pca9685_driver.py` /
`hardware/servo_driver.py` already carry the rewritten raw-register
I2C transport and the verbatim angle-to-duty-cycle math; this node is
purely the ROS 2 <-> `ServoDriver` glue plus the safety watchdog.

  * Lifecycle: instantiates `hardware.servo_driver.ServoDriver` (which
    owns the `pigpio` PCA9685 connection) and loads the servo
    calibration offsets (`neutral_angle_degrees`, `micros_per_rad`,
    `servo_multipliers`, channel `pins`) from ROS 2 parameters.
  * Execution: strictly event-driven -- a callback fires immediately on
    every `/pupper/joint_commands` message and performs the I2C block
    write inline. No polling, no internal control-loop timer.
  * Safety watchdog: a *separate* 30Hz `rclpy` timer independent of the
    command callback. If more than 30ms (3 missed 10ms cycles) has
    elapsed since the last `/pupper/joint_commands` message, it cuts
    all 12 channels' PWM (`ServoDriver.deactivate_all()`) to prevent
    the servos from holding a stale/torqued position after a
    `controller_node` stall or topic dropout.
  * Echoes the commanded joint angles back out on `/joint_states` so
    `robot_state_publisher` can drive the `/tf` tree for RViz.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import JointState

from ..core.config import PWMParams, ServoParams
from ..core.joint_names import JOINT_NAMES, joint_list_to_matrix, matrix_to_joint_list
from ..hardware.servo_driver import ServoDriver

# Time since the last /pupper/joint_commands message after which the
# watchdog cuts servo torque: 3 missed 10ms control-loop cycles.
_WATCHDOG_TIMEOUT_SEC = 0.03
_WATCHDOG_RATE_HZ = 30.0

_PWM_PARAM_DEFAULTS = {
    "pins": [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11],
    "range": 65535,
    "freq": 50.0,
}
_SERVO_PARAM_DEFAULTS = {
    "neutral_position_pwm": 1500.0,
    "micros_per_rad": 11.333 * 180.0 / 3.14159265358979,
    # Row-major flatten of the (3, 4) [axis, leg] matrix: row 0 =
    # abduction x 4 legs, row 1 = inner hip x 4 legs, row 2 = outer hip
    # x 4 legs -- matches `core.config._as_matrix`'s `reshape((3, 4))`.
    "neutral_angle_degrees": [
        -11.0, -9.0, -12.0, -9.0,
        58.0, 48.0, 54.0, 40.0,
        -45.0, -22.0, -42.0, -38.0,
    ],
    "servo_multipliers": [
        1, 1, 1, 1,
        -1, 1, -1, 1,
        1, -1, 1, -1,
    ],
}


class HardwareNode(Node):
    def __init__(self):
        super().__init__("hardware_node")

        self.declare_parameter("i2c_bus", 1)
        self.declare_parameter("i2c_address", 0x40)
        i2c_bus = self.get_parameter("i2c_bus").value
        i2c_address = self.get_parameter("i2c_address").value

        pwm_params = self._load_pwm_params()
        servo_params = self._load_servo_params()

        self.get_logger().info(
            f"Connecting to PCA9685 on I2C bus {i2c_bus}, address "
            f"0x{i2c_address:02X} via pigpiod..."
        )
        self.servo_driver = ServoDriver(
            pwm_params, servo_params, i2c_bus=i2c_bus, i2c_address=i2c_address
        )
        # Start torque-off until the first valid command arrives.
        self.servo_driver.deactivate_all()

        self._last_command_time = None
        self._watchdog_tripped = True

        self.joint_command_sub = self.create_subscription(
            JointState,
            "/pupper/joint_commands",
            self._joint_command_callback,
            QoSPresetProfiles.SENSOR_DATA.value,
        )
        self.joint_state_pub = self.create_publisher(
            JointState, "/joint_states", QoSPresetProfiles.SENSOR_DATA.value
        )

        # Independent from the (event-driven) command callback -- this
        # is what actually enforces the torque cutoff even if
        # `controller_node` dies or the topic silently stops.
        self.watchdog_timer = self.create_timer(
            1.0 / _WATCHDOG_RATE_HZ, self._watchdog_tick
        )

        self.get_logger().info("hardware_node up, waiting for /pupper/joint_commands")

    # ------------------------------------------------------------------
    # Parameter loading
    # ------------------------------------------------------------------
    def _load_pwm_params(self) -> PWMParams:
        params = {}
        for key, default in _PWM_PARAM_DEFAULTS.items():
            params[key] = self.declare_parameter(f"pwm.{key}", default).value
        return PWMParams(params=params)

    def _load_servo_params(self) -> ServoParams:
        params = {}
        for key, default in _SERVO_PARAM_DEFAULTS.items():
            params[key] = self.declare_parameter(f"servo.{key}", default).value
        return ServoParams(params=params)

    # ------------------------------------------------------------------
    # Command handling -- event-driven, fires immediately on message
    # ------------------------------------------------------------------
    def _joint_command_callback(self, msg: JointState):
        joint_angles = self._joint_state_to_matrix(msg)
        if joint_angles is None:
            return

        self.servo_driver.set_actuator_positions(joint_angles)
        self._last_command_time = self.get_clock().now()

        if self._watchdog_tripped:
            self.get_logger().info("/pupper/joint_commands resumed, torque restored")
            self._watchdog_tripped = False

        self._publish_joint_states(joint_angles, msg.header.stamp)

    def _joint_state_to_matrix(self, msg: JointState):
        """Defensive parse: if the message names its joints, reorder by
        `JOINT_NAMES` so an out-of-order/partial publisher can't scramble
        which servo gets which angle. Falls back to assuming canonical
        order when no names are provided.
        """
        if msg.name:
            by_name = dict(zip(msg.name, msg.position))
            missing = [n for n in JOINT_NAMES if n not in by_name]
            if missing:
                self.get_logger().warn(
                    f"/pupper/joint_commands missing joints {missing}, dropping message"
                )
                return None
            values = [by_name[n] for n in JOINT_NAMES]
        else:
            if len(msg.position) != len(JOINT_NAMES):
                self.get_logger().warn(
                    f"/pupper/joint_commands has {len(msg.position)} positions, "
                    f"expected {len(JOINT_NAMES)}; dropping message"
                )
                return None
            values = list(msg.position)
        return joint_list_to_matrix(values)

    def _publish_joint_states(self, joint_angles, stamp=None):
        msg = JointState()
        msg.header.stamp = stamp if stamp else self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = matrix_to_joint_list(joint_angles)
        self.joint_state_pub.publish(msg)

    # ------------------------------------------------------------------
    # Safety watchdog -- independent 30Hz timer
    # ------------------------------------------------------------------
    def _watchdog_tick(self):
        now = self.get_clock().now()
        stale = (
            self._last_command_time is None
            or (now - self._last_command_time).nanoseconds / 1e9 > _WATCHDOG_TIMEOUT_SEC
        )
        if stale and not self._watchdog_tripped:
            self.get_logger().warn(
                f"No /pupper/joint_commands for >{_WATCHDOG_TIMEOUT_SEC * 1000:.0f}ms, "
                "cutting servo torque"
            )
            self.servo_driver.deactivate_all()
            self._watchdog_tripped = True

    def destroy_node(self):
        try:
            self.servo_driver.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HardwareNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()