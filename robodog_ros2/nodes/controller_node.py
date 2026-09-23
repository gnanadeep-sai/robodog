"""
The kinematic engine node.

Replaces the legacy `run_robot_x.py` main loop. Where the legacy script
manually spin-waited on `(time.time() - last_loop) < config.dt` (~100Hz,
jitter-prone) and then ran a blocking joystick-poll -> IMU-read ->
`controller.run()` -> hardware-write pipeline in one thread, this node:

  * Initializes `core.config.Configuration` from ROS 2 parameters
    (declared here with the exact legacy defaults, overridable from
    `config/pupper_params.yaml`), instead of a hardcoded Python object.
  * Is driven by a single deterministic `rclpy` timer at `config.dt`
    (default 0.01s / 100Hz) -- never a manual spin-wait.
  * Subscribes to `/joy` (`sensor_msgs/Joy`) and `/imu/data`
    (`sensor_msgs/Imu`) with non-blocking callbacks that just cache the
    latest arrays/orientation; all parsing happens once per timer tick.
  * Delegates joystick parsing to `core.command_interpreter
    .CommandInterpreter`, which integrates body pitch/roll/height using
    the timer's own strict `dt` -- never a message-rate-derived value --
    eliminating the drift bug in the legacy `JoystickInterface.py`.
  * Steps `core.controller.Controller` (unmodified gait/stance/swing/IK
    math) and publishes the resulting 12 joint targets as a timestamped
    `sensor_msgs/JointState` on `/pupper/joint_commands`.

Zero hardware I/O happens in this node -- that is `hardware_node.py`'s
job, decoupled entirely via the `/pupper/joint_commands` topic.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Imu, Joy, JointState

from ..core.command_interpreter import DEFAULT_JOY_MAPPING, CommandInterpreter
from ..core.config import Configuration
from ..core.controller import Controller
from ..core.joint_names import JOINT_NAMES, matrix_to_joint_list
from ..core.kinematics import four_legs_inverse_kinematics
from ..core.state import State

# ---------------------------------------------------------------------
# Raw `Configuration` defaults, exposed as individual ROS 2 parameters.
# Keys match the `params.get(<key>, ...)` lookups in
# `core/config.py::Configuration.__init__` exactly, so the resolved
# dict can be handed straight back in. Kept out of `core/` itself so
# that module stays a plain dataclass-style object with zero `rclpy`
# awareness.
# ---------------------------------------------------------------------
_CONFIG_PARAM_DEFAULTS = {
    "max_x_velocity": 0.4,
    "max_y_velocity": 0.3,
    "max_yaw_rate": 2.0,
    "max_pitch": 30.0 * np.pi / 180.0,
    "z_time_constant": 0.02,
    "z_speed": 0.03,
    "pitch_deadband": 0.02,
    "pitch_time_constant": 0.25,
    "max_pitch_rate": 0.15,
    "roll_speed": 0.16,
    "yaw_time_constant": 0.3,
    "max_stance_yaw": 1.2,
    "max_stance_yaw_rate": 2.0,
    "delta_x": 0.1,
    "delta_y": 0.09,
    "x_shift": 0.0,
    "default_z_ref": -0.16,
    "z_clearance": 0.07,
    "alpha": 0.5,
    "beta": 0.5,
    "dt": 0.01,
    "num_phases": 4,
    "overlap_time": 0.10,
    "swing_time": 0.15,
    "leg_fb": 0.10,
    "leg_lr": 0.04,
    "leg_l2": 0.115,
    "leg_l1": 0.1235,
    "abduction_offset": 0.03,
    "foot_radius": 0.01,
    "hip_l": 0.0394,
    "hip_w": 0.0744,
    "hip_t": 0.0214,
    "hip_offset": 0.0132,
    "body_l": 0.276,
    "body_w": 0.100,
    "body_t": 0.050,
    "frame_mass": 0.560,
    "module_mass": 0.080,
    "leg_mass": 0.030,
}
# Flattened (row-major) default contact-phase matrix; declared
# separately since ROS 2 parameters can't be 2-D. `Configuration`'s
# `_as_matrix()` helper reshapes a flat list back to (4, 4) fine.
_CONTACT_PHASES_DEFAULT = [
    1, 1, 1, 0,
    1, 0, 1, 1,
    1, 0, 1, 1,
    1, 1, 1, 0,
]


class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller_node")

        self.config = self._load_configuration()
        self.joy_mapping = self._load_joy_mapping()

        self.state = State()
        self.command_interpreter = CommandInterpreter(self.config, self.joy_mapping)
        self.controller = Controller(self.config, four_legs_inverse_kinematics)

        # Cached, non-blocking input state -- populated by subscription
        # callbacks, consumed once per deterministic timer tick.
        self._joy_axes = []
        self._joy_buttons = []
        # (w, x, y, z), identity until the first /imu/data message
        # arrives (matches the legacy `np.array([1, 0, 0, 0])` default
        # used when `--use_imu` was not passed).
        self._quat_orientation = np.array([1.0, 0.0, 0.0, 0.0])

        self.joy_sub = self.create_subscription(
            Joy, "/joy", self._joy_callback, QoSPresetProfiles.SYSTEM_DEFAULT.value
        )
        self.imu_sub = self.create_subscription(
            Imu, "/imu/data", self._imu_callback, QoSPresetProfiles.SENSOR_DATA.value
        )
        self.joint_command_pub = self.create_publisher(
            JointState, "/pupper/joint_commands", QoSPresetProfiles.SENSOR_DATA.value
        )

        # Deterministic control-loop timer -- replaces the legacy
        # `while (time.time() - last_loop) < config.dt: continue`
        # spin-wait. `config.dt` itself comes from the resolved ROS 2
        # parameters above (default 0.01s / 100Hz).
        self.timer = self.create_timer(self.config.dt, self._on_timer)

        self.get_logger().info(
            f"controller_node up: dt={self.config.dt:.4f}s "
            f"({1.0 / self.config.dt:.1f}Hz), "
            f"publishing {len(JOINT_NAMES)} joints on /pupper/joint_commands"
        )

    # ------------------------------------------------------------------
    # Parameter loading
    # ------------------------------------------------------------------
    def _load_configuration(self) -> Configuration:
        params = {}
        for key, default in _CONFIG_PARAM_DEFAULTS.items():
            params[key] = self.declare_parameter(key, default).value
        params["contact_phases"] = self.declare_parameter(
            "contact_phases", _CONTACT_PHASES_DEFAULT
        ).value
        # ps4_color / ps4_deactivated_color are nested dicts, which ROS 2
        # parameters can't hold directly -- keep the `Configuration`
        # defaults for those two.
        return Configuration(params=params)

    def _load_joy_mapping(self) -> dict:
        mapping = {}
        for key, default in DEFAULT_JOY_MAPPING.items():
            mapping[key] = self.declare_parameter(f"joy_mapping.{key}", default).value
        return mapping

    # ------------------------------------------------------------------
    # Subscription callbacks -- non-blocking, cache only
    # ------------------------------------------------------------------
    def _joy_callback(self, msg: Joy):
        self._joy_axes = msg.axes
        self._joy_buttons = msg.buttons

    def _imu_callback(self, msg: Imu):
        # ROS convention is (x, y, z, w); `transforms3d` (used by
        # `core.controller`) expects (w, x, y, z).
        q = msg.orientation
        self._quat_orientation = np.array([q.w, q.x, q.y, q.z])

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------
    def _on_timer(self):
        command = self.command_interpreter.update(
            self.state, self._joy_axes, self._joy_buttons, dt=self.config.dt
        )
        self.state.quat_orientation = self._quat_orientation

        self.controller.run(self.state, command)

        self._publish_joint_command()

    def _publish_joint_command(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = matrix_to_joint_list(self.state.joint_angles)
        self.joint_command_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()