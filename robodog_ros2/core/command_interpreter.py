"""
Replaces the legacy `src/JoystickInterface.py`.

The legacy implementation parsed a UDP dict (`msg["ly"]`, `msg["R1"]`,
...) and integrated body pitch/roll/height by a `message_dt` derived from
the *variable* UDP message rate reported inside the payload itself. That
coupling meant that a jittery or dropped-packet network connection
directly corrupted the kinematic integration (drift).

Here, `CommandInterpreter.update()` is a pure function: it takes the most
recently cached `sensor_msgs/Joy` `axes` / `buttons` arrays (cached
non-blocking by the ROS 2 subscription callback in
`nodes/controller_node.py`) plus the *current* `State`, and always
integrates using the controller's deterministic timer period
(`config.dt`, e.g. exactly 0.01s from `self.create_timer(0.01)`), which
guarantees zero drift regardless of `/joy` publish rate or jitter.

No rclpy imports here -- this stays pure/unit-testable, per the
core/hardware/nodes partitioning requirement.
"""
import numpy as np

from .command import Command
from .utilities import deadband, clipped_first_order_filter

# Default index mapping for a standard Linux `joy` driver PS4/Xbox-style
# gamepad. Overridable via the `joy_mapping` ROS 2 parameter dict so this
# adapts to whatever controller/driver combination is actually plugged in,
# without touching code.
DEFAULT_JOY_MAPPING = {
    # --- axes ---
    "axis_lx": 0,
    "axis_ly": 1,
    "axis_rx": 3,
    "axis_ry": 4,
    "axis_dpad_x": 6,
    "axis_dpad_y": 7,
    # --- buttons (edge-triggered) ---
    "button_trot": 5,  # R1
    "button_hop": 0,  # X / A
    "button_activate": 4,  # L1
}


class CommandInterpreter:
    def __init__(self, config, joy_mapping: dict = None):
        self.config = config
        self.mapping = dict(DEFAULT_JOY_MAPPING)
        if joy_mapping:
            self.mapping.update(joy_mapping)

        self.previous_trot_toggle = 0
        self.previous_hop_toggle = 0
        self.previous_activate_toggle = 0

    @staticmethod
    def _get(seq, index, default=0.0):
        """Defensive indexing: a `/joy` message that hasn't reported this
        many axes/buttons yet (e.g. before the first full frame) yields
        the default instead of raising.
        """
        if seq is None or index is None or index >= len(seq):
            return default
        return seq[index]

    def update(self, state, axes, buttons, dt=None):
        """Compute a Command from the latest cached joystick arrays.

        Parameters
        ----------
        state : core.state.State
            Current robot state (used for filtered integration, exactly
            as in the legacy JoystickInterface).
        axes : sequence[float]
            Latest sensor_msgs/Joy.axes (cached by the ROS subscription).
        buttons : sequence[int]
            Latest sensor_msgs/Joy.buttons.
        dt : float, optional
            Integration timestep. Defaults to config.dt (the
            deterministic controller-timer period) -- NOT any
            message-rate-derived value.

        Returns
        -------
        Command
        """
        dt = self.config.dt if dt is None else dt
        m = self.mapping
        command = Command()

        ####### Handle discrete (edge-triggered) commands ########
        trot_toggle = self._get(buttons, m["button_trot"])
        command.trot_event = trot_toggle == 1 and self.previous_trot_toggle == 0

        hop_toggle = self._get(buttons, m["button_hop"])
        command.hop_event = hop_toggle == 1 and self.previous_hop_toggle == 0

        activate_toggle = self._get(buttons, m["button_activate"])
        command.activate_event = (
            activate_toggle == 1 and self.previous_activate_toggle == 0
        )

        self.previous_trot_toggle = trot_toggle
        self.previous_hop_toggle = hop_toggle
        self.previous_activate_toggle = activate_toggle

        ####### Handle continuous commands ########
        ly = self._get(axes, m["axis_ly"])
        lx = self._get(axes, m["axis_lx"])
        rx = self._get(axes, m["axis_rx"])
        ry = self._get(axes, m["axis_ry"])
        dpad_x = self._get(axes, m["axis_dpad_x"])
        dpad_y = self._get(axes, m["axis_dpad_y"])

        x_vel = ly * self.config.max_x_velocity
        y_vel = lx * -self.config.max_y_velocity
        command.horizontal_velocity = np.array([x_vel, y_vel])
        command.yaw_rate = rx * -self.config.max_yaw_rate

        pitch = ry * self.config.max_pitch
        deadbanded_pitch = deadband(pitch, self.config.pitch_deadband)
        pitch_rate = clipped_first_order_filter(
            state.pitch,
            deadbanded_pitch,
            self.config.max_pitch_rate,
            self.config.pitch_time_constant,
        )
        command.pitch = state.pitch + dt * pitch_rate

        height_movement = dpad_y
        command.height = state.height - dt * self.config.z_speed * height_movement

        roll_movement = -dpad_x
        command.roll = state.roll + dt * self.config.roll_speed * roll_movement

        return command