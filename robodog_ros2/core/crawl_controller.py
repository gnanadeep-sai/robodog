import numpy as np

from .command import Command
from .swing_leg_controller import SwingController
from .stance_controller import StanceController

# Leg indices: 0 = front-right, 1 = front-left, 2 = back-right, 3 = back-left
# FR -> BL -> FL -> BR keeps the swing legs alternating diagonally.
CRAWL_ORDER = [0, 3, 1, 2]

SLOT_TIME = 0.4  # seconds each leg gets (lift, step, put down). Full cycle = 4 x this
MAX_LEAN = 0.06  # metres, limit on how far the body leans over the planted feet
SPEED_SCALE = 0.15  # fraction of max_x/y_velocity used while crawling
YAW_SCALE = 0.3  # fraction of max_yaw_rate used while crawling
INPUT_THRESHOLD = 0.1  # stick must pass 10% of max to count as input


def _smoothstep(x):
    x = min(max(x, 0.0), 1.0)
    return x * x * (3.0 - 2.0 * x)


class CrawlController:
    """Crawl gait: one leg steps at a time, the other three stay planted.

    The gait only advances while there is joystick input. When the stick is
    released the leg that is currently stepping finishes, then everything
    stops and holds still.
    """

    def __init__(self, config):
        self.config = config
        self.stance_controller = StanceController(config)
        self.swing_controller = SwingController(config)
        self.slot_ticks = int(SLOT_TIME / config.dt)
        self.tick = 0  # only counts while crawling, not every control tick

    def reset(self):
        self.tick = 0

    def _scaled_command(self, command):
        """Returns (command used for crawling, True if the stick is pushed)."""
        c = self.config
        push = max(
            abs(command.horizontal_velocity[0]) / c.max_x_velocity,
            abs(command.horizontal_velocity[1]) / c.max_y_velocity,
            abs(command.yaw_rate) / c.max_yaw_rate,
        )
        crawl_command = Command()
        crawl_command.height = command.height
        if push > INPUT_THRESHOLD:
            crawl_command.horizontal_velocity = command.horizontal_velocity * SPEED_SCALE
            crawl_command.yaw_rate = command.yaw_rate * YAW_SCALE
            return crawl_command, True
        return crawl_command, False

    def _touchdown(self, leg_index, command):
        """Where the stepping foot should land. It will stay planted for
        3 slots, so aim half of that travel ahead of the default position.
        """
        c = self.config
        stance_time = 3 * self.slot_ticks * c.dt
        delta_xy = c.alpha * stance_time * command.horizontal_velocity
        theta = c.beta * stance_time * command.yaw_rate
        c_, s_ = np.cos(theta), np.sin(theta)
        x, y = c.default_stance[0, leg_index], c.default_stance[1, leg_index]
        return np.array([c_ * x - s_ * y + delta_xy[0], s_ * x + c_ * y + delta_xy[1], 0.0])

    def step(self, state, command):
        """Calculate foot locations for the next timestep.

        Returns
        -------
        (Numpy array (3, 4), Numpy array (3, 4))
            New foot locations, and a body-lean offset to add to them
            just before inverse kinematics (not stored in the state).
        """
        c = self.config
        crawl_command, has_input = self._scaled_command(command)

        slot_tick = self.tick % self.slot_ticks
        stepping_leg = CRAWL_ORDER[(self.tick // self.slot_ticks) % 4]

        # Wait at a slot boundary until there is input, but never stop mid-step.
        if slot_tick == 0 and not has_input:
            return state.foot_locations.copy(), np.zeros((3, 4))

        quarter = self.slot_ticks // 4
        swing_ticks = self.slot_ticks - 2 * quarter

        new_foot_locations = np.zeros((3, 4))
        for leg in range(4):
            if leg != stepping_leg or not (quarter <= slot_tick < quarter + swing_ticks):
                new_foot_locations[:, leg] = self.stance_controller.next_foot_location(
                    leg, state, crawl_command
                )
                continue
            # stepping leg, in the air
            swing_prop = (slot_tick - quarter) / swing_ticks
            foot = state.foot_locations[:, leg]
            time_left = c.dt * swing_ticks * (1.0 - swing_prop)
            v = (self._touchdown(leg, crawl_command) - foot) / time_left * np.array([1, 1, 0])
            z = self.swing_controller.swing_height(swing_prop) + crawl_command.height
            new_foot_locations[:, leg] = foot * np.array([1, 1, 0]) + np.array([0, 0, z]) + v * c.dt

        # Lean the body toward the middle of the three planted feet: ramp up,
        # hold while the leg is in the air, ramp down.
        planted = [leg for leg in range(4) if leg != stepping_leg]
        centre = state.foot_locations[:2, planted].mean(axis=1)
        centre = np.clip(centre, -MAX_LEAN, MAX_LEAN)
        lean = min(_smoothstep(slot_tick / quarter),
                   _smoothstep((self.slot_ticks - slot_tick) / quarter))
        body_offset = np.zeros((3, 4))
        body_offset[:2, :] = (-centre * lean)[:, np.newaxis]

        self.tick += 1
        return new_foot_locations, body_offset
