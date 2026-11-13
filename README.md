# robodog

ROS 2 (Jazzy) control stack for a quadruped robot dog.

## Usage

### How it works

```
gamepad ──► joy_node ──/joy──► controller_node ──/pupper/joint_commands──► hardware_node ──I2C──► PCA9685 ──► 12 servos
```

| Piece | What it does |
|---|---|
| `joy_node` (upstream `joy` pkg) | Reads the gamepad, publishes `/joy` |
| `controller_node` | 100 Hz timer. Joystick → command → gait/stance/IK → publishes 12 joint angles on `/pupper/joint_commands`. No hardware access. |
| `hardware_node` | Subscribes to `/pupper/joint_commands`, converts angle → PWM, writes to the PCA9685. Echoes angles on `/joint_states`. |
| `calibration_cli` | Interactive bench tool to find each servo's neutral angle. No ROS needed. |
| `i2c_diagnostics` | Bench tool to check the PCA9685 is reachable and jog each channel. No ROS needed. |

Repo layout:

```
robodog-ros2/
├── .gitignore
├── README.md
├── package.xml
├── setup.cfg
├── setup.py
├── Dockerfile                    
├── entrypoint.sh                 
├── compose.yaml                  
├── config/
│   └── pupper_params.yaml        
├── launch/
│   └── robodog.launch.py         
├── resource/
│   └── robodog_ros2
└── robodog_ros2/
    ├── __init__.py
    ├── core/                     
    │   ├── __init__.py
    │   ├── command.py
    │   ├── command_interpreter.py
    │   ├── config.py
    │   ├── controller.py
    │   ├── gaits.py
    │   ├── joint_names.py
    │   ├── kinematics.py
    │   ├── stance_controller.py
    │   ├── state.py
    │   ├── swing_leg_controller.py
    │   └── utilities.py
    ├── hardware/                 
    │   ├── __init__.py
    │   ├── calibration_cli.py
    │   ├── i2c_diagnostics.py
    │   ├── pca9685_driver.py
    │   └── servo_driver.py
    └── nodes/                    
        ├── __init__.py
        ├── controller_node.py
        └── hardware_node.py
```

### Prerequisites

- Raspberry Pi with I2C enabled (`sudo raspi-config` → Interface Options → I2C)
- PCA9685 on I2C bus 1, address 0x40 (check: `i2cdetect -y 1`)
- `pigpiod` running **on the host**: `sudo systemctl enable --now pigpiod`
- Docker with the compose plugin (the container supplies ROS 2 Jazzy and all Python deps)
- A gamepad that shows up as `/dev/input/js0`

### Docker setup

Clone the repo into the `src` folder of a workspace:

```bash
mkdir -p robodog_ws/src
cd robodog_ws/src
git clone <repo-url> robodog-ros2
cd robodog-ros2
```
From
the repo root (`robodog_ws/src/robodog-ros2/`):
```bash
docker compose build              # first time only (or after editing the Dockerfile)
docker compose run --rm --name robodog robodog   # shell with ROS sourced and the package built
```
 
For a second terminal in the running container:
 
```bash
docker exec -it robodog bash
```

After adding new files or changing `setup.py`, rebuild inside the container:
```bash
cd /ros2_ws
colcon build --symlink-install
source install/setup.bash
```

To start completely fresh (wipe the build volumes): 
```bash
docker compose down -v
```

### Hardware Setup

Do these with the robot **on a stand, legs off the ground**.

**Step 1 – check the I2C link**
```bash
i2cdetect -y 1                                  # expect 40 in the grid
python3 -m robodog_ros2.hardware.i2c_diagnostics
```
Jog each channel 0–11 and confirm the right servo moves.

**Step 2 – calibrate neutral angles**
```bash
python3 -m robodog_ros2.hardware.calibration_cli
```
For each of the 12 motors, press `a`/`b` to nudge the link until it is
horizontal/vertical as prompted, `d` when done, then confirm.

**Step 3 – launch**
```bash
ros2 launch robodog_ros2 robodog.launch.py
```

### Controls

| Input | Action |
|---|---|
| L1 | Activate / deactivate |
| R1 | Toggle trot |
| X / A | Hop |
| Left stick | Walk: forward/back (Y), strafe (X) |
| Right stick X | Yaw rate |
| Right stick Y | Body pitch |
| D-pad up/down | Body height |
| D-pad left/right | Body roll |

Button/axis indices are set under `joy_mapping.*` in
`config/pupper_params.yaml`. Check yours with `ros2 topic echo /joy` and
edit the YAML to match; no code changes needed.

### Configuration

Everything lives in `config/pupper_params.yaml`:

- `controller_node`: velocity limits, stance/swing/gait timing, leg geometry,
  masses, `joy_mapping.*`
- `hardware_node`: `i2c_bus`, `i2c_address` (decimal, 64 = 0x40), `pwm.*`,
  and per-robot servo calibration (`servo.*`)

Pass a modified copy with `params_file:=...`. Runtime tweak example:
`ros2 param set /controller_node max_x_velocity 0.2` (parameters are read at
startup, so restart the node for most changes to take effect).

### Safety behaviour

`hardware_node` starts with all servo outputs off. If no
`/pupper/joint_commands` message arrives for more than 30 ms (controller
crash, topic dropout), a watchdog cuts all 12 PWM channels. Torque returns
automatically when commands resume. Ctrl-C also zeroes all channels on exit.

### Debugging

| Symptom | Likely cause |
|---|---|
| `Could not connect to pigpiod` | `pigpiod` not running on the host: `sudo systemctl start pigpiod` |
| `i2c_open` / permission error | Missing `/dev/i2c-1`: enable I2C, and start the container with `--privileged` and `/dev` mounted (compose file does this) |
| No `/joy` messages | Wrong device; check `ls /dev/input/js*` and pass `joy_dev:=` |
| Robot does nothing after launch | Press L1 to activate, then R1 for trot |
| Legs twitch / wrong direction | Re-run calibration; check `servo.servo_multipliers` |

## Project Timeline

- [x] Calibrate the robodog and achieve a basic trot gait
- [x] Port the code to ROS 2
- [ ] Develop improved gait planning without sensor feedback on the old robodog
- [ ] Implement reinforcement learning for locomotion on the new robodog using sensor feedback
- [ ] Enable stable locomotion on uneven terrain

We plan to do the documentation of the project parallelly with the development.