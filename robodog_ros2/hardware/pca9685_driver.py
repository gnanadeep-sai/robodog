"""
Raw pigpio PCA9685 driver.

Pure hardware transport layer -- no ROS imports. Talks to the PCA9685
PWM driver IC over I2C via the `pigpio` daemon (`pigpiod`) using
low-level register writes (`pigpio.i2c_write_i2c_block_data`), replacing
the legacy `adafruit_pca9685` / `busio` (CircuitPython) transport.

Requires the pigpio daemon to be running on the Pi:
    sudo pigpiod
"""
import math
import time

import pigpio

# ---------------------------------------------------------------------
# PCA9685 register map
# ---------------------------------------------------------------------
_MODE1 = 0x00
_MODE2 = 0x01
_LED0_ON_L = 0x06
_ALL_LED_ON_L = 0xFA
_PRESCALE = 0xFE

_MODE1_RESTART = 0x80
_MODE1_AUTO_INCREMENT = 0x20
_MODE1_SLEEP = 0x10
_MODE1_ALLCALL = 0x01

_MODE2_OUTDRV = 0x04

_OSC_CLOCK_HZ = 25_000_000.0
_PWM_STEPS = 4096  # 12-bit counter


class PCA9685Driver:
    """Thin, dependency-free wrapper around the PCA9685 I2C register
    protocol, transported over `pigpio`.
    """

    def __init__(self, i2c_bus: int = 1, i2c_address: int = 0x40, frequency_hz: float = 50.0,
                 pi: "pigpio.pi" = None):
        self.i2c_bus = i2c_bus
        self.i2c_address = i2c_address
        self.frequency_hz = frequency_hz

        self._owns_pi = pi is None
        self.pi = pi if pi is not None else pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError(
                "Could not connect to pigpiod. Is the daemon running? "
                "Start it with `sudo pigpiod`."
            )

        self.handle = self.pi.i2c_open(self.i2c_bus, self.i2c_address)
        self._initialize()

    # ------------------------------------------------------------------
    # Low-level register access
    # ------------------------------------------------------------------
    def _write_byte(self, register: int, value: int):
        self.pi.i2c_write_byte_data(self.handle, register, value & 0xFF)

    def _read_byte(self, register: int) -> int:
        return self.pi.i2c_read_byte_data(self.handle, register)

    def _write_block(self, register: int, data: list):
        self.pi.i2c_write_i2c_block_data(self.handle, register, data)

    # ------------------------------------------------------------------
    # Setup
    # ------------------------------------------------------------------
    def _initialize(self):
        # Wake, enable auto-increment and all-call so we can address
        # multiple channels with sequential block writes.
        self._write_byte(_MODE1, _MODE1_ALLCALL)
        self._write_byte(_MODE2, _MODE2_OUTDRV)
        time.sleep(0.005)

        mode1 = self._read_byte(_MODE1)
        mode1 = mode1 & ~_MODE1_SLEEP  # wake up
        self._write_byte(_MODE1, mode1)
        time.sleep(0.005)

        self.set_pwm_freq(self.frequency_hz)
        self.all_off()

    def set_pwm_freq(self, freq_hz: float):
        """Sets the PWM frequency in Hz, following the standard PCA9685
        sleep -> prescale -> restart sequence.
        """
        self.frequency_hz = freq_hz
        prescale_val = _OSC_CLOCK_HZ / _PWM_STEPS / float(freq_hz) - 1.0
        prescale = int(math.floor(prescale_val + 0.5))

        old_mode = self._read_byte(_MODE1)
        sleep_mode = (old_mode & 0x7F) | _MODE1_SLEEP
        self._write_byte(_MODE1, sleep_mode)  # must sleep to change prescale
        self._write_byte(_PRESCALE, prescale)
        self._write_byte(_MODE1, old_mode)
        time.sleep(0.005)
        self._write_byte(_MODE1, old_mode | _MODE1_RESTART | _MODE1_AUTO_INCREMENT)

    # ------------------------------------------------------------------
    # Channel control
    # ------------------------------------------------------------------
    def set_channel_on_off(self, channel: int, on: int, off: int):
        """Sets the raw 12-bit ON/OFF counter pair for one channel
        (0-4095 each), via a single 4-byte auto-incrementing block write
        starting at that channel's ON_L register.
        """
        on &= 0x0FFF
        off &= 0x0FFF
        register = _LED0_ON_L + 4 * channel
        data = [on & 0xFF, (on >> 8) & 0xFF, off & 0xFF, (off >> 8) & 0xFF]
        self._write_block(register, data)

    def set_channel_duty_cycle(self, channel: int, duty_cycle_16bit: int):
        """Sets a channel's duty cycle from a 16-bit value (0-65535), the
        same resolution the legacy CircuitPython `PWMChannel.duty_cycle`
        API used, so `hardware/servo_driver.py`'s angle-to-duty-cycle math
        can be reused unmodified. Internally rescaled to the PCA9685's
        native 12-bit (0-4095) OFF count, with ON always at 0.
        """
        duty_cycle_16bit = max(0, min(65535, int(duty_cycle_16bit)))
        off = duty_cycle_16bit >> 4  # 65536 / 4096 == 16
        self.set_channel_on_off(channel, on=0, off=off)

    def channel_off(self, channel: int):
        self.set_channel_on_off(channel, on=0, off=0)

    def all_off(self):
        """Zeroes every channel in one block write via the PCA9685
        ALL_LED registers -- used for the safety-watchdog torque cut.
        """
        self._write_block(_ALL_LED_ON_L, [0, 0, 0, 0x10])  # OFF bit 12 (full off)

    def close(self):
        try:
            self.all_off()
        finally:
            self.pi.i2c_close(self.handle)
            if self._owns_pi:
                self.pi.stop()