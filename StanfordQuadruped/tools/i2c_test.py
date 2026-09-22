import time
import board
import busio
from adafruit_pca9685 import PCA9685

# Ensure you have the required libraries installed:
# pip install adafruit-circuitpython-pca9685 adafruit-circuitpython-motor

try:
    from adafruit_motor import servo
    has_motor_lib = True
except ImportError:
    has_motor_lib = False
    print("adafruit_motor not found. Using raw duty cycle instead.")
    print("To install: pip install adafruit-circuitpython-motor")

SPEED_DEG_PER_SEC = 20.0
UPDATE_RATE_HZ = 50.0

def move_servo_smooth(pca, channel, start_angle, target_angle, servos=None):
    if start_angle == target_angle:
        return
        
    delta_angle = target_angle - start_angle
    duration = abs(delta_angle) / SPEED_DEG_PER_SEC
    steps = int(duration * UPDATE_RATE_HZ)
    
    if steps == 0:
        steps = 1
        
    sleep_time = 1.0 / UPDATE_RATE_HZ
    
    for i in range(1, steps + 1):
        current = start_angle + (delta_angle * i / steps)
        if has_motor_lib and servos:
            servos[channel].angle = current
        else:
            pulse_ms = 1.0 + (current / 180.0)
            duty_cycle = int((pulse_ms / 20.0) * 65535)
            pca.channels[channel].duty_cycle = duty_cycle
        time.sleep(sleep_time)

def main():
    print("Initializing I2C and PCA9685...")
    try:
        i2c = busio.I2C(board.SCL, board.SDA)
        pca = PCA9685(i2c)
        pca.frequency = 50
    except ValueError as e:
        print(f"Error initializing I2C: {e}")
        print("Make sure I2C is enabled on your Raspberry Pi and the PCA9685 is connected.")
        return
    except Exception as e:
        print(f"Unexpected error: {e}")
        return

    # Assuming 12 servos for a quadruped, connected to channels 0-11
    servos = None
    if has_motor_lib:
        servos = [servo.Servo(pca.channels[i]) for i in range(12)]
        
    current_angles = [90.0] * 12
    print("\nPCA9685 12-Servo Manual Test")
    print("----------------------------")
    print("This script allows you to test each of the 12 servos individually.")
    
    while True:
        try:
            print("\nOptions:")
            print("  0-11: Select servo channel to test")
            print("  a: Test all 12 servos sequentially")
            print("  q: Quit")
            
            user_input = input("Enter option: ").strip().lower()
            
            if user_input == 'q':
                break
            elif user_input == 'a':
                test_all(pca)
                continue
                
            channel = int(user_input)
            if not (0 <= channel <= 11):
                print("Invalid channel. Please enter a number between 0 and 11.")
                continue
                
            angle_input = input(f"Enter angle for servo {channel} (0-180) or 'n' for neutral (90): ").strip().lower()
            if angle_input == 'n':
                angle = 90.0
            else:
                angle = float(angle_input)
            
            if not (0 <= angle <= 180):
                print("Invalid angle. Please enter a number between 0 and 180.")
                continue
                
            print(f"Moving servo {channel} from {current_angles[channel]:.1f} to {angle:.1f} degrees smoothly...")
            move_servo_smooth(pca, channel, current_angles[channel], angle, servos)
            current_angles[channel] = angle
                
            print(f"Servo {channel} set to {angle} degrees.")
            
        except ValueError:
            print("Invalid input.")
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"Error communicating with PCA9685: {e}")

    pca.deinit()
    print("\nExiting and deinitializing PCA9685.")

def test_all(pca):
    print("\nTesting all 12 servos...")
    if has_motor_lib:
        servos = [servo.Servo(pca.channels[i]) for i in range(12)]
    
    for i in range(12):
        print(f"Moving servo {i} to 90 degrees (neutral)...")
        if has_motor_lib:
            servos[i].angle = 90
        else:
            pulse_ms = 1.5
            duty_cycle = int((pulse_ms / 20.0) * 65535)
            pca.channels[i].duty_cycle = duty_cycle
        time.sleep(0.2)

if __name__ == '__main__':
    main()
