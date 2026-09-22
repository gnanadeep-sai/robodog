import time
import threading
import evdev
from UDPComms import Publisher, Subscriber, timeout

MESSAGE_RATE = 20

joystick_pub = Publisher(8830)
joystick_subscriber = Subscriber(8840, timeout=0.01)

# Initialize state dictionary (matching PS4 expected payload)
state = {
    "ly": 0.0, "lx": 0.0, "rx": 0.0, "ry": 0.0,
    "L2": 0.0, "R2": 0.0,
    "R1": 0, "L1": 0,
    "dpady": 0, "dpadx": 0,
    "x": 0, "square": 0, "circle": 0, "triangle": 0,
    "message_rate": MESSAGE_RATE,
}

def map_axis(val, max_val=32768.0):
    return val / max_val

def read_xbox_gamepad():
    # Find the Xbox controller
    devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
    gamepad = None
    for dev in devices:
        name = dev.name.lower()
        if 'xbox' in name or 'microsoft' in name or 'pad' in name:
            gamepad = dev
            break

    if not gamepad:
        print("Xbox controller not found!")
        return

    print(f"Connected to Xbox controller: {gamepad.name}")

    # Continuously poll the controller
    for event in gamepad.read_loop():
        if event.type == evdev.ecodes.EV_KEY:
            val = event.value
            # Map Xbox buttons to PS4 equivalents
            if event.code in [evdev.ecodes.BTN_A, evdev.ecodes.BTN_SOUTH]:
                state["x"] = val
            elif event.code in [evdev.ecodes.BTN_B, evdev.ecodes.BTN_EAST]:
                state["circle"] = val
            elif event.code in [evdev.ecodes.BTN_X, evdev.ecodes.BTN_WEST]:
                state["square"] = val
            elif event.code in [evdev.ecodes.BTN_Y, evdev.ecodes.BTN_NORTH]:
                state["triangle"] = val
            elif event.code == evdev.ecodes.BTN_TL:
                state["L1"] = val
            elif event.code == evdev.ecodes.BTN_TR:
                state["R1"] = val

        elif event.type == evdev.ecodes.EV_ABS:
            val = event.value
            if event.code == evdev.ecodes.ABS_X:
                state["lx"] = map_axis(val)
            elif event.code == evdev.ecodes.ABS_Y:
                state["ly"] = -map_axis(val) # Invert Y for correct forward/back
            elif event.code == evdev.ecodes.ABS_RX:
                state["rx"] = map_axis(val)
            elif event.code == evdev.ecodes.ABS_RY:
                state["ry"] = -map_axis(val) # Invert Y
            elif event.code == evdev.ecodes.ABS_Z:
                state["L2"] = val / 255.0 # Triggers map 0-1
            elif event.code == evdev.ecodes.ABS_RZ:
                state["R2"] = val / 255.0
            elif event.code == evdev.ecodes.ABS_HAT0X:
                state["dpadx"] = val
            elif event.code == evdev.ecodes.ABS_HAT0Y:
                state["dpady"] = -val

# Launch the listener thread so it doesn't block the UDP publisher
listener_thread = threading.Thread(target=read_xbox_gamepad, daemon=True)
listener_thread.start()

# Main publisher loop
while True:
    joystick_pub.send(state)
    
    try:
        # Attempt to read to clear buffer, though Xbox controllers don't use the LED color data
        msg = joystick_subscriber.get()
    except timeout:
        pass
        
    time.sleep(1 / MESSAGE_RATE)
