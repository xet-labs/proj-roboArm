#!/usr/bin/env python3
import argparse
import re
import evdev
from evdev import InputDevice, categorize, ecodes, list_devices
import serial
import serial.tools.list_ports
import struct
import time
from datetime import datetime

def find_arduino_port():
    ports = serial.tools.list_ports.comports()
    for p in ports:
        if "USB" in p.device or "ACM" in p.device:
            return p.device
    return None

parser = argparse.ArgumentParser(description="Process command-line arguments.")
parser.add_argument("--port", type=str, default="", help="Serial port (default: auto-detect)")
parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")

args = parser.parse_args()

# -------- Configuration --------
# Locate the Xbox controller (search by name using regex for "xbox", "x-box", or "xbox360")
devices = [InputDevice(path) for path in list_devices()]
controller = None
for dev in devices:
    if re.search(r"x[- ]?box(?:360)?", dev.name, re.IGNORECASE):
        controller = dev
        break

if controller is None:
    print("Xbox controller not found!")
    exit(1)

print("Controller:", controller.name, controller.path)

# Set up Serial Connection
port = args.port if args.port else find_arduino_port()
if not port:
    print("Error: Could not find Arduino/ESP32 serial port automatically.")
    exit(1)

print(f"Connecting to ESP32 on {port} at {args.baud} baud...")
ser = serial.Serial(port, args.baud, timeout=0.1)
time.sleep(2)  # Wait for Arduino to reset
print("Connected!\n")

# -------- Controller Data Format --------
# The data packet contains:
#   - 2 magic bytes for UART sync: 0xAA, 0x55
#   - 16-bit unsigned "buttons" bitmask (each bit represents one button/d-pad)
#   - 4 signed 16-bit values for analog stick axes: LX, LY, RX, RY
#   - 2 unsigned 8-bit values for triggers: LT and RT
# Total size: 14 bytes, packed with format '<BBHhhhhBB'

# -------- Controller State --------
state = {
    "buttons": 0,
    "LX": 0,
    "LY": 0,
    "RX": 0,
    "RY": 0,
    "LT": 0,
    "RT": 0,
    "PWR": 0
}

# Mapping of digital button events to bit positions:
button_map = {
    ecodes.BTN_SOUTH: 0,
    ecodes.BTN_EAST: 1,
    ecodes.BTN_NORTH: 2,
    ecodes.BTN_WEST: 3,
    ecodes.BTN_TL: 4,
    ecodes.BTN_TR: 5,
    ecodes.BTN_SELECT: 6,
    ecodes.BTN_START: 7,
    ecodes.BTN_THUMBL: 8,
    ecodes.BTN_THUMBR: 9,
    ecodes.BTN_MODE: 14
}

# Mapping of analog axes to our state keys:
axis_map = {
    ecodes.ABS_X: "LX",    # left stick X
    ecodes.ABS_Y: "LY",    # left stick Y
    ecodes.ABS_RX: "RX",   # right stick X
    ecodes.ABS_RY: "RY",   # right stick Y
    ecodes.ABS_Z: "LT",    # left trigger
    ecodes.ABS_RZ: "RT"    # right trigger
}

# D-Pad events (ABS_HAT0X and ABS_HAT0Y) update the buttons bitmask:
def update_dpad(value, code):
    global state
    if code == ecodes.ABS_HAT0X:
        if value == -1:
            state["buttons"] |= (1 << 12)  # D-pad Left
            state["buttons"] &= ~(1 << 13)
        elif value == 1:
            state["buttons"] |= (1 << 13)  # D-pad Right
            state["buttons"] &= ~(1 << 12)
        else:
            state["buttons"] &= ~((1 << 12) | (1 << 13))
    elif code == ecodes.ABS_HAT0Y:
        if value == -1:
            state["buttons"] |= (1 << 10)  # D-pad Up
            state["buttons"] &= ~(1 << 11)
        elif value == 1:
            state["buttons"] |= (1 << 11)  # D-pad Down
            state["buttons"] &= ~(1 << 10)
        else:
            state["buttons"] &= ~((1 << 10) | (1 << 11))

# Function to print state with timestamp and zero-padded values
def print_state(state, raw_packet=None, raw_event=None):
    ts = datetime.now().strftime("%H:%M:%S.%f")[:-2]
    button_names = {
        0: "A", 1: "B", 2: "X", 3: "Y",
        4: "LB", 5: "RB", 6: "Back", 7: "Start",
        8: "LStick", 9: "RStick", 10: "DPadUp", 11: "DPadDown",
        12: "DPadLeft", 13: "DPadRight"
    }
    pressed = [name for bit, name in button_names.items() if state["buttons"] & (1 << bit)]
    
    print("{}   LX: {:06d}   LY: {:06d}   RX: {:06d}   RY: {:06d}   LT: {:03d}   RT: {:03d}   Buttons: 0x{:04X} ({})".format(
        ts, state["LX"], state["LY"], state["RX"], state["RY"], state["LT"], state["RT"], state["buttons"], 
        ", ".join(pressed) if pressed else "None"
    ))

    raw_info = ""
    if raw_packet:
        raw_info += f"   Tx: {raw_packet.hex()}"
    if raw_event:
        raw_info += f" | EVENT: type={raw_event.type} code={raw_event.code} value={raw_event.value}"
    if raw_info:
        print(raw_info)


# -------- Main Loop --------
try:
    for event in controller.read_loop():
        # Read incoming ESP32 debug logs non-blocking (fetch ALL available bytes instantly)
        if ser.in_waiting:
            try:
                # Read chunks directly without waiting for '\n' to prevent locking the loop
                chunk = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
                print(chunk, end='', flush=True)
            except Exception:
                pass

        raw_event = event  # Store raw event for logging
        if event.type == ecodes.EV_KEY:
            if event.code in button_map:
                bit = button_map[event.code]
                if event.value == 1:  # Button pressed
                    state["buttons"] |= (1 << bit)
                elif event.value == 0:  # Button released
                    state["buttons"] &= ~(1 << bit)
            continue # Don't transmit yet, wait for EV_SYN
            
        elif event.type == ecodes.EV_ABS:
            if event.code in axis_map:
                val = event.value
                # Apply deadzone for analog sticks
                if event.code in (ecodes.ABS_X, ecodes.ABS_Y, ecodes.ABS_RX, ecodes.ABS_RY):
                    if abs(val) < 128:
                        val = 0
                state[axis_map[event.code]] = val
            elif event.code in (ecodes.ABS_HAT0X, ecodes.ABS_HAT0Y):
                update_dpad(event.value, event.code)
            continue # Don't transmit yet, wait for EV_SYN
        
        elif event.type == ecodes.EV_SYN:
            # ONLY transmit when the controller signals the state update batch is completely finished!
            # Pack the state into a 14-byte UART packet (little-endian + magic 0xAA 0x55)
            packet = struct.pack('<BBHhhhhBB',
                                 0xAA, 0x55,
                                 state["buttons"],
                                 state["LX"],
                                 state["LY"],
                                 state["RX"],
                                 state["RY"],
                                 state["LT"],
                                 state["RT"])

            ser.write(packet)
            ser.flush()
            
            # Print state, raw event, and raw packet - matching the UDP script log exactly
            print_state(state, packet)
        # print_state(state, packet, raw_event)

except KeyboardInterrupt:
    print("\nStopping...")
