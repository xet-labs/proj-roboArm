#!/usr/bin/env python3
import serial
import serial.tools.list_ports
import struct
import time

class JoypadUART:
    def __init__(self, port=None, baudrate=115200):
        if port is None:
            port = self.find_arduino_port()
            if not port:
                raise RuntimeError("Could not find Arduino/ESP32 COM port automatically.")
        
        print(f"Connecting to ESP32 on {port} at {baudrate} baud...")
        self.ser = serial.Serial(port, baudrate, timeout=0.1)
        time.sleep(2)  # Wait for Arduino to reset upon connection
        print("Connected!")

    def find_arduino_port(self):
        """Attempts to auto-detect a USB serial port."""
        ports = serial.tools.list_ports.comports()
        for p in ports:
            if "USB" in p.device or "ACM" in p.device:
                return p.device
        return None

    def send_state(self, btns=0, lx=0, ly=0, rx=0, ry=0, lt=0, rt=0):
        """
        Packs Joypad state into exactly 14 bytes:
        [0xAA, 0x55] (Header) + [12 bytes State payload]
        Memory layout of btn::Joypad::State:
        - btns (uint16_t) -> H
        - axisLX (int16_t) -> h
        - axisLY (int16_t) -> h
        - axisRX (int16_t) -> h
        - axisRY (int16_t) -> h
        - triggerLT (uint8_t) -> B
        - triggerRT (uint8_t) -> B
        """
        # < for Little Endian
        # B = unsigned char (1 byte)
        # H = unsigned short (2 bytes)
        # h = signed short (2 bytes)
        packet = struct.pack('<BBHhhhhBB', 
                             0xAA, 0x55,    # Magic Header
                             btns, lx, ly, rx, ry, lt, rt)
        
        self.ser.write(packet)
        self.ser.flush()

    def read_debug_logs(self):
        """Reads and prints any incoming debug information from the ESP32."""
        while self.ser.in_waiting:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print(f"[ESP32] {line}")
            except Exception as e:
                pass


if __name__ == "__main__":
    try:
        joypad = JoypadUART()
        print("Starting dummy transmission. Press Ctrl+C to stop.")
        
        # Example Loop: Send dummy data
        while True:
            # Send 'Button A' (1) and 'Button X' (4) = 5
            # Push LX axis forward slightly (5000)
            # Pull Right Trigger fully (255)
            joypad.send_state(btns=5, lx=5000, rt=255)
            
            # Print whatever stats the ESP32 sends back to PC
            joypad.read_debug_logs()
            
            # 15ms interval is ~66 Hz, fast enough for responsive control
            time.sleep(0.015) 

    except KeyboardInterrupt:
        print("\nStopping...")
