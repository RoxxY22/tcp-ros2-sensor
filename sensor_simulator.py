import socket
import threading
import time
import random

# Simulate sensor data
def generate_status():
    supply_voltage = random.randint(3000, 5000)  # mV
    env_temp = random.randint(-100, 500)  # deci-C
    yaw = random.randint(-1800, 1800)  # deci-deg
    pitch = random.randint(-900, 900)
    roll = random.randint(-900, 900)
    
    # Convert to little-endian hex (2 bytes each, 4 hex chars)
    def to_hex_le(value, signed=False):
        if signed:
            value = value & 0xFFFF if value >= 0 else (1 << 16) + value
        low = value & 0xFF
        high = (value >> 8) & 0xFF
        return f"{low:02X}{high:02X}"
    
    payload = (
        to_hex_le(supply_voltage) +
        to_hex_le(env_temp, signed=True) +
        to_hex_le(yaw, signed=True) +
        to_hex_le(pitch, signed=True) +
        to_hex_le(roll, signed=True)
    )
    return f"$11{payload}\r\n"

# Server
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind(('localhost', 2000))
server.listen(1)
print("Sensor simulator listening on localhost:2000")

conn, addr = server.accept()
print(f"Connected by {addr}")

interval = 1000  # Default ms
running = False
def send_status():
    while running:
        conn.send(generate_status().encode())
        time.sleep(interval / 1000)

thread = threading.Thread(target=send_status)
thread.daemon = True

while True:
    data = conn.recv(1024).decode().strip()
    if not data:
        break
    print(f"Received: {data}")
    
    if data.startswith('#03') and len(data) == 7:  # #03 + 4 hex (strip removes \r\n)
        interval_hex = data[3:7]
        # Little-endian: swap bytes
        interval = int(interval_hex[2:4] + interval_hex[0:2], 16)
        running = True
        if not thread.is_alive():
            thread.start()
        print(f"Started with interval {interval}ms")
    elif data.startswith('#09') and len(data) == 3:  # #09 (strip removes \r\n)
        running = False
        print("Stopped")