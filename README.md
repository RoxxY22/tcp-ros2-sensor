# tcp-ros2-sensor
# EyeROV ROS2 Sensor Node - TCP Hex protocol


## Overview
This ROS2 Python node communicates with a sensor over TCP port 2000, where the sensor acts as the  TCP Server.

It implements the **hex-encoded ASCII protocol** as specified:
- Commands: `#` start, 2-char ID, payload (even hex chars), `<CR><LF>`
- Responses: `$` start, 2-char ID, payload, `<CR><LF>`
- All data in **little-endian**, **hex ASCII**, only `0-9`, `A-F`

## ROS2 Topics (Requirement #2)
| Topic Name               | Message Type       | Unit       | Description                             |
| ------------------------ | ------------------ | ---------- | --------------------------------------- |
| `/sensor/supply_voltage` | `std_msgs/UInt16`  | millivolts | Sensor supply voltage (raw mV)          |
| `/sensor/env_temp`       | `std_msgs/Float32` | °C         | Environment temperature (deci-°C → ÷10) |
| `/sensor/yaw`            | `std_msgs/Float32` | degrees    | Yaw angle (deci-deg → ÷10)              |
| `/sensor/pitch`          | `std_msgs/Float32` | degrees    | Pitch angle (deci-deg → ÷10)            |
| `/sensor/roll`           | `std_msgs/Float32` | degrees    | Roll angle (deci-deg → ÷10)             |

---

## Start / Stop the Sensor (Requirement #3)

###1. **Auto-Start on Launch**
- Node **automatically sends** Start command using parameter `interval_ms`
- Default: **1000 ms**

```bash
ros2 run sensor_node sensor_node


###2.  **Stop the Sensor**

```bash
ros2 service call /stop_sensor std_srvs/srv/Empty "{}"
→ Sends: #09\r\n


###3. Start with Custom interval
```bash
# Set interval in milliseconds
ros2 param set /sensor_node interval_ms 500

# Start sensor
ros2 service call /start_sensor std_srvs/srv/SetBool "{data: true}"
→ Sends: #03F401\r\n (500 ms in little-endian hex)


Protocol Implementation
| Message | Format                  | Example                                       |
| ------- | ----------------------- | --------------------------------------------- |
| Start   | `#03<4 hex chars>\r\n`  | `#03E803\r\n` → 1000 ms (0x03E8 = `E8 03` LE) |
| Stop    | `#09\r\n`               | `#09\r\n`                                     |
| Status  | `$11<20 hex chars>\r\n` | `$11D0071A00...\r\n`                          |

Payload: 20 hex chars = 10 bytes
Decoding: bytes.fromhex() → struct.unpack('<H') / ('<h')
Units converted: deci-C → °C, deci-deg → deg


Project Structure
.
├── README.md
├── sensor_simulator.py          # For testing
└── src/
    └── sensor_node/
        ├── package.xml
        ├── setup.py
        └── sensor_node/
            └── sensor_node.py   # Main node
            
            
            

How to Build & Run
```bash
# Build
colcon build --packages-select sensor_node
source install/setup.bash

# Run simulator (Terminal 1)
python3 sensor_simulator.py

# Run node (Terminal 2)
ros2 run sensor_node sensor_node


Test
```bash
ros2 topic echo /sensor/yaw
ros2 service call /stop_sensor std_srvs/srv/Empty "{}"



