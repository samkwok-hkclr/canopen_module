# CANopen Module

## Overview
The `canopen_module` is an STM32-based firmware designed for Robostore. It facilitates data acquisition and control using RS485 Modbus RTU and CANopen protocols. The module retrieves ultrasonic and vacuum data, controls valves, and communicates with an IP) via CANopen.

## Features
- **Ultrasonic Data Acquisition**: Collects data from ultrasonic sensors via RS485 using the Modbus RTU protocol.
- **Vacuum Data Acquisition**: Collects vacuum sensor data via RS485 using the Modbus RTU protocol.
- **Pump Control**: Manages valve operations.
- **Gripper Control**: Manages gripper operations via RS485 using the Modbus RTU protocol.
- **CANopen Communication**: Enables communication with an IPC to:
  - Read ultrasonic and vacuum data
  - Send valve control commands

## Configuration
- **CAN ID**: `0x55`
- **Baudrate**: `1Mbps`

### TPDO Mapping
| PDO ID | Content                          |
|--------|----------------------------------|
| 1      | 0x6000, 0x6002, 0x6024, 0x6025   |
| 2      | 0x6010, 0x6011, 0x6012           |
| 3      | 0x6035, 0x6036, 0x6037, 0x6032   |

### RPDO Mapping
| PDO ID | Content                    |
|--------|----------------------------|
| 1      | 0x6020, 0x6021             |
| 2      | 0x603F, 0x6030             |
| 3      | 0x6032                     |
| 4      | 0x6031                     |

## Usage
1. Configure the CAN network with the specified baudrate (1Mbps)
2. Use the predefined PDO mappings for efficient data exchange
3. Monitor/control devices through the Object Dictionary entries
