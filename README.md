# CANopen Module

## Overview
The `canopen_module` is an STM32-based firmware designed for industrial applications. It facilitates data acquisition and control using RS485 Modbus RTU and CANopen protocols. The module retrieves ultrasonic and vacuum data, controls valves, and communicates with an Industrial PC (IPC) via CANopen.

## Features
- **Ultrasonic Data Acquisition**: Collects data from ultrasonic sensors via RS485 using the Modbus RTU protocol.
- **Vacuum Data Acquisition**: Collects vacuum sensor data via RS485 using the Modbus RTU protocol.
- **Valve Control**: Manages valve operations.
- **CANopen Communication**: Enables communication with an IPC to:
  - Read ultrasonic and vacuum data
  - Send valve control commands

## Configuration
- **CAN ID**: `0x55`
- **Baudrate**: `1Mbps`

### CANopen Object Dictionary
| Index  | Subindex | Type     | Description                                |
|--------|----------|----------|--------------------------------------------|
| 0x6000 | 0x0      | uint16   | Vacuum Pressure Value                      |
| 0x6010 | 0x0      | uint16   | Ultrasonic Range Value                     |
| 0x6011 | 0x0      | uint16   | Ultrasonic Sensor Internal Temperature     |
| 0x6012 | 0x0      | uint16   | Ultrasonic Time of Flight                  |
| 0x6020 | 0x0      | uint8    | Valve 1 Control (non-zero value = turn-on) |
| 0x6021 | 0x0      | uint8    | Valve 2 Control (non-zero value = turn-on) |
| 0x6024 | 0x0      | uint8    | Valve 1 State                              |
| 0x6025 | 0x0      | uint8    | Valve 2 State                              |
| 0x6100 | 0x0      | uint8    | Running increment                          |
| 0x6110 | 0x0      | uint8    | Set non-zero value to restart the board    |

### TPDO Mapping
| PDO ID | Content                    |
|--------|----------------------------|
| 1      | 0x6000 (Vacuum Pressure)   |
| 2      | 0x6010, 0x6011, 0x6012     |
| 3      | 0x6024, 0x6025 (Valve States) |

### RPDO Mapping
| PDO ID | Content                    |
|--------|----------------------------|
| 1      | 0x6020, 0x6021 (Valve Control) |
| 2      | 0x6110 (Board Reset)        |

## Usage
1. Configure the CAN network with the specified baudrate (1Mbps)
2. Use the predefined PDO mappings for efficient data exchange
3. Monitor/control devices through the Object Dictionary entries
