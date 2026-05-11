# G55 CAN Bus Gateway

Custom Teensy 4.1 CAN gateway for a 2011 Mercedes-Benz G55 AMG W463.

## Hardware

- Teensy 4.1
- Treedix Teensy 4.1 screw-terminal breakout
- GC9A01 1.28 inch round 240x240 SPI IPS display
- ECSING SN65HVD230 CAN module for bench testing
- DSD TECH SH-C31G isolated USB-CAN adapter
- Future target: iDatalink Maestro RR2 ADS-MRR2 + Sony XAV-9500ES

## Current firmware phase

`v0.1_safe_obd_display`

- Polls only safe OBD Mode 01 PIDs
- TX ID: 0x7DF
- RX IDs: 0x7E8-0x7EF
- Displays live RPM, speed, coolant, throttle, IAT, and MAP
- Uses strict PID allowlist

## Wiring

### GC9A01 Display

| Display | Teensy 4.1 |
|---|---|
| VCC | 3.3V |
| GND | GND |
| SCL / CLK / SCK | Pin 13 |
| SDA / DIN / MOSI | Pin 11 |
| CS | Pin 10 |
| DC | Pin 9 |
| RES / RST | Pin 8 |
| BL / LED | 3.3V |

### CAN1

| Teensy 4.1 | SN65HVD230 |
|---|---|
| 3.3V | VCC |
| GND | GND |
| Pin 22 / CTX1 | CTX / TXD |
| Pin 23 / CRX1 | CRX / RXD |

### Vehicle OBD CAN

| OBD-II | CAN Module |
|---|---|
| Pin 6 CAN High | CANH |
| Pin 14 CAN Low | CANL |
| Pin 4/5 Ground | GND |

## Safety

Firmware must not transmit arbitrary diagnostic requests.

Allowed Mode 01 PIDs only:

- 010C RPM
- 010D vehicle speed
- 0105 coolant temp
- 0111 throttle position
- 010F intake air temp
- 010B MAP

No Mode 04, Mode 08, Mode 10, Mode 11, Mode 27, Mode 2E, Mode 31, or programming services.
