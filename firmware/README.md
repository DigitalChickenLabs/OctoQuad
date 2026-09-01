# Download OctoQuad Firmware.

**WARNING: Flashing unofficial firmware may cause permanent damage to the OctoQuad, or to devices to which it is connected.**

## Firmware files

From time to time, official firmware updates for the OctoQuad may be released. 
These Official firmware binaries are found here. 

## Note about Firmware Versions

The OctoQuad FTC Edition MK1 shipped with firmare v2. It can, however, be upgraded to firmware v3 to gain support for CRC data validation. The OctoQuad FTC Edition MK2 shipped with firmware v3. It CANNOT be downgraded to firmware v2.

## Flashing firmware

To flash a firmware image onto the OctoQuad, follow the procedure below:

 - Remove all power and data connections from the OctoQuad.
 - Press and hold the BOOTSEL button (‘B’ on resin printed case)
 - While holding BOOTSEL, connect the OctoQuad to a computer using the micro-USB port
 - Wait until the emulated USB drive appears on the computer. The LED will remain off.
 - Drag-n-drop the firmware image onto the emulated USB drive
 - The OctoQuad will automatically flash the firmware and reboot. Flashing is complete when the emulated USB drive disappears and the status LED begins blinking an interface code.

## Release Notes

### v3.1.0

 - Improves IMU stability on FTC Ed MK2
 - Adds diagnostic parameters for the IMU and MCU uptime

### v3.0.11

 - Improves accuracy of absolute localizer velocity on FTC Ed MK2
 - Improves initial IMU calibration accuracy on FTC Ed MK2

### v3.0.10

 - Initial firmware release supporting OctoQuad FTC Ed MK2 (also backwards compatible with other OctoQuad models, to gain support for CRCs on data)
 - Note the register map changed from v2 and driver software will need to be updated accordingly

### v2.0.15

 - Initial firmware release compatible with original OctoQuad and OctoQuad FTC Ed MK1
