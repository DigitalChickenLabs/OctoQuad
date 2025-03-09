# Note about Firmware Versions

The OctoQuad FTC Edition MK1 shipped with firmare v2. It can, however, be upgraded to firmware v3 to gain support for CRC data validation. The OctoQuad FTC Edition MK2 shipped with firmware v3. It CANNOT be downgraded to firmware v2.

# Using the OctoQuad in the FTC SDK

The FTC SDK has a driver built-in for firmware v2. See the example code shipped with the SDK [here](https://github.com/FIRST-Tech-Challenge/FtcRobotController/blob/master/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/SensorOctoQuad.java).

However, for firmwre v3 (which is required for the OctoQuad FTC Edition MK2), you MUST use the driver supplied in this folder. To use this driver, simply copy&paste it into your TeamCode module, and configure the OctoQuad in your robot configuration as "OctoQuad FW v3".

# OctoQuad FTC Edition MK2 Localizer Quickstart

To quickly get up and runnning with the absolute localizer functions of the MK2 board, review the Localizer Quickstart PDF document located in the documentation directory of this repository.
