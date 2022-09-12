
# For using the OctoQuad in FIRST Tech Challenge, start [HERE.](https://github.com/DigitalChickenLabs/OctoQuadFtcDriver)

The OctoQuad is an 8-channel quadrature & pulse width decode module. For quadrature signals, position and velocity can be tracked for all 8 channels simultaneously at up to a 250KHz pulse rate (up to 1 million counts/s). For pulse width signals, pulses up to 65535us are supported. The channel inputs and I2C lines are protected from ESD to +/- 15kV (air).

Options for interfacing to OctoQuad are very flexible:

 - I2C (up to 400KHz bus speed)

 - SPI (up to 1MHz bus speed)

 - UART (ASCII-based protocol, 115200 baud, 10-60Hz data output)

 - USB serial (ASCII-based protocol, 10-60Hz data output)

The OctoQuad is perfect for use cases where there is a need to decode many pulse width or high-speed encoder signals, but doing so directly on the system is not feasible. For example, reading quadrature encoders on a Raspberry Pi can be problematic because Linux is not a real-time operating system and does not guarantee accurate timing. Similarly, decoding many high speed quadrature signals on a microprocessor can consume a significant amount of processor time and requires many digital pins. When using an OctoQuad, this processing is offloaded, leaving your processor free for other tasks and removing the need to worry about precise timing.

**All logic and power is 3.3v only! The OctoQuad is not 5v tolerant!**

The OctoQuad can be powered via or USB or via the 3.3v input pin on the board. All channel connections and the dedicated I2C connector use a JST-PH connector, while the auxiliary header uses a standard 0.1" pin spacing.

A wide range of sample code is available, including for Arduino, Raspberry Pi, and PC.
