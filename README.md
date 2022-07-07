The OctoQuad is an 8-channel quadrature decode module which tracks position and velocity for up to 8 quadrature encoders simultaneously at up to a 250KHz pulse rate (up to 1 million counts/s). The encoder channels and I2C lines are protected from ESD to +/- 15kV (air).

Options for interfacing to OctoQuad are very flexible:

 - I2C (up to 400KHz bus speed)

 - SPI (up to 1MHz bus speed)

 - UART (ASCII-based protocol, 115200 baud, 10-60Hz data output)

 - USB serial (ASCII-based protocol, 10-60Hz data output)

The OctoQuad is perfect for use cases where there is a need to decode many high-speed encoder signals but doing so directly on the system is not feasible. For example, reading quadrature encoders on a Raspberry Pi can be problematic because Linux is not a real-time operating and does not guarantee accurate timing. Similarly, decoding many high speed quadrature signals on a microprocessor can consume a significant amount of processor time and requires many digital pins. When using an OctoQuad, this processing is offloaded, leaving your processor free for other tasks and removing the need to worry about precise timing.

**All logic and power is 3.3v only! The OctoQuad is *not* 5v tolerant!**

The OctoQuad can be powered via or USB or via the 3v3 input pin on the board. All encoder channel connections and the dedicated I2C connector use a JST-PH connector, while the auxiliary header uses a standard 0.1" pin spacing.

A wide range of sample code is available, including for Arduino, Raspberry Pi, and PC.
