/*
 * Copyright (c) 2022 DigitalChickenLabs
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package com.octoquad.example;

import com.fazecast.jSerialComm.SerialPort;

import java.nio.charset.StandardCharsets;
import java.util.Scanner;

public class Main
{
    public static void main(String[] args) throws InterruptedException
    {
        SerialPort port = queryForSerialPort();
        System.out.printf("Opening port %s\r\n", port.getSystemPortPath());

        if(port.openPort())
        {
            System.out.println("Successfully opened port!");
            runWithOpenPort(port);
        }
        else
        {
            System.out.println("Failed to open port");
        }
    }

    public static void runWithOpenPort(SerialPort port) throws InterruptedException
    {
        port.setComPortTimeouts(SerialPort.TIMEOUT_READ_BLOCKING, 0, 0);

        // This isn't needed for USB serial mode, but, we want to be compatible with a USB to UART
        // adapter cable being used with the OctoQuad in serial mode, too.
        port.setComPortParameters(115200, 8, 1, 0);

        byte[] readBuf = new byte[1];
        byte[] lineBuf = new byte[256];
        int lineBufIdx = 0;

        // Synchronize to stream
        for(;;)
        {
            port.readBytes(readBuf, 1);
            if(readBuf[0] == '\n') break;
        }

        // Turn on "fast mode" (60Hz)
        sendCommand('F', port);

        // Read each line and parse it
        for(;;)
        {
            port.readBytes(readBuf, 1);
            lineBuf[lineBufIdx] = readBuf[0];
            lineBufIdx++;

            // Did we hit the end of the line?
            if(readBuf[0] == '\n')
            {
                String line = new String(lineBuf, 0,lineBufIdx-2, StandardCharsets.US_ASCII); // Trim off "\r\n"
                lineBufIdx = 0;
                int[] encoders = parseEncoderString(line);

                // Print out one of the encoder counts
                System.out.printf("Encoder 3 count: %d\r\n", encoders[3]);
            }
        }
    }

    // Write a one character command to the device
    public static void sendCommand(char cmd, SerialPort port)
    {
        port.writeBytes(new byte[]{(byte)cmd}, 1);
    }

    // Parse the CSV encoder string into an array
    public static int[] parseEncoderString(String string)
    {
        String[] parts = string.split(",");
        int[] encoders = new int[8];

        for(int i = 0; i < 8; i++)
        {
            encoders[i] = Integer.parseInt(parts[i]);
        }

        return encoders;
    }

    // Display a list of serial ports to the user and let them choose
    // which one they want to open
    public static SerialPort queryForSerialPort()
    {
        Scanner scanner = new Scanner(System.in);

        SerialPort[] comPorts = SerialPort.getCommPorts();
        System.out.println("Available COM ports:");

        for(int i = 0; i < comPorts.length; i++)
        {
            System.out.printf(" [%d] %s\r\n", i, comPorts[i].getSystemPortPath());
        }

        System.out.printf("Please select a COM port: ");
        int selection = scanner.nextInt();
        return comPorts[selection];
    }
}
