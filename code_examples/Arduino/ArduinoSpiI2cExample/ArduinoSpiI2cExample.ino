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

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "octoquad.h"
#include "octoquad.c"

// ---------------------------------------------------------------------------------
//                               PLATFORM-SPECIFIC HAL
//----------------------------------------------------------------------------------

#define MY_SPI_SS_PIN 7

static bool platform_i2c_read_registers(const uint8_t addr, const uint8_t reg, uint8_t n, uint8_t* const dst)
{
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(addr, n);

    while(Wire.available() < n);

    for(int i = 0; i < n; i++)
    {
        dst[i] = Wire.read();
    }

    return true;
}

static bool platform_i2c_write_registers(const uint8_t addr, const uint8_t reg, const uint8_t* const src, const uint8_t n)
{
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(src, n);
    Wire.endTransmission();
    return true;
}

static bool platform_spi_write_blocking(const uint8_t* const src, uint8_t n)
{
    uint8_t scratch[n];
    memcpy(scratch, src, n);
    SPI.transfer(scratch, n); // Avoid trashing src
    return true;
}

static bool platform_spi_write_read_blocking(const uint8_t* const src, uint8_t* const rx, uint8_t n)
{
    memcpy(rx, src, n);
    SPI.transfer(rx, n); // In place update
    return true;
}

static void platform_sleep_us(const uint16_t us)
{
    delayMicroseconds(us);
}

static void platform_spi_cs_assert(bool assert)
{
    if(assert)
    {
        digitalWrite(MY_SPI_SS_PIN, 0);
    }
    else
    {
        digitalWrite(MY_SPI_SS_PIN, 1);
    }
}

// ---------------------------------------------------------------------------------
//                               HELPER FUNCTIONS
//----------------------------------------------------------------------------------

void setup_spi_interface()
{
    // Configure slave select pin as output
    pinMode(MY_SPI_SS_PIN, OUTPUT);

    // We don't want to talk to the OctoQuad just yet
    platform_spi_cs_assert(false);

    // Configure the SPI peripheral
    SPI.begin();
    SPI.beginTransaction(SPISettings(1000 * 1000, MSBFIRST, SPI_MODE3));
}

void setup_i2c_interface()
{
    // Join I2C as bus master
    Wire.begin();
}

// ---------------------------------------------------------------------------------
//                               MAIN
//----------------------------------------------------------------------------------

// CHOOSE INTERFACE MODE HERE
static OctoQuadInterface INTERFACE_CHOICE = OCTOQUAD_INTERFACE_SPI;

// For use with sprintf
char strBuf[256];

void setup()
{
    if(INTERFACE_CHOICE == OCTOQUAD_INTERFACE_SPI)
    {
        setup_spi_interface();
    }
    else if(INTERFACE_CHOICE == OCTOQUAD_INTERFACE_I2C)
    {
        setup_i2c_interface();
    }

    // Define the platform HAL implementation for the OctoQuad driver
    OctoQuadPlatformImpl platform = {
            .i2c_read_registers = &platform_i2c_read_registers,
            .i2c_write_registers = &platform_i2c_write_registers,
            .spi_write_blocking = &platform_spi_write_blocking,
            .spi_write_read_blocking = &platform_spi_write_read_blocking,
            .spi_cs_assert = &platform_spi_cs_assert,
            .sleep_us = &platform_sleep_us
    };

    // Init the OctoQuad driver
    octoquad_init(INTERFACE_CHOICE, platform);

    // Init USB serial
    Serial.begin(115200);

    // Wait for PC connection
    uint8_t counter = 0;
    while(!Serial.available())
    {
        sprintf(strBuf, "OctoQuad Arduino example... send any char to continue [%3d]\r\n", counter);
        Serial.print(strBuf);
        delay(30);
        counter++;
    }

    // Check the CHIP_ID
    uint8_t chipId;
    if(!octoquad_read_chip_id(&chipId)) goto error;

    if(chipId == OCTOQUAD_CHIP_ID)
    {
        sprintf(strBuf, "CHIP_ID reports 0x%X as expected\r\n", chipId);
        Serial.print(strBuf);
    }
    else
    {
        sprintf(strBuf, "CHIP_ID check failed, got 0x%X expect 0x%X\r\n", chipId, OCTOQUAD_CHIP_ID);
        Serial.print(strBuf);
        goto error;
    }

    // Read the firmware version
    OctoQuadFwVersion firmwareVersion;
    if(!octoquad_read_fw_version(&firmwareVersion)) goto error;

    // Print to console
    sprintf(strBuf, "OctoQuad Reports FW v%d.%d.%d\r\n", firmwareVersion.maj, firmwareVersion.min, firmwareVersion.eng);
    Serial.print(strBuf);

    // Check if that FW version is compatible
    if(firmwareVersion.maj != OCTOQUAD_DRIVER_SUPPORTED_FW_VERSION_MAJ)
    {
        sprintf(strBuf, "Cannot continue: The connected OctoQuad is running a firmware with a different major version than this program expects (got %d, expect %d)\r\n", firmwareVersion.maj, OCTOQUAD_DRIVER_SUPPORTED_FW_VERSION_MAJ);
        Serial.print(strBuf);
        goto error;
    }

    for(int i = 5; i > 0; i--)
    {
        sprintf(strBuf, "\rBeginning reads in %d", i);
        Serial.print(strBuf);
        delay(1000);
    }

    octoquad_reset_all_encoders();

    OctoQuadEncoderData data;
    memset((uint8_t*)&data, 0, sizeof(data));

    for(;;)
    {
        if(!octoquad_read_all_encoder_data_struct(&data)) goto error;

        sprintf(strBuf, "POS[%d,%d,%d,%d,%d,%d,%d,%d] VEL[%d,%d,%d,%d,%d,%d,%d,%d]\r\n",
               data.enc0,
               data.enc1,
               data.enc2,
               data.enc3,
               data.enc4,
               data.enc5,
               data.enc6,
               data.enc7,
               data.vel0,
               data.vel1,
               data.vel2,
               data.vel3,
               data.vel4,
               data.vel5,
               data.vel6,
               data.vel7
        );
        Serial.print(strBuf);

        delay(16);
    }

    error:
    Serial.println("Stopping: error");
    while (1);
}

void loop()
{
  
}
