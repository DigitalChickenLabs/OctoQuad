// BUILDING:
// 1. Install PiGPIO
// 2. gcc octoquad.c main.c -lpigpio -o example.elf
// 3. sudo ./example.elf


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

#include <stdio.h>
#include <pigpio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "octoquad.h"

// ---------------------------------------------------------------------------------
//                               PLATFORM-SPECIFIC HAL
//----------------------------------------------------------------------------------

#define MY_SPI_SS_PIN 5

// To hold the handle returned by PiGPIO
int i2cHandle;
int spiHandle;

static bool platform_i2c_read_registers(const uint8_t addr, const uint8_t reg, uint8_t n, uint8_t* const dst)
{
    i2cReadI2CBlockData(i2cHandle, reg, dst, n);
    return true;
}

static bool platform_i2c_write_registers(const uint8_t addr, const uint8_t reg, const uint8_t* const src, const uint8_t n)
{
    i2cWriteI2CBlockData(i2cHandle, reg, src, n);
    return true;
}

static bool platform_spi_write_blocking(const uint8_t* const src, uint8_t n)
{
    uint8_t rx[n];
    spiXfer(spiHandle, src, rx, n);
    return true;
}

static bool platform_spi_write_read_blocking(const uint8_t* const src, uint8_t* const rx, uint8_t n)
{
    spiXfer(spiHandle, src, rx, n);
    return true;
}

static void platform_sleep_us(const uint16_t us)
{
    struct timespec req;
    req.tv_nsec = us * 1000;
    req.tv_sec = 0;
    clock_nanosleep(CLOCK_MONOTONIC, 0, &req, NULL);
}

void platform_sleep_ms(uint32_t ms)
{
    struct timespec req;
    req.tv_nsec = ms * 1000 * 1000;
    req.tv_sec = 0;
    clock_nanosleep(CLOCK_MONOTONIC, 0, &req, NULL);
}

static void platform_spi_cs_assert(bool assert)
{
    if(assert)
    {
        gpioWrite(MY_SPI_SS_PIN, 0);
    }
    else
    {
        gpioWrite(MY_SPI_SS_PIN, 1);
    }
}

// ---------------------------------------------------------------------------------
//                               HELPER FUNCTIONS
//----------------------------------------------------------------------------------

void setup_spi_interface()
{
    // Init PiGPIO library
    gpioInitialise();
    
    // Configure slave select pin as output
    gpioSetMode(MY_SPI_SS_PIN, PI_OUTPUT);
    
    // Set SPI parameters & open
    spiHandle = spiOpen(0, 1000*1000, 3);
    
    if(spiHandle < 0)
    {
        printf("Failed to open SPI, code %d\r\n", spiHandle);
        exit(-1);
    }

    platform_spi_cs_assert(false);
}

void setup_i2c_interface()
{
    // Init PiGPIO library
    gpioInitialise();
    
    // Set SPI parameters & open
    i2cHandle = i2cOpen(1, OCTOQUAD_I2C_ADDR, 0);
    
    if(i2cHandle < 0)
    {
        printf("Failed to open I2C, code %d\r\n", i2cHandle);
        exit(-1);
    }
}

// ---------------------------------------------------------------------------------
//                               MAIN
//----------------------------------------------------------------------------------

// CHOOSE INTERFACE MODE HERE
static OctoQuadInterface INTERFACE_CHOICE = OCTOQUAD_INTERFACE_SPI;

void main()
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
            .spi_write_read_blocking = &platform_spi_write_read_blocking,
            .spi_write_blocking = &platform_spi_write_blocking,
            .spi_cs_assert = &platform_spi_cs_assert,
            .sleep_us = &platform_sleep_us,
            .sleep_ms = &platform_sleep_ms
    };

    // Init the OctoQuad driver
    octoquad_init(INTERFACE_CHOICE, platform);

    printf("OctoQuad Raspberry Pi Example\r\n");

    // Check the CHIP_ID
    uint8_t chipId;
    if(!octoquad_get_chip_id(&chipId)) goto error;

    if(chipId == OCTOQUAD_CHIP_ID)
    {
        printf("CHIP_ID reports 0x%X as expected\r\n", chipId);
    }
    else
    {
        printf("CHIP_ID check failed, got 0x%X expect 0x%X\r\n", chipId, OCTOQUAD_CHIP_ID);
        goto error;
    }

    // Read the firmware version
    OctoQuadFwVersion firmwareVersion;
    if(!octoquad_get_fw_version(&firmwareVersion)) goto error;

    // Print to console
    printf("OctoQuad Reports FW v%d.%d.%d\r\n", firmwareVersion.maj, firmwareVersion.min, firmwareVersion.eng);

    // Check if that FW version is compatible
    if(firmwareVersion.maj != OCTOQUAD_DRIVER_SUPPORTED_FW_VERSION_MAJ)
    {
        printf("Cannot continue: The connected OctoQuad is running a firmware with a different major version than this program expects (got %d, expect %d)\r\n", firmwareVersion.maj, OCTOQUAD_DRIVER_SUPPORTED_FW_VERSION_MAJ);
        goto error;
    }
    
    OctoQuadChannelBankMode channelBankMode;
    if(!octoquad_get_channel_bank_mode(&channelBankMode)) goto error;
    printf("Channel Bank Mode = %d\r\n", channelBankMode);

    OctoQuadI2cRecoveryMode recoveryMode;
    if(!octoquad_get_i2c_recovery_mode(&recoveryMode)) goto error;
    printf("I2C Recovery Mode = %d\r\n", recoveryMode);

    for(int i = ENCODER_IDX_MIN; i <= ENCODER_IDX_MAX; i++)
    {
        uint8_t intvl;
        if(!octoquad_get_velocity_measurement_intvl(i, &intvl)) goto error;
        printf("Channel %d velocity sample interval = %d\r\n", i, intvl);
    }

    for(int i = ENCODER_IDX_MIN; i <= ENCODER_IDX_MAX; i++)
    {
        OctoQuadChannelPulseWidthParams params;
        if(!octoquad_get_channel_pulse_width_params(i, &params)) goto error;
        printf("Channel %d pulse min/max = %d/%d\r\n", i, params.min, params.max);
    }

    bool channelDirections[8];
    octoquad_get_all_channel_directions(channelDirections);

    for(int i = ENCODER_IDX_MIN; i <= ENCODER_IDX_MAX; i++)
    {
        printf("Channel %d reverse = %d\r\n", i, channelDirections[i]);
    }

    for(int i = 5; i > 0; i--)
    {
        printf("\rBeginning reads in %d", i);
        fflush(stdout);
        platform_sleep_ms(999);
    }

    octoquad_reset_all_positions();

    int32_t counts[8];
    int16_t vels[8];

    for(;;)
    {
        // On the Pi we have to read this in 2 operations because of the maximum I2C transfer size
        if(!octoquad_read_all_positions(counts)) goto error;
        if(!octoquad_read_all_velocities(vels)) goto error;

        printf("POS[%d,%d,%d,%d,%d,%d,%d,%d] VEL[%d,%d,%d,%d,%d,%d,%d,%d]\r\n",
               counts[0],
               counts[1],
               counts[2],
               counts[3],
               counts[4],
               counts[5],
               counts[6],
               counts[7],
               vels[0],
               vels[1],
               vels[2],
               vels[3],
               vels[4],
               vels[5],
               vels[6],
               vels[7]
        );

        fflush(stdout);
        platform_sleep_ms(16);
    }

    error:
    printf("Stopping: error\r\n");
    exit(-1);
}
