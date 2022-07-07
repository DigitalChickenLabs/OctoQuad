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

#include "hardware/gpio.h"
#include <stdio.h>
#include "hardware/i2c.h"
#include "hardware/sync.h"
#include "pico/unique_id.h"
#include "string.h"
#include "hardware/spi.h"
#include "octoquad.h"
#include "pico/stdlib.h"
#include "tusb.h"

// ---------------------------------------------------------------------------------
//                               PLATFORM-SPECIFIC HAL
//----------------------------------------------------------------------------------

#define MY_SPI_TX_PIN 19
#define MY_SPI_RX_PIN 16
#define MY_SPI_SCL_PIN 18
#define MY_SPI_SS_PIN 17
#define MY_I2C_SDA_PIN 18
#define MY_I2C_SCL_PIN 19

#define MY_SPI_INST spi0
#define MY_I2C_INST i2c1
#define I2C_TIMEOUT_US 50000

static bool platform_i2c_read_registers(const uint8_t addr, const uint8_t reg, uint8_t n, uint8_t* const dst)
{
    // Write register address; hold onto bus
    if(1 != i2c_write_timeout_us(MY_I2C_INST, addr, &reg, 1, true, I2C_TIMEOUT_US)) goto error;

    // Read in data; release bus
    if(n != i2c_read_timeout_us(MY_I2C_INST, addr, dst, n, false, I2C_TIMEOUT_US)) goto error;

    // If we end up here we're all set :)
    return true;

    // If we got sent here that means something went wrong
    error:
    return false;
}

static bool platform_i2c_write_registers(const uint8_t addr, const uint8_t reg, const uint8_t* const src, const uint8_t n)
{
    uint8_t txBuf[n+1];
    txBuf[0] = reg;
    memcpy(txBuf+1, src, n);
    return n+1 == i2c_write_timeout_us(MY_I2C_INST, addr, txBuf, n+1, false, I2C_TIMEOUT_US);
}

static bool platform_spi_write_blocking(const uint8_t* const src, uint8_t n)
{
    spi_write_blocking(MY_SPI_INST, src, n);
    return true;
}

static bool platform_spi_write_read_blocking(const uint8_t* const src, uint8_t* const rx, uint8_t n)
{
    spi_write_read_blocking(MY_SPI_INST, src, rx, n);
    return true;
}

static void platform_sleep_us(const uint16_t us)
{
    sleep_us(us);
}

static void platform_spi_cs_assert(bool assert)
{
    if(assert)
    {
        gpio_put(MY_SPI_SS_PIN, 0);
    }
    else
    {
        gpio_put(MY_SPI_SS_PIN, 1);
    }
}

// ---------------------------------------------------------------------------------
//                               HELPER FUNCTIONS
//----------------------------------------------------------------------------------

void setup_spi_interface()
{
    // Set up the SPI peripheral
    spi_init(MY_SPI_INST, 1000 * 1000);
    spi_set_slave(MY_SPI_INST, false);
    spi_set_format(MY_SPI_INST, 8, 1, 1, SPI_MSB_FIRST);

    // Bind pins to SPI peripheral
    gpio_set_function(MY_SPI_RX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(MY_SPI_TX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(MY_SPI_SCL_PIN, GPIO_FUNC_SPI);

    // We control slave select separately
    gpio_set_function(MY_SPI_SS_PIN, GPIO_FUNC_SIO);
    gpio_set_dir(MY_SPI_SS_PIN, GPIO_OUT);
    platform_spi_cs_assert(false);
}

void setup_i2c_interface()
{
    // Set up the I2C peripheral
    i2c_init(MY_I2C_INST, 100 * 1000); // Pico has really weak pullups so we have to use a super slow speed (or, add external pullups)

    // Bind pins to I2C peripheral
    gpio_set_function(MY_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(MY_I2C_SCL_PIN, GPIO_FUNC_I2C);

    gpio_pull_up(MY_I2C_SDA_PIN);
    gpio_pull_up(MY_I2C_SCL_PIN);
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
            .sleep_us = &platform_sleep_us
    };

    // Init the OctoQuad driver
    octoquad_init(INTERFACE_CHOICE, platform);

    // Init USB serial
    stdio_usb_init();

    // Wait for PC connection
    while (!tud_cdc_connected())
    {
        sleep_ms(1);
    }
    sleep_ms(200);

    printf("OctoQuad RPi Pico Example\r\n");

    // Check the CHIP_ID
    uint8_t chipId;
    if(!octoquad_read_chip_id(&chipId)) goto error;

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
    if(!octoquad_read_fw_version(&firmwareVersion)) goto error;

    // Print to console
    printf("OctoQuad Reports FW v%d.%d.%d\r\n", firmwareVersion.maj, firmwareVersion.min, firmwareVersion.eng);

    // Check if that FW version is compatible
    if(firmwareVersion.maj != OCTOQUAD_DRIVER_SUPPORTED_FW_VERSION_MAJ)
    {
        printf("Cannot continue: The connected OctoQuad is running a firmware with a different major version than this program expects (got %d, expect %d)\r\n", firmwareVersion.maj, OCTOQUAD_DRIVER_SUPPORTED_FW_VERSION_MAJ);
        goto error;
    }

    for(int i = 5; i > 0; i--)
    {
        printf("\rBeginning reads in %d", i);
        sleep_ms(1000);
    }

    octoquad_reset_all_encoders();

    OctoQuadEncoderData data;
    memset((uint8_t*)&data, 0, sizeof(data));

    for(;;)
    {
        if(!octoquad_read_all_encoder_data_struct(&data)) goto error;

        printf("POS[%d,%d,%d,%d,%d,%d,%d,%d] VEL[%d,%d,%d,%d,%d,%d,%d,%d]\r\n",
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

        sleep_ms(16);
    }

    error:
    printf("Stopping: error");
    while (1);
}