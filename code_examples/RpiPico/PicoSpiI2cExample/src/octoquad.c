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

#include "string.h"
#include "octoquad.h"

#define OCTOQUAD_FLAG_WRITE ((uint8_t)('W'))
#define OCTOQUAD_FLAG_WRITE_STICKY_REG ((uint8_t)('S'))
#define OCTOQUAD_FLAG_READ ((uint8_t)('R'))

#define OCTOQUAD_REG_CHIP_ID 0x00
#define OCTOQUAD_REG_FW_VERSION 0x01
#define OCTOQUAD_REG_COMMAND 0x04
#define OCTOQUAD_REG_ENC0 0x08
#define OCTOQUAD_REG_VEL0 0x28
#define OCTOQUAD_REG_VEL_INTVL_0 0x38

#define OCTOQUAD_CMD_POWER_UP_RESET 1
#define OCTOQUAD_CMD_RESET_ENCODERS 2
#define OCTOQUAD_CMD_SET_ENCODERS_DIR 3
#define OCTOQUAD_CMD_SET_I2C_RESET_MODE 4

#define SPI_ASSERT_DELAY_US 50

static OctoQuadPlatformImpl platformImpl;
static bool isI2c = true;
static uint8_t txBuf[64];
static uint8_t rxBuf[64];

// ---------------------------------------------------------------------------------
//                               I2C
//----------------------------------------------------------------------------------

// Just little helper functions :)
static bool i2c_read_registers(const uint8_t reg, uint8_t n, uint8_t* const dst)
{
    return platformImpl.i2c_read_registers(OCTOQUAD_I2C_ADDR, reg, n, dst);
}
static bool i2c_write_registers(const uint8_t reg, const uint8_t* const src, uint8_t n)
{
    return platformImpl.i2c_write_registers(OCTOQUAD_I2C_ADDR, reg, src, n);
}
static bool i2c_read_register(const uint8_t reg, uint8_t* const dst)
{
    return platformImpl.i2c_read_registers(OCTOQUAD_I2C_ADDR, reg, 1, dst);
}
static bool i2c_write_register(const uint8_t reg, const uint8_t data)
{
    return platformImpl.i2c_write_registers(OCTOQUAD_I2C_ADDR, reg, &data, 1);
}

// ---------------------------------------------------------------------------------
//                               SPI
//----------------------------------------------------------------------------------

static inline void spi_select_chip()
{
    platformImpl.spi_cs_assert(true);
    platformImpl.sleep_us(SPI_ASSERT_DELAY_US);
}

static inline void spi_deselect_chip()
{
    platformImpl.spi_cs_assert(false);
    platformImpl.sleep_us(SPI_ASSERT_DELAY_US);
}

// Handle the OctoQuad's idea of reading registers over SPI
static bool spi_read_registers(const uint8_t reg, const uint8_t n, uint8_t* const dst)
{
    // Write the register address
    txBuf[0] = OCTOQUAD_FLAG_WRITE;
    txBuf[1] = reg;
    spi_select_chip();
    platformImpl.spi_write_blocking(txBuf, 2);
    spi_deselect_chip();

    // Now do the read
    txBuf[0] = OCTOQUAD_FLAG_READ;
    spi_select_chip();
    platformImpl.spi_write_read_blocking(txBuf, rxBuf, n+1); // First byte is src address, so read one extra
    spi_deselect_chip();

    // If the source address sent by the OctoQuad doesn't agree with what we think
    // it should be, then, uh, we have a problem...
    if(rxBuf[0] != reg) goto error;

    // Pull out the actual register data minus the source address
    memcpy(dst, rxBuf+1, n);

    // If we end up here then we're all set :)
    return true;

    error:
    return false;
}

// Handle the OctoQuad's idea of writing registers over SPI
static bool spi_write_registers(const uint8_t reg, const uint8_t* const src, uint8_t n)
{
    // Concat the write flag, register address, and data
    txBuf[0] = OCTOQUAD_FLAG_WRITE;
    txBuf[1] = reg;
    memcpy(txBuf+2, src, n);

    // Send it off
    spi_select_chip();
    platformImpl.spi_write_blocking(txBuf, n+2);
    spi_deselect_chip();

    // We don't really have a way to know if the write failed, unfortunately
    return true;
}

// Just little helper functions :)
static bool spi_write_register(const uint8_t reg, const uint8_t data)
{
    return spi_write_registers(reg, &data, 1);
}
static bool spi_read_register(const uint8_t reg, uint8_t* const dst)
{
    return spi_read_registers(reg, 1, dst);
}

// ---------------------------------------------------------------------------------
//                               Delegating
//----------------------------------------------------------------------------------

static bool read_register(const uint8_t reg, uint8_t* dst)
{
    if(isI2c)
    {
        return i2c_read_register(reg, dst);
    }
    else
    {
        return spi_read_register(reg, dst);
    }
}

static bool read_registers(const uint8_t reg, const uint8_t n, uint8_t* const dst)
{
    if(isI2c)
    {
        return i2c_read_registers(reg, n, dst);
    }
    else
    {
        return spi_read_registers(reg, n, dst);
    }
}

static bool write_register(const uint8_t reg, const uint8_t data)
{
    if(isI2c)
    {
        return i2c_write_register(reg, data);
    }
    else
    {
        return spi_write_register(reg, data);
    }
}

static bool write_registers(const uint8_t reg, const uint8_t* const src, const uint8_t n)
{
    if(isI2c)
    {
        return i2c_write_registers(reg, src, n);
    }
    else
    {
        return spi_write_registers(reg, src, n);
    }
}

// ---------------------------------------------------------------------------------
//                               API
//----------------------------------------------------------------------------------

bool octoquad_init(OctoQuadInterface interface, OctoQuadPlatformImpl platform)
{
    if(interface == OCTOQUAD_INTERFACE_SPI || interface == OCTOQUAD_INTERFACE_I2C)
    {
        isI2c = interface == OCTOQUAD_INTERFACE_I2C;
        platformImpl = platform;

        return true;
    }

    return false;
}

bool octoquad_read_chip_id(uint8_t* const out)
{
    return read_register(OCTOQUAD_REG_CHIP_ID, out);
}

bool octoquad_read_fw_version(OctoQuadFwVersion * dst)
{
    return read_registers(OCTOQUAD_REG_FW_VERSION, sizeof(OctoQuadFwVersion), (uint8_t*) dst);
}

bool octoquad_read_single_count(uint8_t encoder, int32_t* out)
{
    if(!ENCODER_IDX_IN_RANGE(encoder))
    {
        return false;
    }

    const uint8_t target_reg = OCTOQUAD_REG_ENC0 + encoder * sizeof(int32_t);

    return read_registers(target_reg, sizeof(int32_t), (uint8_t*) out);
}

bool octoquad_read_single_velocity(uint8_t encoder, int16_t* out)
{
    if(!ENCODER_IDX_IN_RANGE(encoder))
    {
        return false;
    }

    const uint8_t target_reg = OCTOQUAD_REG_VEL0 + encoder * sizeof(int16_t);

    return read_registers(target_reg, sizeof(int16_t), (uint8_t*) out);
}

bool octoquad_read_all_counts(int32_t out[NUM_ENCODERS])
{
    return read_registers(OCTOQUAD_REG_ENC0, NUM_ENCODERS * sizeof(int32_t), (uint8_t*) out);
}

bool octoquad_read_all_velocities(int16_t out[NUM_ENCODERS])
{
    return read_registers(OCTOQUAD_REG_VEL0, NUM_ENCODERS * sizeof(int16_t), (uint8_t*) out);
}

bool octoquad_read_all_encoder_data_struct(OctoQuadEncoderData* out)
{
    return read_registers(OCTOQUAD_REG_ENC0, sizeof(OctoQuadEncoderData), (uint8_t*)out);
}

bool octoquad_read_all_encoder_data(int32_t countsOut[NUM_ENCODERS], int16_t velsOut[NUM_ENCODERS])
{
    OctoQuadEncoderData data;
    bool ok = read_registers(OCTOQUAD_REG_ENC0, sizeof(OctoQuadEncoderData), (uint8_t*)&data);

    if(!ok)
    {
        return false;
    }

    memcpy(countsOut, (uint8_t*)&data, NUM_ENCODERS * sizeof(int32_t));
    memcpy(velsOut, (uint8_t*)&data, NUM_ENCODERS * sizeof(int16_t));

    return true;
}

bool octoquad_reset_single_encoder(uint8_t encoder)
{
    if(!ENCODER_IDX_IN_RANGE(encoder))
    {
        return false;
    }

    uint8_t outgoing[2];
    outgoing[0] = OCTOQUAD_CMD_RESET_ENCODERS;
    outgoing[1] = (uint8_t) (1 << encoder);

    return write_registers(OCTOQUAD_REG_COMMAND, outgoing, sizeof(outgoing));
}

bool octoquad_reset_all_encoders()
{
    uint8_t outgoing[2];
    outgoing[0] = OCTOQUAD_CMD_RESET_ENCODERS;
    outgoing[1] = (uint8_t) 0xFF;

    return write_registers(OCTOQUAD_REG_COMMAND, outgoing, sizeof(outgoing));
}

bool octoquad_set_single_velocity_measurement_intvl(uint8_t encoder, uint8_t intvl)
{
    if(!ENCODER_IDX_IN_RANGE(encoder))
    {
        return false;
    }

    uint8_t target_reg = OCTOQUAD_REG_VEL_INTVL_0 + encoder;

    return write_register(target_reg, intvl);
}

bool octoquad_get_single_velocity_measurement_intvl(uint8_t encoder, uint8_t* out)
{
    if(!ENCODER_IDX_IN_RANGE(encoder))
    {
        return false;
    }

    uint8_t target_reg = OCTOQUAD_REG_VEL_INTVL_0 + encoder;

    return read_register(target_reg, out);
}

bool octoquad_get_all_velocity_measurement_intvls(uint8_t out[NUM_ENCODERS])
{
    return read_registers(OCTOQUAD_REG_VEL_INTVL_0, NUM_ENCODERS, out);
}

bool octoquad_set_all_velocity_measurement_intvls(uint8_t intvl)
{
    uint8_t outgoing[NUM_ENCODERS];
    for(int i = 0; i < sizeof(outgoing); i++)
    {
        outgoing[i] = intvl;
    }

    return write_registers(OCTOQUAD_REG_VEL_INTVL_0, outgoing, sizeof(outgoing));
}

bool octoquad_set_i2c_reset_mode(OctoQuadI2cResetMode mode)
{
    uint8_t outgoing[2];
    outgoing[0] = OCTOQUAD_CMD_SET_I2C_RESET_MODE;
    outgoing[1] = (uint8_t) mode;

    return write_registers(OCTOQUAD_REG_COMMAND, outgoing, sizeof(outgoing));
}
