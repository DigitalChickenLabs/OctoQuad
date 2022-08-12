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

#ifndef OCTOQUADPICOLIB_OCTOQUAD_H
#define OCTOQUADPICOLIB_OCTOQUAD_H

#include "stdint.h"
#include "stdbool.h"

#define NUM_ENCODERS 8
#define ENCODER_IDX_MIN 0
#define ENCODER_IDX_MAX (NUM_ENCODERS - 1)
#define ENCODER_IDX_IN_RANGE(val) ((val >= ENCODER_IDX_MIN && val <= ENCODER_IDX_MAX))

#define OCTOQUAD_CHIP_ID 0x51
#define OCTOQUAD_I2C_ADDR 0x30

#define OCTOQUAD_DRIVER_SUPPORTED_FW_VERSION_MAJ 2

// ---------------------------------------------------------------------------------
//                                API STRUCTS
//----------------------------------------------------------------------------------
typedef struct __attribute__((packed))
{
    int32_t enc0;
    int32_t enc1;
    int32_t enc2;
    int32_t enc3;
    int32_t enc4;
    int32_t enc5;
    int32_t enc6;
    int32_t enc7;
    int16_t vel0;
    int16_t vel1;
    int16_t vel2;
    int16_t vel3;
    int16_t vel4;
    int16_t vel5;
    int16_t vel6;
    int16_t vel7;
} OctoQuadEncoderData;

typedef struct __attribute__((packed))
{
    uint8_t maj;
    uint8_t min;
    uint8_t eng;
} OctoQuadFwVersion;

typedef struct __attribute__((packed))
{
    uint16_t min;
    uint16_t max;
} OctoQuadChannelPulseWidthParams;

// ---------------------------------------------------------------------------------
//                                API ENUMS
//----------------------------------------------------------------------------------

typedef enum
{
    OCTOQUAD_I2C_RECOVERY_MODE_NONE = 0,
    OCTOQUAD_I2C_RECOVERY_MODE_1_PERIPH_RST_ON_FRAME_ERR = 1,
    OCTOQUAD_I2C_RECOVERY_MODE_2_M1_PLUS_SCL_IDLE_ONESHOT_TGL = 2
} OctoQuadI2cRecoveryMode;

typedef enum
{
    OCTOQUAD_CHANNEL_BANK_MODE_ALL_QUADRATURE = 0,
    OCTOQUAD_CHANNEL_BANK_MODE_ALL_PULSE_IN = 1,
    OCTOQUAD_CHANNEL_BANK_MODE_BANK1QUAD_BANK2PULSE = 2
} OctoQuadChannelBankMode;

typedef enum
{
    OCTOQUAD_INTERFACE_I2C,
    OCTOQUAD_INTERFACE_SPI
} OctoQuadInterface;

// ---------------------------------------------------------------------------------
//                                PLATFORM IMPL
//----------------------------------------------------------------------------------

typedef struct
{
    bool (*i2c_read_registers) (const uint8_t addr, const uint8_t reg, uint8_t n, uint8_t* const dst);
    bool (*i2c_write_registers) (const uint8_t addr, const uint8_t reg, const uint8_t* const src, const uint8_t n);
    bool (*spi_write_blocking) (const uint8_t* const src, uint8_t n);
    bool (*spi_write_read_blocking) (const uint8_t* const src, uint8_t* const rx, uint8_t n);
    void (*spi_cs_assert) (bool assert);
    void (*sleep_us) (const uint16_t us);
    void (*sleep_ms) (const uint16_t ms);
} OctoQuadPlatformImpl;

// ---------------------------------------------------------------------------------
//                                API FUNCTIONS
//----------------------------------------------------------------------------------

bool octoquad_init(OctoQuadInterface inft, OctoQuadPlatformImpl platform);
bool octoquad_read_chip_id(uint8_t* const out);
bool octoquad_read_fw_version(OctoQuadFwVersion * dst);

bool octoquad_read_single_position(uint8_t encoder, int32_t* out);
bool octoquad_read_single_velocity(uint8_t encoder, int16_t* out);

bool octoquad_read_all_positions(int32_t out[NUM_ENCODERS]);
bool octoquad_read_all_velocities(int16_t out[NUM_ENCODERS]);

bool octoquad_read_all_encoder_data_struct(OctoQuadEncoderData * out);
bool octoquad_read_all_encoder_data(int32_t countsOut[NUM_ENCODERS], int16_t velsOut[NUM_ENCODERS]);

bool octoquad_reset_single_position(uint8_t chan);
bool octoquad_reset_all_positions();

bool octoquad_set_velocity_measurement_intvl(uint8_t chan, uint8_t intvl);
bool octoquad_read_velocity_measurement_intvl(uint8_t chan, uint8_t* out);

bool octoquad_set_i2c_recovery_mode(OctoQuadI2cRecoveryMode mode);
bool octoquad_read_i2c_recovery_mode(OctoQuadI2cRecoveryMode* mode);

bool octoquad_set_channel_bank_mode(OctoQuadChannelBankMode mode);
bool octoquad_read_channel_bank_mode(OctoQuadChannelBankMode* out);

bool octoquad_set_channel_pulse_width_params(uint8_t chan, OctoQuadChannelPulseWidthParams params);
bool octoquad_read_channel_pulse_width_params(uint8_t chan, OctoQuadChannelPulseWidthParams* out);

bool octoquad_set_all_channel_directions(const bool reverse[8]);
bool octoquad_read_all_channel_directions(bool reverseOut[8]);

bool octoquad_set_single_channel_direction(uint8_t chan, bool reverse);
bool octoquad_read_single_channel_direction(uint8_t chan, bool* reverseOut);

bool octoquad_write_params_to_flash();
bool octoquad_reset_everything();

#endif //OCTOQUADPICOLIB_OCTOQUAD_H
