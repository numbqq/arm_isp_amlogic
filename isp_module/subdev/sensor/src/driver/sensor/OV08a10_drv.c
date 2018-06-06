/*
*
* SPDX-License-Identifier: GPL-2.0
*
* Copyright (C) 2011-2018 ARM or its affiliates
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; version 2.
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
* or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
* for more details.
* You should have received a copy of the GNU General Public License along
* with this program; if not, write to the Free Software Foundation, Inc.,
* 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
*
*/

//-------------------------------------------------------------------------------------
//STRUCTURE:
//  VARIABLE SECTION:
//        CONTROLS - dependence from preprocessor
//        DATA     - modulation
//        RESET     - reset function
//        MIPI     - mipi settings
//        FLASH     - flash support
//  CONSTANT SECTION
//        DRIVER
//-------------------------------------------------------------------------------------

#include "acamera_types.h"
#include "sensor_init.h"
#include "acamera_math.h"
#include "acamera_command_api.h"
#include "acamera_sbus_api.h"
#include "acamera_sensor_api.h"
#include "system_timer.h"
#include "acamera_firmware_config.h"
#include "sensor_bus_config.h"
#include "OV08a10_seq.h"
#include "OV08a10_config.h"
#include "system_am_mipi.h"
#include "system_am_adap.h"

static void start_streaming( void *ctx );
static void stop_streaming( void *ctx );

static sensor_mode_t supported_modes[3] = {
    {
        .wdr_mode = WDR_MODE_LINEAR,
        .fps = 5 * 256,
        .resolution.width = 1280,
        .resolution.height = 720,
        .bits = 10,
        .exposures = 1,
    },
    {
        .wdr_mode = WDR_MODE_LINEAR,
        .fps = 30 * 256,
        .resolution.width = 1920,
        .resolution.height = 1080,
        .bits = 10,
        .exposures = 1,
    },
    {
        .wdr_mode = WDR_MODE_LINEAR,
        .fps = 30 * 256,
        .resolution.width = 3840,
        .resolution.height = 2160,
        .bits = 10,
        .exposures = 1,
    }
};

typedef struct _sensor_context_t {
    uint8_t address; // Sensor address for direct write (not used currently)
    acamera_sbus_t sbus;
    sensor_param_t param;
    uint8_t seq_width;
    uint8_t streaming_flg;
    uint16_t again[4];
    uint8_t again_delay;
    uint16_t int_time_S;
    uint16_t int_time_M;
    uint16_t int_time_L;
    uint32_t shs1;
    uint32_t shs2;
    uint32_t shs3;
    uint32_t shs1_old;
    uint32_t shs2_old;
    uint32_t rhs1;
    uint32_t rhs2;
    uint32_t again_limit;
    uint8_t s_fps;
    uint32_t vmax;
    uint8_t int_cnt;
    uint8_t gain_cnt;
    uint32_t pixel_clock;
    uint16_t max_S;
    uint16_t max_M;
    uint16_t max_L;
    uint16_t frame;
    uint32_t wdr_mode;
} sensor_context_t;

#if SENSOR_BINARY_SEQUENCE
static const char p_sensor_data[] = SENSOR__OV08A10_SEQUENCE_DEFAULT;
#else
static const acam_reg_t **p_sensor_data = seq_table;
#endif


//*************************************************************************************
//--------------------DATA-------------------------------------------------------------
//--------------------RESET------------------------------------------------------------
static void sensor_hw_reset_enable( void )
{
}

static void sensor_hw_reset_disable( void )
{
}

//--------------------FLASH------------------------------------------------------------

static int32_t sensor_alloc_analog_gain( void *ctx, int32_t gain )
{
	sensor_context_t *p_ctx = ctx;

	uint16_t again = (gain * 20) >> LOG2_GAIN_SHIFT;

	if (again > p_ctx->again_limit)
		again = p_ctx->again_limit;

	if (p_ctx->again[0] != again) {
		p_ctx->gain_cnt = p_ctx->again_delay + 1;
		p_ctx->again[0] = again;
	}

	return (((int32_t)again) << LOG2_GAIN_SHIFT) / 20;
}

static int32_t sensor_alloc_digital_gain( void *ctx, int32_t gain )
{
    return 0;
}

static void sensor_alloc_integration_time( void *ctx, uint16_t *int_time_S, uint16_t *int_time_M, uint16_t *int_time_L )
{
    sensor_context_t *p_ctx = ctx;
    uint16_t tmp;

    switch ( p_ctx->wdr_mode ) {
    case WDR_MODE_LINEAR: // Normal mode
        if ( *int_time_S > p_ctx->vmax - 2 ) *int_time_S = p_ctx->vmax - 2;
        if ( *int_time_S < 1 ) *int_time_S = 1;
        tmp = p_ctx->vmax - *int_time_S - 1;
        if ( p_ctx->int_time_S != tmp ) {
            p_ctx->int_cnt = 2;
            p_ctx->int_time_S = tmp;
        }
        break;
    case WDR_MODE_FS_LIN: // DOL3 Frames
        if ( *int_time_S < 2 ) *int_time_S = 2;
        if ( *int_time_S > p_ctx->max_S ) *int_time_S = p_ctx->max_S;
        if ( *int_time_L < 2 ) *int_time_L = 2;
        if ( *int_time_L > p_ctx->max_L ) *int_time_L = p_ctx->max_L;

        if ( *int_time_M < 2 ) *int_time_M = 2;
        if ( *int_time_M > p_ctx->max_M ) *int_time_M = p_ctx->max_M;

        if ( p_ctx->int_time_S != *int_time_S || p_ctx->int_time_M != *int_time_M || p_ctx->int_time_L != *int_time_L ) {
            p_ctx->int_cnt = 3;

            p_ctx->int_time_S = *int_time_S;
            p_ctx->int_time_M = *int_time_M;
            p_ctx->int_time_L = *int_time_L;

            p_ctx->shs3 = p_ctx->frame - *int_time_L - 1;
            p_ctx->shs1 = p_ctx->rhs1 - *int_time_M - 1;
            p_ctx->shs2 = p_ctx->rhs2 - *int_time_S - 1;
        }
        break;
    }
}

static void sensor_update( void *ctx )
{
    sensor_context_t *p_ctx = ctx;
    acamera_sbus_ptr_t p_sbus = &p_ctx->sbus;

    if ( p_ctx->int_cnt || p_ctx->gain_cnt ) {
        // ---------- Start Changes -------------
        acamera_sbus_write_u8( p_sbus, 0x0201, 1 );

        // ---------- Analog Gain -------------
        if ( p_ctx->gain_cnt ) {
            p_ctx->gain_cnt--;
            acamera_sbus_write_u8( p_sbus, 0x0214, p_ctx->again[p_ctx->again_delay] );
        }

        // -------- Integration Time ----------
        if ( p_ctx->int_cnt ) {
            p_ctx->int_cnt--;
            switch ( p_ctx->wdr_mode ) {
            case WDR_MODE_LINEAR:
                acamera_sbus_write_u8( p_sbus, 0x0221, ( p_ctx->int_time_S >> 8 ) & 0xFF );
                acamera_sbus_write_u8( p_sbus, 0x0220, ( p_ctx->int_time_S >> 0 ) & 0xFF );
                break;
            case WDR_MODE_FS_LIN:
                p_ctx->shs2_old = p_ctx->shs2;
                p_ctx->shs1_old = p_ctx->shs1;
                // SHS3
                acamera_sbus_write_u8( p_sbus, 0x0229, ( p_ctx->shs3 >> 8 ) & 0xFF );
                acamera_sbus_write_u8( p_sbus, 0x0228, ( p_ctx->shs3 >> 0 ) & 0xFF );

                // SHS1
                acamera_sbus_write_u8( p_sbus, 0x0221, ( p_ctx->shs1_old >> 8 ) & 0xFF );
                acamera_sbus_write_u8( p_sbus, 0x0220, ( p_ctx->shs1_old >> 0 ) & 0xFF );

                // SHS2
                acamera_sbus_write_u8( p_sbus, 0x0225, ( p_ctx->shs2_old >> 8 ) & 0xFF );
                acamera_sbus_write_u8( p_sbus, 0x0224, ( p_ctx->shs2_old >> 0 ) & 0xFF );
                break;
            }
        }

        // ---------- End Changes -------------
        acamera_sbus_write_u8( p_sbus, 0x0201, 0 );
    }
    p_ctx->shs1_old = p_ctx->shs1;
    p_ctx->shs2_old = p_ctx->shs2;
    p_ctx->again[3] = p_ctx->again[2];
    p_ctx->again[2] = p_ctx->again[1];
    p_ctx->again[1] = p_ctx->again[0];
}

static void sensor_set_mode( void *ctx, uint8_t mode )
{
    sensor_context_t *p_ctx = ctx;
    sensor_param_t *param = &p_ctx->param;
    acamera_sbus_ptr_t p_sbus = &p_ctx->sbus;

    sensor_hw_reset_enable();
    system_timer_usleep( 10000 );
    sensor_hw_reset_disable();
    system_timer_usleep( 10000 );
	LOG(LOG_ERR, "%s: LIKE:wdr mode 0x%08x\n", __func__, param->modes_table[mode].wdr_mode);
	return;
    switch ( param->modes_table[mode].wdr_mode ) {
    case WDR_MODE_LINEAR:
		LOG(LOG_ERR, "%s: load setting\n", __func__);
        //sensor_load_sequence( p_sbus, p_ctx->seq_width, p_sensor_data, SENSOR_OV08A10_SEQUENCE_DEFAULT_INIT );
		sensor_load_sequence( p_sbus, p_ctx->seq_width, p_sensor_data, SENSOR_OV08A10_SEQUENCE_DEFAULT_PREVIEW );
        p_ctx->s_fps = 30;
        p_ctx->again_delay = 2;
        param->integration_time_apply_delay = 2;
        param->isp_exposure_channel_delay = 0;
        break;
    case WDR_MODE_FS_LIN:
        p_ctx->again_delay = 0;
        param->integration_time_apply_delay = 2;
        param->isp_exposure_channel_delay = 0;

        if ( param->modes_table[mode].exposures == 2 ) {

            //sensor_load_sequence( p_sbus, p_ctx->seq_width, p_sensor_data, SENSOR_IMX290_SEQUENCE_DEFAULT_WDR_720P );
        } else {

            //sensor_load_sequence( p_sbus, p_ctx->seq_width, p_sensor_data, SENSOR_IMX290_SEQUENCE_DEFAULT_WDR_720P );
        }
        p_ctx->s_fps = 50;
        break;
    default:
        return;
        break;
    }

    if ( param->modes_table[mode].fps == 25 * 256 ) {
        acamera_sbus_write_u8( p_sbus, 0x0218, 0x46 );
        acamera_sbus_write_u8( p_sbus, 0x0219, 0x05 );
        p_ctx->s_fps = 25;
        p_ctx->vmax = 1350;
    } else {
        p_ctx->vmax = 1125;
    }

    uint8_t r = ( acamera_sbus_read_u8( p_sbus, 0x8207 ) >> 4 );
    switch ( r ) {
    case 0: // HD 1080p
        param->active.width = 1920;
        param->active.height = 1080;
        p_ctx->max_L = 3936;
        p_ctx->max_M = 486 + 30;
        p_ctx->max_S = 60 - 30;
        p_ctx->rhs1 = 523;
        p_ctx->rhs2 = 560;
        // p_ctx->vmax=1125;
        break;
    case 1:
        param->active.width = 1280;
        param->active.height = 720;
        p_ctx->max_L = 2502;
        p_ctx->max_M = 420;
        p_ctx->max_S = 60;
        p_ctx->rhs1 = 427;
        p_ctx->rhs2 = 494;
        break;
    default:
        // 4- Window cropping from 1080p, Other- Prohibited
        //LOG(LOG_CRIT,"WRONG IMAGE SIZE CONFIG");
        break;
    }

    // Enable syncs on XHS and XVS pins
    //acamera_sbus_write_u8(p_sbus, 0x024b, 0x0A);

    acamera_sbus_write_u8( p_sbus, 0x0231, ( p_ctx->rhs1 >> 8 ) & 0xFF );
    acamera_sbus_write_u8( p_sbus, 0x0230, ( p_ctx->rhs1 >> 0 ) & 0xFF );
    acamera_sbus_write_u8( p_sbus, 0x0235, ( p_ctx->rhs2 >> 8 ) & 0xFF );
    acamera_sbus_write_u8( p_sbus, 0x0234, ( p_ctx->rhs2 >> 0 ) & 0xFF );
    param->total.width = ( (uint16_t)acamera_sbus_read_u8( p_sbus, 0x821D ) << 8 ) | acamera_sbus_read_u8( p_sbus, 0x821C );
    param->lines_per_second = p_ctx->pixel_clock / param->total.width;
    param->total.height = (uint16_t)p_ctx->vmax;
    param->pixels_per_line = param->total.width;
    param->integration_time_min = SENSOR_MIN_INTEGRATION_TIME;
    if ( param->modes_table[mode].wdr_mode == WDR_MODE_LINEAR ) {
        param->integration_time_limit = SENSOR_MAX_INTEGRATION_TIME_LIMIT;
        param->integration_time_max = p_ctx->vmax - 2;
    } else {
        param->integration_time_limit = 60;
        param->integration_time_max = 60;
        if ( param->modes_table[mode].exposures == 2 ) {
            param->integration_time_long_max = ( p_ctx->vmax << 1 ) - 256;
            param->lines_per_second = param->lines_per_second >> 1;
            p_ctx->frame = p_ctx->vmax << 1;
        } else {
            param->integration_time_long_max = ( p_ctx->vmax << 2 ) - 256;
            param->lines_per_second = param->lines_per_second >> 2;
            p_ctx->frame = p_ctx->vmax << 2;
        }
    }
    param->sensor_exp_number = param->modes_table[mode].exposures;
    param->integration_time_limit = SENSOR_MAX_INTEGRATION_TIME_LIMIT;
    param->mode = mode;
    p_ctx->wdr_mode = param->modes_table[mode].wdr_mode;

    if ( p_ctx->streaming_flg )
        start_streaming( ctx );
    else
        stop_streaming( ctx );

	LOG( LOG_CRIT, "RES:%dx%d\n", (int)param->active.width, (int)param->active.height );
}

static uint16_t sensor_get_id( void *ctx )
{
	sensor_context_t *p_ctx = ctx;
	uint32_t sensor_id = 0;

	sensor_id |= acamera_sbus_read_u8(&p_ctx->sbus, 0x300a) << 16;
	sensor_id |= acamera_sbus_read_u8(&p_ctx->sbus, 0x300b) << 8;
	sensor_id |= acamera_sbus_read_u8(&p_ctx->sbus, 0x300c);

	if (sensor_id != SENSOR_CHIP_ID) {
		LOG(LOG_ERR, "%s: Failed to read sensor id\n", __func__);
		return 0xFF;
	}

	LOG(LOG_ERR, "%s: Success Read sensor id 0x%x\n", __func__, sensor_id);

    return 0;
}

static const sensor_param_t *sensor_get_parameters( void *ctx )
{
    sensor_context_t *p_ctx = ctx;
    return (const sensor_param_t *)&p_ctx->param;
}

static void sensor_disable_isp( void *ctx )
{
}

static uint32_t read_register( void *ctx, uint32_t address )
{
    sensor_context_t *p_ctx = ctx;
    acamera_sbus_ptr_t p_sbus = &p_ctx->sbus;
    return acamera_sbus_read_u8( p_sbus, address );
}

static void write_register( void *ctx, uint32_t address, uint32_t data )
{
	sensor_context_t *p_ctx = ctx;
    acamera_sbus_ptr_t p_sbus = &p_ctx->sbus;
    acamera_sbus_write_u8( p_sbus, address, data );
}

static void stop_streaming( void *ctx )
{
	sensor_context_t *p_ctx = ctx;
	acamera_sbus_ptr_t p_sbus = &p_ctx->sbus;
	p_ctx->streaming_flg = 0;
	acamera_sbus_write_u8(p_sbus, 0x0100, 0x00);
	LOG(LOG_ERR, "%s: Stream Off\n", __func__);
}

static void start_streaming( void *ctx )
{
	sensor_context_t *p_ctx = ctx;
    acamera_sbus_ptr_t p_sbus = &p_ctx->sbus;
    p_ctx->streaming_flg = 1;
    acamera_sbus_write_u8(p_sbus, 0x0100, 0x01);
	LOG(LOG_ERR, "%s: Stream On\n", __func__);
}

void sensor_deinit_ov08a10( void *ctx )
{
	reset_sensor_bus_counter();
	am_adap_deinit();
	am_mipi_deinit();
}
//--------------------Initialization------------------------------------------------------------
void sensor_init_ov08a10( void **ctx, sensor_control_t *ctrl )
{
    // Local sensor data structure
	am_mipi_info_t mipi_info;
	static sensor_context_t s_ctx;
	struct am_adap_info info;
	*ctx = &s_ctx;

	s_ctx.sbus.mask = SBUS_MASK_ADDR_16BITS |
			SBUS_MASK_SAMPLE_8BITS |SBUS_MASK_ADDR_SWAP_BYTES;
	s_ctx.sbus.control = I2C_CONTROL_MASK;
	s_ctx.sbus.bus = 1;//get_next_sensor_bus_address();
	s_ctx.sbus.device = SENSOR_DEV_ADDRESS;
	acamera_sbus_init(&s_ctx.sbus, sbus_i2c);

	sensor_get_id(&s_ctx);

	//sensor_load_sequence( &s_ctx.sbus, 1, p_sensor_data, SENSOR_OV08A10_SEQUENCE_DEFAULT_PREVIEW );
	sensor_load_sequence( &s_ctx.sbus, 1, p_sensor_data, SENSOR_OV08A10_SEQUENCE_SECOND_PREVIEW);
	LOG(LOG_ERR, "Load setting %d\n", SENSOR_OV08A10_SEQUENCE_DEFAULT_PREVIEW);

	s_ctx.address = SENSOR_DEV_ADDRESS;
	s_ctx.seq_width = 1;
	s_ctx.streaming_flg = 0;
	s_ctx.again[0] = 0;
	s_ctx.again[1] = 0;
	s_ctx.again[2] = 0;
	s_ctx.again[3] = 0;
	s_ctx.again_limit = 20;//AGAIN_MAX_DB + DGAIN_MAX_DB;
	s_ctx.pixel_clock = 148500000;

	s_ctx.param.again_accuracy = 1 << LOG2_GAIN_SHIFT;
	s_ctx.param.sensor_exp_number = 1;
	s_ctx.param.again_log2_max = 0;
	s_ctx.param.dgain_log2_max = 0;
	s_ctx.param.integration_time_apply_delay = 2;
	s_ctx.param.isp_exposure_channel_delay = 0;
	s_ctx.param.modes_table = supported_modes;
	s_ctx.param.modes_num = array_size( supported_modes );
	s_ctx.param.active.width = 1920;
	s_ctx.param.active.height = 1080;
	s_ctx.param.total.width = 3840;
	s_ctx.param.total.height = 2160;
	s_ctx.param.sensor_ctx = &s_ctx;

	ctrl->alloc_analog_gain = sensor_alloc_analog_gain;
	ctrl->alloc_digital_gain = sensor_alloc_digital_gain;
	ctrl->alloc_integration_time = sensor_alloc_integration_time;
	ctrl->sensor_update = sensor_update;
	ctrl->set_mode = sensor_set_mode;
	ctrl->get_id = sensor_get_id;
	ctrl->get_parameters = sensor_get_parameters;
	ctrl->disable_sensor_isp = sensor_disable_isp;
	ctrl->read_sensor_register = read_register;
	ctrl->write_sensor_register = write_register;
	ctrl->start_streaming = start_streaming;
	ctrl->stop_streaming = stop_streaming;

	// Reset sensor during initialization
	sensor_hw_reset_enable();
	system_timer_usleep( 1000 ); // reset at least 1 ms
	sensor_hw_reset_disable();
	system_timer_usleep( 1000 );

	memset(&mipi_info, 0, sizeof(mipi_info));
	mipi_info.lanes = 4;
	mipi_info.ui_val = 2;

	am_mipi_init(&mipi_info);

	info.fmt = AM_RAW10;
	info.img.width = s_ctx.param.active.width;
	info.img.height = s_ctx.param.active.height;
	info.path = PATH0;
	info.mode = DIR_MODE;
	am_adap_set_info(&info);
	am_adap_init();
	am_adap_start(0);

	//start_streaming(&s_ctx);

	LOG(LOG_ERR, "%s: Success subdev init\n", __func__);
}

//*************************************************************************************