/*
 * ldc1101.h
 *
 *  Created on: Jun 28, 2021
 *      Author: David Ryan
 */

#ifndef INC_LDC1101_H_
#define INC_LDC1101_H_

#include "stm32f4xx_hal.h"

/* PUBLIC DEFINE CONFIG VALUES */

/* Status reporting */
#define LDC1101_OK 		0x01
#define LDC1101_NOT_OK 	0x00

#define NUMBER_OF_REGS 	32

/* Register RP_SET Field Descriptions (RW) */
#define LDC1101_RP_SET_RP_MAX_96KOhm                 0x00
#define LDC1101_RP_SET_RP_MAX_48KOhm                 0x10
#define LDC1101_RP_SET_RP_MAX_24KOhm                 0x20
#define LDC1101_RP_SET_RP_MAX_12KOhm                 0x30
#define LDC1101_RP_SET_RP_MAX_6KOhm                  0x40
#define LDC1101_RP_SET_RP_MAX_3KOhm                  0x50
#define LDC1101_RP_SET_RP_MAX_1_5KOhm                0x60
#define LDC1101_RP_SET_RP_MAX_0_75KOh                0x70
#define LDC1101_RP_SET_RP_MIN_96KOhm                 0x00
#define LDC1101_RP_SET_RP_MIN_48KOhm                 0x01
#define LDC1101_RP_SET_RP_MIN_24KOhm                 0x02
#define LDC1101_RP_SET_RP_MIN_12KOhm                 0x03
#define LDC1101_RP_SET_RP_MIN_6KOhm                  0x04
#define LDC1101_RP_SET_RP_MIN_3KOhm                  0x05
#define LDC1101_RP_SET_RP_MIN_1_5KOhm                0x06
#define LDC1101_RP_SET_RP_MIN_0_75KOh                0x07

/* Configure Internal Time Constant 1 (RW) */
#define LDC1101_TC1_C1_0_75pF     0x00
#define LDC1101_TC1_C1_1_5pF      0x40
#define LDC1101_TC1_C1_3pF        0x80
#define LDC1101_TC1_C1_6pF        0xC0
#define LDC1101_TC1_R1_417kOhm    0x00
#define LDC1101_TC1_R1_212_7kOhm  0x10
#define LDC1101_TC1_R1_21_1kOhm   0x1F

/* Configure Internal Time Constant 2 (RW) */
#define LDC1101_TC2_C2_3pF        0x00
#define LDC1101_TC2_C2_6pF        0x40
#define LDC1101_TC2_C2_12pF       0x80
#define LDC1101_TC2_C2_24pF       0xC0
#define LDC1101_TC2_R2_835kOhm    0x00
#define LDC1101_TC2_R2_426_4kOhm  0x20
#define LDC1101_TC2_R2_30_5kOhm   0x3F

/* Configure RP+L conversion interval (RW) */

#define LDC1101_DIG_CFG_MIN_FREQ_500kHz   	0x00
#define LDC1101_DIG_CFG_MIN_FREQ_533kHz		0x10
#define LDC1101_DIG_CFG_MIN_FREQ_571kHz		0x20
#define LDC1101_DIG_CFG_MIN_FREQ_615kHz		0x30
#define LDC1101_DIG_CFG_MIN_FREQ_666kHz		0x40
#define LDC1101_DIG_CFG_MIN_FREQ_727kHz		0x50
#define LDC1101_DIG_CFG_MIN_FREQ_800kHz		0x60
#define LDC1101_DIG_CFG_MIN_FREQ_888kHz		0x70
#define LDC1101_DIG_CFG_MIN_FREQ_1MHz		0x80
#define LDC1101_DIG_CFG_MIN_FREQ_114MHz		0x90
#define LDC1101_DIG_CFG_MIN_FREQ_133MHz		0xA0
#define LDC1101_DIG_CFG_MIN_FREQ_160MHz		0xB0
#define LDC1101_DIG_CFG_MIN_FREQ_2MHz		0xC0
#define LDC1101_DIG_CFG_MIN_FREQ_266MHz		0xD0
#define LDC1101_DIG_CFG_MIN_FREQ_4MHz		0xE0
#define LDC1101_DIG_CFG_MIN_FREQ_8MHz		0xF0

#define LDC1101_DIG_CFG_RESP_TIME_192s    0x02
#define LDC1101_DIG_CFG_RESP_TIME_384s    0x03
#define LDC1101_DIG_CFG_RESP_TIME_768s    0x04
#define LDC1101_DIG_CFG_RESP_TIME_1536s   0x05
#define LDC1101_DIG_CFG_RESP_TIME_3072s   0x06
#define LDC1101_DIG_CFG_RESP_TIME_6144s   0x07

/* Configure INTB reporting on SDO pin (RW) */
#define LDC1101_INTB_MODE_REPORT_LHR_DATA_READY               0x20
#define LDC1101_INTB_MODE_L_CONVERSION_TO_L_THRESHOLDS        0x10
#define LDC1101_INTB_MODE_L_CONVERSION_TO_L_HIGH_THRESHOLDS   0x08
#define LDC1101_INTB_MODE_REPORT_RP_L_DATA_READY              0x04
#define LDC1101_INTB_MODE_RP_CONVERSION_TO_L_THRESHOLDS       0x02
#define LDC1101_INTB_MODE_RP_CONVERSION_TO_L_HIGH_THRESHOLDS  0x01
#define LDC1101_INTB_MODE_NO_OUTPUT                           0x00

/* High Resolution L Configuration (RW) */
#define LDC1101_LHR_CFG_FREQUENCY_DIVIDED_BY_2   0x01
#define LDC1101_LHR_CFG_FREQUENCY_DIVIDED_BY_4   0x02
#define LDC1101_LHR_CFG_FREQUENCY_DIVIDED_BY_8   0x03



/* PRIVITE DEFINE */

/* LDC1101 REDGESTER ADDRESSES */

#define _LDC1101_REG_CFG_RP_MEASUREMENT_DYNAMIC_RANGE  	0x01
#define _LDC1101_REG_CFG_INTERNAL_TIME_CONSTANT_1      	0x02
#define _LDC1101_REG_CFG_INTERNAL_TIME_CONSTANT_2      	0x03
#define _LDC1101_REG_CFG_RP_L_CONVERSION_INTERVAL      	0x04
#define _LDC1101_REG_CFG_ADDITIONAL_DEVICE_SETTINGS    	0x05
#define _LDC1101_REG_RP_THRESH_H_LSB                   	0x06
#define _LDC1101_REG_RP_THRESH_H_MSB                   	0x07
#define _LDC1101_REG_RP_THRESH_L_LSB                   	0x08
#define _LDC1101_REG_RP_THRESH_L_MSB                   	0x09
#define _LDC1101_REG_CFG_INTB_MODE                     	0x0A
#define _LDC1101_REG_CFG_POWER_STATE                   	0x0B
#define _LDC1101_REG_AMPLITUDE_CONTROL_REQUIREMENT     	0x0C

#define _LDC1101_REG_L_THRESH_HI_LSB                   	0x16
#define _LDC1101_REG_L_THRESH_HI_MSB                   	0x17
#define _LDC1101_REG_L_THRESH_LO_LSB                   	0x18
#define _LDC1101_REG_L_THRESH_LO_MSB                   	0x19

#define _LDC1101_REG_RP_L_MEASUREMENT_STATUS           	0x20
#define _LDC1101_REG_RP_DATA_LSB                       	0x21
#define _LDC1101_REG_RP_DATA_MSB                       	0x22
#define _LDC1101_REG_L_DATA_LSB                        	0x23
#define _LDC1101_REG_L_DATA_MSB                        	0x24

#define _LDC1101_REG_LHR_RCOUNT_LSB                    	0x30
#define _LDC1101_REG_LHR_RCOUNT_MSB                    	0x31
#define _LDC1101_REG_LHR_OFFSET_LSB                    	0x32
#define _LDC1101_REG_LHR_OFFSET_MSB                    	0x33
#define _LDC1101_REG_CFG_LHR                           	0x34

#define _LDC1101_REG_LHR_DATA_LSB						0x38
#define _LDC1101_REG_LHR_DATA_MID            			0x39
#define _LDC1101_REG_LHR_DATA_MSB               	    0x3A
#define _LDC1101_REG_LHR_STATUS             	        0x3B

#define _LDC1101_REG_DEVICE_RID			                0x3E
#define _LDC1101_REG_DEVICE_ID            	            0x3F

/* Configure additional device settings (RW) */
#define _LDC1101_ALT_CFG_SHUTDOWN_ENABLE     0x02
#define _LDC1101_ALT_CFG_SHUTDOWN_DISABLE    0x00
#define _LDC1101_ALT_CFG_L_OPTIMAL_DISABLED  0x00
#define _LDC1101_ALT_CFG_L_OPTIMAL_ENABLE    0x01

/* Configure INTB reporting on SDO pin (RW) */
#define _LDC1101_INTB_MODE_DONT_REPORT_INTB_ON_SDO_PIN         0x00
#define _LDC1101_INTB_MODE_REPORT_INTB_ON_SDO_PIN              0x80

/* Register RP_SET Field Descriptions (RW) */
#define _LDC1101_RP_SET_RP_MAX_IS_DRIVEN              0x00
#define _LDC1101_RP_SET_RP_MAX_CURRENT_IS_IGNORED     0x80

/* Configure Power State (RW) */
#define _LDC1101_FUNC_MODE_ACTIVE_CONVERSION_MODE  0x00
#define _LDC1101_FUNC_MODE_SLEEP_MODE              0x01
#define _LDC1101_FUNC_MODE_SHUTDOWN_MODE           0x02

/* High Resolution L Configuration (RW) */
#define _LDC1101_LHR_CFG_FREQUENCY_NOT_DIVIDED    0x00

/* LDC1101 struct status byte bit positions */
#define _LDC1101_STATUS_POWER_STATE 			0b00000001
#define _LDC1101_STATUS_MODE 					0b00000010
#define _LDC1101_STATUS_BIT_2					0b00000100
#define _LDC1101_STATUS_NO_SENSOR_OSCILLATION	0b00001000
#define _LDC1101_STATUS_LHR_ERROR				0b00010000
#define _LDC1101_STATUS_INTB_MODE				0b00100000
#define _LDC1101_STATUS_BIT_6 					0b01000000
#define _LDC1101_STATUS_BIT_7 					0b10000000

/* LDC1101 external const declarations */
extern const uint8_t _REG_ADRS[32];
extern const uint8_t _REG_DEFAULT_VALS[32];


/**
 * @param  GPIOx where x can be (A..K) to select the GPIO peripheral for STM32F429X device or
 *                      x can be (A..I) to select the GPIO peripheral for STM32F40XX and STM32F427X devices.
 * @param  GPIO_Pin specifies the port bit to be written.
 *          This parameter can be one of GPIO_PIN_x where x can be (0..15).
 */
struct GPIO_PIN {
	GPIO_TypeDef *GPIOx;
	uint16_t GPIO_Pin;
};

/*
 * status byte in LDC1101_SPI struct:
 * [B7][B6][B5][B4][B3][B2][B1][B0]
 * [-][-][INTB][LHR Error][Sensor Oscillation][-][Mode][Power State]
 *
 * Individual status bits reflect specific register bits:
 * Power State - 0: sleep mode | 1: active calculation mode
 * Mode - 0: RP+L mode selected | 1: LHR mode selected
 * No Sensor Oscillation - Reflects: STATUS:NO_SENSOR_OSC (register 0x20-bit7)
 * 		When the resonance impedance of the sensor, RP, drops below the programmed Rp_MIN, the sensor
 *		oscillation may stop. This condition could occur when a target comes too close to the sensor or if RP_SET:RP_MIN (register 0x01-
 *		bits[2:0]) is set too high.
 * LHR Error - 0: No LHR Error | 1: LHR Error; either Zero Count Error, Conversion Over-range Error,
 * 		Conversion Under-range Error and/or Conversion Over-flow Error. Further investigation of the LHR_STATUS reg is required.
 * INTB - 0: INTB mode disabled | 1: INTB mode active
 *
 */


/**
 * @brief	LDC1101 Struct that handles all ldc1101 communication
 * @param 	status Error reporting status byte
 * @param 	hspi Pointer to a hspi struct
 * @param 	cs GPIO_PIN struct
 */
struct LDC1101 {
	uint8_t status;
	SPI_HandleTypeDef *hspi;
	struct GPIO_PIN cs;
};

/**
 * @brief	Rp and L data structure to store and reference
 * 			rp and l measurements
 * @param	status Error reporting status register
 * @param	rp_msb Most significant byte for rp measurement
 * @param	rp_lsb Least significant byte for rp measurement
 * @param	l_msb Most significant byte for l measurement
 * @param	l_lsb Least significant byte for l measurement
 * @param	rp Full 16 bit value of rp measurement
 * @param	l Full 16 bit value of l measurement
 * @param	rp_avg Running average of rp measurement
 * @param	l_avg Running average of l measurement
 * @param	avg_delta Average calculation delta
 */
struct RPL_DATA {
	uint8_t status;
	uint8_t rp_msb;
	uint8_t rp_lsb;
	uint8_t l_msb;
	uint8_t l_lsb;
	uint16_t rp;
	uint16_t l;
	uint16_t rp_avg;
	uint16_t l_avg;
	uint8_t avg_delta;
};


/**
 * @brief	L High Resolution structure for storing and referencing
 * 			lhr measurements
 * @param	status Error reporting status register
 * @param	l_lsb Least significant byte for l measurement
 * @param	l_mid Mid byte for l measurement
 * @param	l_msb Most significant byte for l measurement
 * @param	l	Full 32 bit value for l measurement
 * 			(only uses the 24 least significant bits)
 * @param	l_avg Running average of l measurement
 * @param	avg_delta Average calculation delta
 */
struct LHR_DATA {
	uint8_t status;
	uint8_t l_lsb;
	uint8_t l_mid;
	uint8_t l_msb;
	uint32_t l;
	uint32_t l_avg;
	uint8_t avg_delta;
};


/**
 * @brief	Structure to store chip rid and id
 * @param	rid Rid value
 * @param	id Id value
 */
struct LDC1101_CHIP {
	uint8_t rid;
	uint8_t id;
};

/* HAL FUNCTIONS */

void ldc1101_hal_write_cs(struct GPIO_PIN *pin, uint8_t value);

void ldc1101_hal_toggle_cs(struct GPIO_PIN *pin);

void ldc1101_hal_spi_transmit(SPI_HandleTypeDef *hspi, uint8_t *pData,
		uint16_t Size, uint32_t Timeout);

void ldc1101_hal_deactivate_clkin();

void ldc1101_hal_activate_clkin();

void ldc1101_hal_uart_transmit(UART_HandleTypeDef *huart, uint8_t *pData,
		uint16_t Size, uint32_t Timeout);

/* BASIC OPERATIONS */
void ldc1101_test_data_packet(struct LDC1101 *ldc1101);

void ldc1101_write_reg(struct LDC1101 *ldc1101, uint8_t reg, uint8_t data);

uint8_t LDC1101_read_reg(struct LDC1101 *ldc1101, uint8_t reg);

uint8_t ldc1101_write_and_check_reg(struct LDC1101 *ldc1101, uint8_t reg,
		uint8_t data);

void ldc1101_edit_reg(struct LDC1101 *ldc1101, uint8_t reg, uint8_t data,
		uint8_t mask);

uint8_t ldc1101_edit_and_check_reg(struct LDC1101 *ldc1101, uint8_t reg,
		uint8_t data, uint8_t mask);

void ldc1101_read_consecutive_reg(struct LDC1101 *ldc1101, uint8_t startReg,
		uint8_t *buf, uint8_t length);

void ldc1101_read_all_reg(struct LDC1101 *ldc1101);


/*
 * POWER CONTROL
 * returns _LDC1101_OK or LDC1101_NOT_OK
 */

uint8_t ldc1101_shutdown(struct LDC1101 *ldc1101);

uint8_t ldc1101_startup(struct LDC1101 *ldc1101);

uint8_t ldc1101_enter_sleep(struct LDC1101 *ldc1101);

uint8_t ldc1101_exit_sleep(struct LDC1101 *ldc1101);

uint8_t ldc1101_toggle_sleep(struct LDC1101 *ldc1101);

uint8_t ldc1101_restart(struct LDC1101 *ldc1101);


/* INITIALIZATION */

uint8_t ldc1101_init(struct LDC1101 *ldc1101);

uint8_t ldc1101_rpl_setup(struct LDC1101 *ldc1101);

uint8_t ldc1101_lhr_setup(struct LDC1101 *ldc1101);

/*
 * OPERATION AND MODE CONFIGERATION
 * returns _LDC1101_OK or LDC1101_NOT_OK
 */

uint8_t ldc1101_enable_optimize_for_l(struct LDC1101 *ldc1101);

uint8_t ldc1101_disable_optimize_for_l(struct LDC1101 *ldc1101);

uint8_t ldc1101_enable_rp_dynamic_range(struct LDC1101 *ldc1101, uint8_t RP_MAX,
		uint8_t RP_MIN);

uint8_t ldc1101_disable_rp_dynamic_range(struct LDC1101 *ldc1101);

uint8_t ldc1101_configure_rp_dynamic_range(struct LDC1101 *ldc1101,
		uint8_t RP_MAX, uint8_t RP_MIN);

uint8_t ldc1101_configure_resp_time(struct LDC1101 *ldc1101, uint8_t MIN_FREQ,
		uint8_t RESP_TIME);

uint8_t ldc1101_enable_intb_mode(struct LDC1101 *ldc1101, uint8_t MODE);

uint8_t ldc1101_disable_intb_mode(struct LDC1101 *ldc1101);

uint8_t ldc1101_configure_tc1(struct LDC1101 *ldc1101, uint8_t R_VAL,
		uint8_t C_VAL);

uint8_t ldc1101_configure_tc2(struct LDC1101 *ldc1101, uint8_t R_VAL,
		uint8_t C_VAL);

uint8_t ldc1101_configure_dig(struct LDC1101 *ldc1101, uint8_t MIN_FREQ,
		uint8_t RESP_TIME);

uint8_t ldc1101_configure_rp_high_low_threshold(struct LDC1101 *ldc1101,
		uint16_t RP_THRESH_HI, uint16_t RP_THRESH_LOW);

uint8_t ldc1101_configure_l_high_low_threshold(struct LDC1101 *ldc1101,
		uint16_t L_THRESH_HI, uint16_t L_THRESH_LOW);

uint8_t ldc1101_configure_lhr_rcount(struct LDC1101 *ldc1101, uint16_t RCOUNT);

uint8_t ldc1101_configure_lhr_offset(struct LDC1101 *ldc1101, uint16_t OFFSET);



/*
 * NORMAL OPERATION
 * returns data structures
 */

struct LDC1101_CHIP ldc1101_get_id(struct LDC1101 *ldc1101);

struct RPL_DATA ldc1101_read_rpl_data(struct LDC1101 *ldc1101, 	struct RPL_DATA data);

struct LHR_DATA ldc1101_read_lhr_data(struct LDC1101 *ldc1101, struct LHR_DATA data);



/* UART PRINT */

void ldc1101_print_chip(UART_HandleTypeDef *huart, struct LDC1101 *ldc1101,
		struct LDC1101_CHIP chip);

void ldc1101_print_rpl_data(UART_HandleTypeDef *huart, struct RPL_DATA rplData);

void ldc1101_print_lhr_data(UART_HandleTypeDef *huaft, struct LHR_DATA lhrData);

void ldc1101_print_reg(UART_HandleTypeDef *huart, struct LDC1101 *ldc1101,
		uint8_t reg);

void ldc1101_print_all_reg(UART_HandleTypeDef *huart, struct LDC1101 *ldc1101);

void ldc1101_quick_print_rpl(UART_HandleTypeDef *huart, struct RPL_DATA rplData);

void ldc1101_quick_print_lhr(UART_HandleTypeDef *huart, struct LHR_DATA lhrData);



/*
 * UTILS
 */

uint8_t ldc1101_running_avg_8bit(uint8_t avg, uint8_t newValue, uint8_t delta);

uint16_t ldc1101_running_avg_16bit(uint16_t avg, uint16_t newValue, uint16_t delta);

uint32_t ldc1101_running_avg_32bit(uint32_t avg, uint32_t newValue, uint32_t delta);

void ldc1101_status_clear_bit(struct LDC1101 *ldc1101, uint8_t bit);

void ldc1101_status_set_bit(struct LDC1101 *ldc1101, uint8_t bit);

uint8_t ldc1101_mask_edit_byte(uint8_t oldData, uint8_t newData, uint8_t mask);

#endif /* INC_LDC1101_H_ */

/**
 * Copyright (c) 2021 2022 David Ryan
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
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

// -eof
