/*
 * ldc1101.cpp
 *
 *  Created on: Jun 28, 2021
 *      Author: David Ryan
 */

#include <string.h> // memcpy
#include <stdio.h>

#include "ldc1101.h"
#include "stm32f4xx_hal.h"

/************************************************
 *  ldc1101 extern const definitions
 ***********************************************/

/**
 * Register Addresses
 */
const uint8_t _REG_ADRS[32] = { 0x01, // - REG_RP_SET
		0x02, // - REG_TC1
		0x03, // - REG_TC2
		0x04, // - REG_DIG_CONFIG
		0x05, // - REG_ALT_CONFIG
		0x06, // - REG_RP_THRESH_H_LSB
		0x07, // - REG_RP_THRESH_H_MSB
		0x08, // - REG_RP_THRESH_L_LSB
		0x09, // - REG_RP_THRESH_L_MSB
		0x0a, // - REG_INTB_MODE
		0x0b, // - REG_START_CONFIG
		0x0c, // - REG_D_CONF
		0x16, // - REG_L_THRESH_HI_LSB
		0x17, // - REG_L_THRESH_HI_MSB
		0x18, // - REG_L_THRESH_LO_LSB
		0x19, // - REG_L_THRESH_LO_MSB
		0x20, // - REG_STATUS
		0x21, // - REG_RP_DATA_LSB - R
		0x22, // - REG_RP_DATA_MSB - R
		0x23, // - REG_L_DATA_LSB - R
		0x24, // - REG_L_DATA_MSB - R
		0x30, // - REG_LHR_RCOUNT_LSB
		0x31, // - REG_LHR_RCOUNT_MSB
		0x32, // - REG_LHR_OFFSET_LSB
		0x33, // - REG_LHR_OFFSET_MSB
		0x34, // - REG_LHR_CONFIG
		0x38, // - REG_LHR_DATA_LSB - R
		0x39, // - REG_LHR_DATA_MID - R
		0x3a, // - REG_LHR_DATA_MSB - R
		0x3b, // - REG_LHR_STATUS - R
		0x3e, // - REG_RID - R
		0x3f // - REG_CHIP_ID - R
		};

/**
 * Default data on startup
 * used for restoration
 */
const uint8_t _REG_DEFAULT_VALS[32] = { 0x07,	// REG_RP_SET
		0x90,	// REG_TC1
		0xa0,	// REG_TC2
		0x03,	// REG_DIG_CONFIG
		0x00,	// REG_ALT_CONFIG
		0x00,	// REG_RP_THRESH_H_LSB
		0x00,	// REG_RP_THRESH_H_MSB
		0x00,	// REG_RP_THRESH_L_LSB
		0x00,	// REG_RP_THRESH_L_MSB
		0x00,	// REG_INTB_MODE
		0x01,	// REG_START_CONFIG
		0x00,	// REG_D_CONF
		0x00,	// REG_L_THRESH_HI_LSB
		0x00,	// REG_L_THRESH_HI_MSB
		0x00,	// REG_L_THRESH_LO_LSB
		0x00,	// REG_L_THRESH_LO_MSB
		0x00,	// REG_STATUS
		0x00,	// REG_RP_DATA_LSB - R
		0x00,	// REG_RP_DATA_MSB - R
		0x00,	// REG_L_DATA_LSB - R
		0x00,	// REG_L_DATA_MSB - R
		0x00,	// REG_LHR_RCOUNT_LSB
		0x00,	// REG_LHR_RCOUNT_MSB
		0x00,	// REG_LHR_OFFSET_LSB
		0x00,	// REG_LHR_OFFSET_MSB
		0x00,	// REG_LHR_CONFIG
		0x00,	// REG_LHR_DATA_LSB - R
		0x00,	// REG_LHR_DATA_MID - R
		0x00,	// REG_LHR_DATA_MSB - R
		0x00,	// REG_LHR_STATUS - R
		0x02,	// REG_RID - R
		0xd4,	// REG_CHIP_ID - R
		};

/************************************************
 *  ldc1101 HAL_SPI functions
 ***********************************************/

/**
 * @brief	Sets or clears the cs pin
 * @note   	This function uses GPIOx_BSRR register to allow atomic read/modify
 *         	accesses. In this way, there is no risk of an IRQ occurring between
 *         	the read and the modify access.
 * @param  	Structure containing GPIOx and GPIO_Pin data
 * @param  	PinState specifies the value to be written to the selected bit.
 *          This parameter can be one of the GPIO_PinState enum values:
 *            	@arg GPIO_PIN_RESET: to clear the port pin
 *            	@arg GPIO_PIN_SET: to set the port pin
 *@return	None
 */
void ldc1101_hal_write_cs(struct GPIO_PIN *pin, uint8_t value) {

	HAL_GPIO_WritePin(pin->GPIOx, pin->GPIO_Pin, value);

}


/**
  * @brief	Toggles the specified GPIO pins.
  * @param  GPIOx Where x can be (A..K) to select the GPIO peripheral for STM32F429X device or
  *                      x can be (A..I) to select the GPIO peripheral for STM32F40XX and STM32F427X devices.
  *         Not advised to be used
  * @param  GPIO_Pin Specifies the pins to be toggled.
  * @return None
  */
void ldc1101_hal_toggle_cs(struct GPIO_PIN *pin) {
	HAL_GPIO_TogglePin(pin->GPIOx, pin->GPIO_Pin);
}

/**
 * @brief  Transmit an amount of data in blocking mode.
 * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
 *               the configuration information for SPI module.
 * @param  pData pointer to data buffer
 * @param  Size amount of data to be sent
 * @param  Timeout Timeout duration
 * @return HAL status
 */
void ldc1101_hal_spi_transmit(SPI_HandleTypeDef *hspi, uint8_t *pData,
		uint16_t Size, uint32_t Timeout) {
	HAL_SPI_Transmit(hspi, pData, Size, Timeout);
}

/**
  * @brief  Transmit and Receive an amount of data in blocking mode.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param  pTxData pointer to transmission data buffer
  * @param  pRxData pointer to reception data buffer
  * @param  Size amount of data to be sent and received
  * @param  Timeout Timeout duration
  * @return HAL status
  */
void ldc1101_hal_spi_transmit_receive(SPI_HandleTypeDef *hspi, uint8_t *pTxData,
		uint8_t *pRxData, uint16_t Size, uint32_t Timeout) {
	HAL_SPI_TransmitReceive(hspi, pTxData, pRxData, Size, Timeout);
}

/************************************************
 *  ldc1101 HAL_RCC functions
 ***********************************************/

/**
 * @brief	Using the system specific HAL library deactivate the clock pin
 * 			and bring it low
 * @param	void
 */
void ldc1101_hal_deactivate_clkin() {

	// unimplemented on stm32f44re

}

/**
 * @brief	Using the system specific HAL library activate the clock pin
 * 			at a constant frequency
 * @param	void
 */
void ldc1101_hal_activate_clkin() {

	// unimplemented on stm32f44re

}

/************************************************
 *  ldc1101 HAL_UART functions
 ***********************************************/

/**
 * @brief  Sends an amount of data in blocking mode.
 * @note   When UART parity is not enabled (PCE = 0), and Word Length is configured to 9 bits (M1-M0 = 01),
 *         the sent data is handled as a set of u16. In this case, Size must indicate the number
 *         of u16 provided through pData.
 * @param  huart Pointer to a UART_HandleTypeDef structure that contains
 *               the configuration information for the specified UART module.
 * @param  pData Pointer to data buffer (u8 or u16 data elements).
 * @param  Size  Amount of data elements (u8 or u16) to be sent
 * @param  Timeout Timeout duration
 */
void ldc1101_hal_uart_transmit(UART_HandleTypeDef *huart, uint8_t *pData,
		uint16_t Size, uint32_t Timeout) {

	HAL_UART_Transmit(huart, pData, Size, Timeout);

}

/************************************************
 *  ldc1101 driver functions
 ***********************************************/

/**
 * @brief	Sends a empty spi data buffer without activating the
 * 			cs line
 * @note	No effect is had on the chip
 * @param	ldc1101 pointer to an LDC1101 struct
 * @return	Void
 */
void ldc1101_test_data_packet(struct LDC1101 *ldc1101) {

	uint8_t pData;

	ldc1101_hal_spi_transmit(ldc1101->hspi, &pData, 1, 100);

	HAL_Delay(10);
}

/**
 * @brief	Pulls cs low and writes the data buf to the spi line
 * @note	na
 * @param	ldc1101 pointer to an LDC1101 sturct
 * @param	reg Address of the register to write to
 * @param	data Data to be written
 * @return	void
 */
void ldc1101_write_reg(struct LDC1101 *ldc1101, uint8_t reg, uint8_t data) {

	reg = reg & 0b01111111; // w/r bit is MSB, write = 0

	uint8_t rxData[2] = { reg, data };

	ldc1101_hal_write_cs(&ldc1101->cs, 0);

	ldc1101_hal_spi_transmit(ldc1101->hspi, rxData, 2, 100);

	ldc1101_hal_write_cs(&ldc1101->cs, 1);

}

/**
 * @brief	Pulls cs low and reads the value of the given registure
 * @note	na
 * @param	ldc1101 A pointer to an LDC1101 sturct
 * @param	reg The address of the register to be read
 * @return	uint8_t value of reg
 */
uint8_t ldc1101_read_reg(struct LDC1101 *ldc1101, uint8_t reg) {

	reg = reg | 0b10000000; // w/r bit is MSB, read = 1

	uint8_t txData[2] = { reg, 0x0 };

	uint8_t rxData[2];

	ldc1101_hal_write_cs(&ldc1101->cs, 0);

	ldc1101_hal_spi_transmit_receive(ldc1101->hspi, txData, rxData, 2, 100);

	ldc1101_hal_write_cs(&ldc1101->cs, 1);

	return rxData[1];
}

/**
 * @brief	Pulls cs low and writes a value to the given register
 * 			checks if value was written
 * @note	na
 * @param	ldc1101 A pointer to a LDC1101 struct
 * @param	reg The address of the register to be written to
 * @param	data The data to write
 * @return	uint8_t ldc1101 OK value (true or false)
 */
uint8_t ldc1101_write_and_check_reg(struct LDC1101 *ldc1101, uint8_t reg,
		uint8_t data) {

	ldc1101_write_reg(ldc1101, reg, data);

	uint8_t returnData = ldc1101_read_reg(ldc1101, reg);

	if (data == returnData) {
		return LDC1101_OK;
	}

	return LDC1101_NOT_OK;
}

/**
 * @brief	Pulls cs low and writes edits the register data contents
 * 			based on the new data and the mask
 * @note	na
 * @param	ldc1101 A pointer to a LDC1101 sturct
 * @param	reg The register for the data to be written to
 * @param	data The new data to be written
 * @param	mask The mask that determines what portion of the data byte to write
 * 			to the register
 * @return	void
 */
void ldc1101_edit_reg(struct LDC1101 *ldc1101, uint8_t reg, uint8_t data,
		uint8_t mask) {

	uint8_t oldData = ldc1101_read_reg(ldc1101, reg);

	uint8_t newData = ldc1101_mask_edit_byte(oldData, data, mask);

	ldc1101_write_reg(ldc1101, reg, newData);

}

/**
 * @brief	Pulls cs low and edits the regesture data contents
 * 			based on the new data and the mask
 * @note	na
 * @param	ldc1101 A pointer to a LDC1101 struct
 * @param	reg The register for the data to be editted in
 * @param	data The new data to be written
 * @param	mask The mask that determines what portion fo the data byte to write
 * 			to the register
 */

uint8_t ldc1101_edit_and_check_reg(struct LDC1101 *ldc1101, uint8_t reg,
		uint8_t data, uint8_t mask) {

	uint8_t oldData = ldc1101_read_reg(ldc1101, reg);

	uint8_t newData = ldc1101_mask_edit_byte(oldData, data, mask);

	return ldc1101_write_and_check_reg(ldc1101, reg, newData);
}

/**
 * @brief	Reads a specified number of registers into a buffer
 * @note	na
 * @param	ldc1101 A pointer to an LDC1101 struct
 * @param	startReg Starting register address
 * @param	buf Buffer to read data into
 * @param	length The number of registers to read
 */

void ldc1101_read_consecutive_reg(struct LDC1101 *ldc1101, uint8_t startReg,
		uint8_t *buf, uint8_t length) {

	startReg = startReg | 0b10000000;

	uint8_t txData[length + 1];

	txData[0] = startReg;

	// initialize array to 0
	for (uint8_t i = 1; i <= length; i++) {
		txData[i] = 0x0;
	}

	uint8_t rxData[length + 1];

	ldc1101_hal_write_cs(&ldc1101->cs, 0);

	ldc1101_hal_spi_transmit_receive(ldc1101->hspi, txData, rxData, length + 1,
			100);

	ldc1101_hal_write_cs(&ldc1101->cs, 1);

	memcpy(buf, rxData + 1, length);

	return;
}

/**
 * @brief	Reads all registers of the ldc1101 chip
 * @note	No return value or buffer, used for debugging purposes only
 * 			viewable on a logic analyzer
 * @param	ldc1101 A pointer to a LDC1101 sturct
 */
void ldc1101_read_all_reg(struct LDC1101 *ldc1101) {
	uint8_t txData[64];
	txData[0] = 0b10000001;
	for (uint8_t i = 1; i <= 64; i++) {
		txData[i] = i;
	}

	ldc1101_hal_write_cs(&ldc1101->cs, 0);

	ldc1101_hal_spi_transmit(ldc1101->hspi, txData, 64, 100);

	ldc1101_hal_write_cs(&ldc1101->cs, 1);

}

/*
 * POWER CONTROL
 * returns _LDC1101_OK or _LDC1101_NOT_OK
 */

/**
 * @brief	Writes the proper values to proper registers to enable
 * 			shutdown mode
 * @note	Clock must be pulled low
 * @param	ldc1101 A pointer to a LDC1101 struct
 */
uint8_t ldc1101_shutdown(struct LDC1101 *ldc1101) {

	/**
	 * 1. Set ALT_CONFIG.SHUTDOWN_EN = 1 (register 0x05-bit[1]).
	 * 2. Stop toggling the CLKIN pin input and drive the CLKIN pin Low.
	 * 3. Set START_CONFIG.FUNC_MODE = b10 (register 0x0B:bits[1:0]). This register can be written while the
	 * LDC1101 is in active mode; on completion of the register write the LDC1101 will enter shutdown.
	 */

	if (ldc1101_enter_sleep(ldc1101) == LDC1101_NOT_OK) {
		return LDC1101_NOT_OK;
	}

	if (ldc1101_write_and_check_reg(ldc1101,
	_LDC1101_REG_CFG_ADDITIONAL_DEVICE_SETTINGS,
	_LDC1101_FUNC_MODE_SHUTDOWN_MODE) == LDC1101_NOT_OK) {
		return LDC1101_NOT_OK;
	}

	ldc1101_hal_deactivate_clkin();

	if (ldc1101_write_and_check_reg(ldc1101, _LDC1101_REG_CFG_POWER_STATE,
	_LDC1101_FUNC_MODE_SHUTDOWN_MODE) == LDC1101_NOT_OK) {
		return LDC1101_NOT_OK;
	}

	return LDC1101_OK;

}

/**
 * @brief	Writes the proper values to the proper registers
 * 			to enable startup mode
 * @note
 * @param	ldc1101 A ponter to a LDC1101 struct
 */
uint8_t ldc1101_startup(struct LDC1101 *ldc1101) {

	ldc1101_test_data_packet(ldc1101);

	ldc1101_read_all_reg(ldc1101);

	HAL_Delay(10);

	if (ldc1101_exit_sleep(ldc1101) == LDC1101_NOT_OK) {
		return LDC1101_NOT_OK;
	}

	HAL_Delay(10);

	ldc1101_read_all_reg(ldc1101);

	return LDC1101_OK;
}

/**
 * @brief	Writes the proper values to the proper registers
 * 			to enable sleep mode
 * @note	na
 * @param	ldc1101 A pointer to a LDC1101 struct
 */
uint8_t ldc1101_enter_sleep(struct LDC1101 *ldc1101) {

	ldc1101_status_clear_bit(ldc1101, _LDC1101_STATUS_POWER_STATE);

	return ldc1101_write_and_check_reg(ldc1101, _LDC1101_REG_CFG_POWER_STATE,
	_LDC1101_FUNC_MODE_SLEEP_MODE);
}

/**
 * @brief	Writes the proper values to the proper registers
 * 			to disable sleep mode
 * @note	na
 * @param	ldc1101 A pointer to an LDC1101 struct
 */
uint8_t ldc1101_exit_sleep(struct LDC1101 *ldc1101) {

	ldc1101_status_set_bit(ldc1101, _LDC1101_STATUS_POWER_STATE);

	return ldc1101_write_and_check_reg(ldc1101, _LDC1101_REG_CFG_POWER_STATE,
	_LDC1101_FUNC_MODE_ACTIVE_CONVERSION_MODE);
}

/**
 * @brief	Reads current sleep state value and toggles it
 * @note	na
 * @param	ldc1101 A ponter to an LDC1101 struct
 */
uint8_t ldc1101_toggle_sleep(struct LDC1101 *ldc1101) {
	uint8_t currentPowerState = ldc1101_read_reg(ldc1101,
	_LDC1101_REG_CFG_POWER_STATE);
	return ldc1101_write_and_check_reg(ldc1101, _LDC1101_REG_CFG_POWER_STATE,
			!currentPowerState);

}

/**
 * @brief	Calls shutdown and startup sequentially
 * @note	na
 * @param	ldc1101 A pointer to an LDC1101 struct
 */
uint8_t ldc1101_restart(struct LDC1101 *ldc1101) {

	if (ldc1101_shutdown(ldc1101) == LDC1101_NOT_OK) {
		return LDC1101_NOT_OK;
	}

	HAL_Delay(300);

	if (ldc1101_startup(ldc1101) == LDC1101_NOT_OK) {
		return LDC1101_NOT_OK;
	}

	return LDC1101_OK;

}

/* INITIALIZATION */

/**
 * @brief	Costume function to initialize the chip
 * @note	na
 * @param	ldc1101 A pointer to a LDC1101 struct
 */
uint8_t ldc1101_init(struct LDC1101 *ldc1101) {

	ldc1101_read_all_reg(ldc1101);

	HAL_Delay(10);

	if (ldc1101_exit_sleep(ldc1101) == LDC1101_NOT_OK) {
		return LDC1101_NOT_OK;
	}

	HAL_Delay(10);

	ldc1101_read_all_reg(ldc1101);

	return LDC1101_OK;
}

/**
 * @brief	Costume function to set up rpl readings
 * @note	na
 * @param	ldc1101 A pointer to a LDC1101 struct
 * @param
 */
uint8_t ldc1101_rpl_setup(struct LDC1101 *ldc1101) {

	// no initial setup required for rpl measurements
	return LDC1101_OK;
}

/**
 * @brief	Costume function to set up lhr readings
 * @note	na
 * @param	ldc1101 A pointer to a LDC1101 struct
 */
uint8_t ldc1101_lhr_setup(struct LDC1101 *ldc1101) {

	uint8_t pData;

	ldc1101_hal_spi_transmit(ldc1101->hspi, &pData, 1, 100);

	HAL_Delay(10);

	ldc1101_read_all_reg(ldc1101);

	HAL_Delay(10);

	ldc1101_write_reg(ldc1101, _LDC1101_REG_LHR_OFFSET_LSB, 0xff);

	HAL_Delay(10);

	ldc1101_write_reg(ldc1101, _LDC1101_REG_CFG_LHR, 0x1);

	HAL_Delay(10);

	ldc1101_read_all_reg(ldc1101);

	return LDC1101_OK;

}

/*
 * OPERATION AND MODE CONFIGERATION
 * returns _LDC1101_OK or _LDC1101_NOT_OK
 */

/**
 * @brief	Enables optimized induction reading
 * @note	Optimize sensor drive signal for L measurements (for both High-Res L and L
 measurement). When LOPTIMAL is enabled, RP measurements are not
 completed. It is also necessary to set DOK_REPORT=1 when this mode is
 enabled.
 * @param	ldc1101 A pointer to a LDC1101 struct
 */
uint8_t ldc1101_enable_optimize_for_l(struct LDC1101 *ldc1101) {

	if ((ldc1101->status & _LDC1101_STATUS_POWER_STATE) == 1) {

		if (ldc1101_exit_sleep(ldc1101) == LDC1101_NOT_OK) {
			return LDC1101_NOT_OK;
		}

		uint8_t okFlag = ldc1101_write_and_check_reg(ldc1101,
		_LDC1101_REG_CFG_ADDITIONAL_DEVICE_SETTINGS,
		_LDC1101_ALT_CFG_L_OPTIMAL_ENABLE);

		if (ldc1101_enter_sleep(ldc1101) == LDC1101_NOT_OK) {
			return LDC1101_NOT_OK;
		}

		return okFlag;

	}
	return ldc1101_write_and_check_reg(ldc1101,
	_LDC1101_REG_CFG_ADDITIONAL_DEVICE_SETTINGS,
	_LDC1101_ALT_CFG_L_OPTIMAL_ENABLE);

}

/**
 * @brief 	Disables optimized induction readings
 * @note	na
 * @param	ldc1101 A pointer to a LDC1101 struct
 */
uint8_t ldc1101_disable_optimize_for_l(struct LDC1101 *ldc1101) {

	if ((ldc1101->status & _LDC1101_STATUS_POWER_STATE) == 1) {

		if (ldc1101_exit_sleep(ldc1101) == LDC1101_NOT_OK) {
			return LDC1101_NOT_OK;
		}

		uint8_t okFlag = ldc1101_write_and_check_reg(ldc1101,
		_LDC1101_REG_CFG_ADDITIONAL_DEVICE_SETTINGS,
		_LDC1101_ALT_CFG_L_OPTIMAL_DISABLED);

		if (ldc1101_enter_sleep(ldc1101) == LDC1101_NOT_OK) {
			return LDC1101_NOT_OK;
		}

		return okFlag;

	}
	return ldc1101_write_and_check_reg(ldc1101,
	_LDC1101_REG_CFG_ADDITIONAL_DEVICE_SETTINGS,
	_LDC1101_ALT_CFG_L_OPTIMAL_DISABLED);

}

/**
 * @brief	Enables and configures the dynamic range for rp readings
 * @note	na
 * @param	ldc1101 A pointer to a LDC1101 struct
 * @param	RP_MAX Maximum value for the dynamic range
 * @param	RP_MIN Minimum value for the dynamic range
 */
uint8_t ldc1101_enable_rp_dynamic_range(struct LDC1101 *ldc1101, uint8_t RP_MAX,
		uint8_t RP_MIN) {

	uint8_t data = _LDC1101_RP_SET_RP_MAX_IS_DRIVEN | RP_MAX | RP_MIN;

	if ((ldc1101->status & _LDC1101_STATUS_POWER_STATE) == 1) {

		if (ldc1101_exit_sleep(ldc1101) == LDC1101_NOT_OK) {
			return LDC1101_NOT_OK;
		}

		uint8_t okFlag = ldc1101_write_and_check_reg(ldc1101,
		_LDC1101_REG_CFG_RP_MEASUREMENT_DYNAMIC_RANGE, data);

		if (ldc1101_enter_sleep(ldc1101) == LDC1101_NOT_OK) {
			return LDC1101_NOT_OK;
		}

		return okFlag;

	}
	return ldc1101_write_and_check_reg(ldc1101,
	_LDC1101_REG_CFG_RP_MEASUREMENT_DYNAMIC_RANGE, data);

}

/**
 * @brief	Disables any dynamic range settings for rp readings
 * @note	na
 * @param	ldc1101 A pointer to a LDC1101 struct
 */
uint8_t ldc1101_disable_rp_dynamic_range(struct LDC1101 *ldc1101) {

	if ((ldc1101->status & _LDC1101_STATUS_POWER_STATE) == 1) {

		if (ldc1101_exit_sleep(ldc1101) == LDC1101_NOT_OK) {
			return LDC1101_NOT_OK;
		}

		uint8_t okFlag = ldc1101_write_and_check_reg(ldc1101,
		_LDC1101_REG_CFG_RP_MEASUREMENT_DYNAMIC_RANGE,
		_LDC1101_RP_SET_RP_MAX_CURRENT_IS_IGNORED);

		if (ldc1101_enter_sleep(ldc1101) == LDC1101_NOT_OK) {
			return LDC1101_NOT_OK;
		}

		return okFlag;

	}
	return ldc1101_write_and_check_reg(ldc1101,
	_LDC1101_REG_CFG_RP_MEASUREMENT_DYNAMIC_RANGE,
	_LDC1101_RP_SET_RP_MAX_CURRENT_IS_IGNORED);

}

/**
 * @brief	Configures the dynamic range for rp readings
 * @note	This setting improves the RP measurement accuracy for very high Q coils by
 *	 	 	driving 0A as the RPMAX current drive. Typically, sensors with Q > 50 can benefit
 *		 	from enabling this mode.
 * @param	ldc1101 A pointer to a LDC1101 struct
 * @param	RP_MAX The maximum input dynamic range for the sensor RP measurement,
 * 			The programmed RP_MIN setting must not exceed the programmed RP_MAX setting.
 * @param	RP_MIN The minimum input dynamic range for the sensor RP measurement,
 * 			The programmed RP_MIN setting must not exceed the programmed RP_MAX setting.
 *
 */
uint8_t ldc1101_configure_rp_dynamic_range(struct LDC1101 *ldc1101,
		uint8_t RP_MAX, uint8_t RP_MIN) {

	uint8_t data = _LDC1101_RP_SET_RP_MAX_IS_DRIVEN | RP_MAX | RP_MIN;

	if ((ldc1101->status & _LDC1101_STATUS_POWER_STATE) == 1) {

		if (ldc1101_exit_sleep(ldc1101) == LDC1101_NOT_OK) {
			return LDC1101_NOT_OK;
		}

		uint8_t okFlag = ldc1101_write_and_check_reg(ldc1101,
		_LDC1101_REG_CFG_RP_MEASUREMENT_DYNAMIC_RANGE, data);

		if (ldc1101_enter_sleep(ldc1101) == LDC1101_NOT_OK) {
			return LDC1101_NOT_OK;
		}

		return okFlag;

	}
	return ldc1101_write_and_check_reg(ldc1101,
	_LDC1101_REG_CFG_RP_MEASUREMENT_DYNAMIC_RANGE, data);

}

/**
 * @brief	Enables intb reporting mode
 * @note	na
 * @param	ldc1101 A pointer to a LDC1101 struct
 */
uint8_t ldc1101_enable_intb_mode(struct LDC1101 *ldc1101, uint8_t MODE) {

	ldc1101_status_set_bit(ldc1101, _LDC1101_STATUS_INTB_MODE);

	uint8_t data = _LDC1101_INTB_MODE_REPORT_INTB_ON_SDO_PIN | MODE;

	if ((ldc1101->status & _LDC1101_STATUS_POWER_STATE) == 1) {

		if (ldc1101_exit_sleep(ldc1101) == LDC1101_NOT_OK) {
			return LDC1101_NOT_OK;
		}

		uint8_t okFlag = ldc1101_write_and_check_reg(ldc1101,
		_LDC1101_REG_CFG_INTB_MODE, data);

		if (ldc1101_enter_sleep(ldc1101) == LDC1101_NOT_OK) {
			return LDC1101_NOT_OK;
		}

		return okFlag;

	}
	return ldc1101_write_and_check_reg(ldc1101,
	_LDC1101_REG_CFG_INTB_MODE, data);

}

/**
 * @brief	Disable intb reporting mode
 * @note	na
 * @param	ldc1101 A pointer to a LDC1101 struct
 */
uint8_t ldc1101_disable_intb_mode(struct LDC1101 *ldc1101) {

	ldc1101_status_clear_bit(ldc1101, _LDC1101_STATUS_INTB_MODE);

	if ((ldc1101->status & _LDC1101_STATUS_POWER_STATE) == 1) {

		if (ldc1101_exit_sleep(ldc1101) == LDC1101_NOT_OK) {
			return LDC1101_NOT_OK;
		}

		uint8_t okFlag = ldc1101_write_and_check_reg(ldc1101,
		_LDC1101_REG_CFG_INTB_MODE,
		_LDC1101_INTB_MODE_DONT_REPORT_INTB_ON_SDO_PIN);

		if (ldc1101_enter_sleep(ldc1101) == LDC1101_NOT_OK) {
			return LDC1101_NOT_OK;
		}

		return okFlag;

	}
	return ldc1101_write_and_check_reg(ldc1101,
	_LDC1101_REG_CFG_INTB_MODE,
	_LDC1101_INTB_MODE_DONT_REPORT_INTB_ON_SDO_PIN);

}

/**
 * @brief	Configures the timing values for tc1
 * @note	na
 * @param	ldc1101 A pointer to a LDC1101 struct
 * @param	R_VAL The resistive component used to configure internal time constant 1
 * @param	C_VAL The capacitive component used to configure internal time constant 1
 */
uint8_t ldc1101_configure_tc1(struct LDC1101 *ldc1101, uint8_t R_VAL,
		uint8_t C_VAL) {

	uint8_t data = R_VAL | C_VAL;

	if ((ldc1101->status & _LDC1101_STATUS_POWER_STATE) == 1) {

		if (ldc1101_exit_sleep(ldc1101) == LDC1101_NOT_OK) {
			return LDC1101_NOT_OK;
		}

		uint8_t okFlag = ldc1101_write_and_check_reg(ldc1101,
		_LDC1101_REG_CFG_INTERNAL_TIME_CONSTANT_1, data);

		if (ldc1101_enter_sleep(ldc1101) == LDC1101_NOT_OK) {
			return LDC1101_NOT_OK;
		}

		return okFlag;

	}
	return ldc1101_write_and_check_reg(ldc1101,
	_LDC1101_REG_CFG_INTERNAL_TIME_CONSTANT_1, data);

}

/**
 * @brief	Configures the timing values for tc2
 * @note	na
 * @param	ldc1101 A pointer to a LDC1101 struct
 * @param	R_VAL The resistive component used to configure internal time constant 2
 * @param	C_VAL The capacitive component used to configure internal time constant 2
 */
uint8_t ldc1101_configure_tc2(struct LDC1101 *ldc1101, uint8_t R_VAL,
		uint8_t C_VAL) {

	uint8_t data = R_VAL | C_VAL;

	if ((ldc1101->status & _LDC1101_STATUS_POWER_STATE) == 1) {

		if (ldc1101_exit_sleep(ldc1101) == LDC1101_NOT_OK) {
			return LDC1101_NOT_OK;
		}

		uint8_t okFlag = ldc1101_write_and_check_reg(ldc1101,
		_LDC1101_REG_CFG_INTERNAL_TIME_CONSTANT_2, data);

		if (ldc1101_enter_sleep(ldc1101) == LDC1101_NOT_OK) {
			return LDC1101_NOT_OK;
		}

		return okFlag;

	}
	return ldc1101_write_and_check_reg(ldc1101,
	_LDC1101_REG_CFG_INTERNAL_TIME_CONSTANT_2, data);

}

/**
 * @brief	configures the digital settings of the ldc1101
 * @note	na
 * @param	ldc1101 A pointer to a LDC1101 struct
 * @param	MIN_FREQ Minimum frequency, Configure this register based on the lowest possible sensor frequency. This is
 typically when the target is providing minimum interaction with the sensor,
 although with some steel and ferrite targets, the minimum sensor frequency
 occurs with maximum target interaction.
 * @param	RESP_TIME The Response Time, which is the number of sensor periods used per
 conversion. This setting applies to the RP and Standard Resolution L
 measurement, but not the High Resolution L measurement.
 */
uint8_t ldc1101_configure_dig(struct LDC1101 *ldc1101, uint8_t MIN_FREQ,
		uint8_t RESP_TIME) {

	uint8_t data = MIN_FREQ | RESP_TIME;

	if ((ldc1101->status & _LDC1101_STATUS_POWER_STATE) == 1) {

		if (ldc1101_exit_sleep(ldc1101) == LDC1101_NOT_OK) {
			return LDC1101_NOT_OK;
		}

		uint8_t okFlag = ldc1101_write_and_check_reg(ldc1101,
		_LDC1101_REG_CFG_RP_L_CONVERSION_INTERVAL, data);

		if (ldc1101_enter_sleep(ldc1101) == LDC1101_NOT_OK) {
			return LDC1101_NOT_OK;
		}

		return okFlag;

	}
	return ldc1101_write_and_check_reg(ldc1101,
	_LDC1101_REG_CFG_RP_L_CONVERSION_INTERVAL, data);

}

/**
 * @brief	Configures the high and low threshold registers
 * 			for rp readings
 * @note	na
 * @param	ldc1101 A pointer to a LDC1101 struct
 * @param	RP_THRESH_HI The 16 bit value for the high threshold
 * @parma	RP_THRESH_LOW The 16 bit value for the low threshold
 */
uint8_t ldc1101_configure_rp_high_low_threshold(struct LDC1101 *ldc1101,
		uint16_t RP_THRESH_HI, uint16_t RP_THRESH_LOW) {

	uint8_t hiLsb = RP_THRESH_HI;
	uint8_t hiMsb = RP_THRESH_HI >> 8;
	uint8_t lowLsb = RP_THRESH_LOW;
	uint8_t lowMsb = RP_THRESH_LOW >> 8;

	if ((ldc1101->status & _LDC1101_STATUS_POWER_STATE) == 1) {

		if (ldc1101_exit_sleep(ldc1101) == LDC1101_NOT_OK) {
			return LDC1101_NOT_OK;
		}

		uint8_t okFlagHiLsb = ldc1101_write_and_check_reg(ldc1101,
		_LDC1101_REG_RP_THRESH_H_LSB, hiLsb);
		uint8_t okFlagHiMsb = ldc1101_write_and_check_reg(ldc1101,
		_LDC1101_REG_RP_THRESH_H_MSB, hiMsb);

		uint8_t okFlagLowLsb = ldc1101_write_and_check_reg(ldc1101,
		_LDC1101_REG_RP_THRESH_L_LSB, lowLsb);
		uint8_t okFlagLowMsb = ldc1101_write_and_check_reg(ldc1101,
		_LDC1101_REG_RP_THRESH_L_MSB, lowMsb);

		if (ldc1101_enter_sleep(ldc1101) == LDC1101_NOT_OK) {
			return LDC1101_NOT_OK;
		}

		return okFlagHiLsb | okFlagHiMsb | okFlagLowLsb | okFlagLowMsb;

	}

	uint8_t okFlagHiLsb = ldc1101_write_and_check_reg(ldc1101,
	_LDC1101_REG_RP_THRESH_H_LSB, hiLsb);
	uint8_t okFlagHiMsb = ldc1101_write_and_check_reg(ldc1101,
	_LDC1101_REG_RP_THRESH_H_MSB, hiMsb);

	uint8_t okFlagLowLsb = ldc1101_write_and_check_reg(ldc1101,
	_LDC1101_REG_RP_THRESH_L_LSB, lowLsb);
	uint8_t okFlagLowMsb = ldc1101_write_and_check_reg(ldc1101,
	_LDC1101_REG_RP_THRESH_L_MSB, lowMsb);

	return okFlagHiLsb | okFlagHiMsb | okFlagLowLsb | okFlagLowMsb;

}

/**
 * @brief	Configures the high and low threshold registers
 * 			for l readings
 * @note	na
 * @param	ldc1101 A pointer to a LDC1101 struct
 * @param	RP_THRESH_HI The 16 bit value for the high threshold
 * @parma	RP_THRESH_LOW The 16 bit value for the low threshold
 */
uint8_t ldc1101_configure_l_high_low_threshold(struct LDC1101 *ldc1101,
		uint16_t L_THRESH_HI, uint16_t L_THRESH_LOW) {

	uint8_t hiLsb = L_THRESH_HI;
	uint8_t hiMsb = L_THRESH_HI >> 8;
	uint8_t lowLsb = L_THRESH_LOW;
	uint8_t lowMsb = L_THRESH_LOW >> 8;

	if ((ldc1101->status & _LDC1101_STATUS_POWER_STATE) == 1) {

		if (ldc1101_exit_sleep(ldc1101) == LDC1101_NOT_OK) {
			return LDC1101_NOT_OK;
		}

		uint8_t okFlagHiLsb = ldc1101_write_and_check_reg(ldc1101,
		_LDC1101_REG_L_THRESH_HI_LSB, hiLsb);
		uint8_t okFlagHiMsb = ldc1101_write_and_check_reg(ldc1101,
		_LDC1101_REG_L_THRESH_HI_MSB, hiMsb);

		uint8_t okFlagLowLsb = ldc1101_write_and_check_reg(ldc1101,
		_LDC1101_REG_L_THRESH_LO_LSB, lowLsb);
		uint8_t okFlagLowMsb = ldc1101_write_and_check_reg(ldc1101,
		_LDC1101_REG_L_THRESH_LO_MSB, lowMsb);

		if (ldc1101_enter_sleep(ldc1101) == LDC1101_NOT_OK) {
			return LDC1101_NOT_OK;
		}

		return okFlagHiLsb | okFlagHiMsb | okFlagLowLsb | okFlagLowMsb;

	}

	uint8_t okFlagHiLsb = ldc1101_write_and_check_reg(ldc1101,
	_LDC1101_REG_L_THRESH_HI_LSB, hiLsb);
	uint8_t okFlagHiMsb = ldc1101_write_and_check_reg(ldc1101,
	_LDC1101_REG_L_THRESH_HI_MSB, hiMsb);

	uint8_t okFlagLowLsb = ldc1101_write_and_check_reg(ldc1101,
	_LDC1101_REG_L_THRESH_LO_LSB, lowLsb);
	uint8_t okFlagLowMsb = ldc1101_write_and_check_reg(ldc1101,
	_LDC1101_REG_L_THRESH_LO_MSB, lowMsb);

	return okFlagHiLsb | okFlagHiMsb | okFlagLowLsb | okFlagLowMsb;

}

/**
 * @brief
 * @note
 * @param
 * @param
 */
uint8_t ldc1101_configure_lhr_rcount(struct LDC1101 *ldc1101, uint16_t RCOUNT) {

	return 1; // TODO write lhr rcount config
}

/**
 * @brief
 * @note
 * @param
 * @param
 */
uint8_t ldc1101_configure_lhr_offset(struct LDC1101 *ldc1101, uint16_t OFFSET) {

	return 1; // TODO write lhr_offset config
}

/*
 * NORMAL OPERATION
 * returns data structures
 */

/**
 * @brief	Retrieves the chip id and rid
 * @note	Used for establishing connection purposes as id and rid values
 * 			are known constants ID: 0xd4, RID: 0x02
 * @param	ldc1101 A pointer to a LDC1101 struct
 */
struct LDC1101_CHIP ldc1101_get_id(struct LDC1101 *ldc1101) {

	struct LDC1101_CHIP chip;

	uint8_t buf[2];

	ldc1101_read_consecutive_reg(ldc1101, _LDC1101_REG_DEVICE_RID, buf, 2);

	chip.rid = buf[0];
	chip.id = buf[1];

	return chip;

}

/**
 * @brief	Retrieves rpl data from rpl data registers
 * @note	ldc1101 must be configured for rpl measurements for
 * 			best result
 * @param	ldc1101 A pointer to a LDC1101 struct
 * @param	data The RPL_DATA struct to be updated
 */
struct RPL_DATA ldc1101_read_rpl_data(struct LDC1101 *ldc1101,
		struct RPL_DATA data) {

	uint8_t buf[5];

	uint16_t rpCpy = data.rp;
	uint16_t lCpy = data.l;

	ldc1101_read_consecutive_reg(ldc1101, _LDC1101_REG_RP_L_MEASUREMENT_STATUS,
			buf, 5);

	data.status = buf[0];
	data.rp_lsb = buf[1];
	data.rp_msb = buf[2];
	data.l_lsb = buf[3];
	data.l_msb = buf[4];
	data.rp = buf[1] | buf[2] << 8;
	data.l = buf[3] | buf[4] << 8;

	data.rp_avg = ldc1101_running_avg_16bit(rpCpy, data.rp, data.avg_delta);
	data.l_avg = ldc1101_running_avg_16bit(lCpy, data.l, data.avg_delta);

	return data;
}

/**
 * @brief	Retrieves lhr
 * @note
 * @param	ldc1101 A ponter to a LDC1101 struct
 * @param	data The LHR_DATA struct to be updated
 */
struct LHR_DATA ldc1101_read_lhr_data(struct LDC1101 *ldc1101,
		struct LHR_DATA data) {

	uint8_t buf[4];

	uint32_t lCpy = data.l;

	ldc1101_read_consecutive_reg(ldc1101, _LDC1101_REG_LHR_DATA_LSB, buf, 4);

	data.status = buf[3];
	data.l_lsb = buf[0];
	data.l_mid = buf[1];
	data.l_msb = buf[2];
	data.l = buf[0] | buf[1] << 8 | buf[2] << 16;

	data.l_avg = ldc1101_running_avg_32bit(lCpy, data.l, data.avg_delta);

	return data;
}

/* UART PRINT */

/**
 * @brief	Given a chip struct it will print its contents in a human readable
 * 			form over a uart connection
 * @note	na
 * @param	huart A pointer to a huart struct
 * @param	ldc1101 A pointer to a LDC1101 struct
 * @param	chip A ldc1101_chip struct
 */
void ldc1101_print_chip(UART_HandleTypeDef *huart, struct LDC1101 *ldc1101,
		struct LDC1101_CHIP chip) {

	char msgRid[29];
	char msgId[28];

	sprintf(msgRid, "Device RID value: %8u\n\r", chip.rid);
	sprintf(msgId, "Device ID value: %8u\n\r", chip.id);

	HAL_UART_Transmit(huart, msgRid, sizeof(msgRid), 100);
	HAL_UART_Transmit(huart, msgId, sizeof(msgId), 100);

}

/**
 * @brief	Prints the contents of a given register over uart
 * @note	na
 * @param	huart A pointer to a huart struct
 * @param	ldc1101 A pointer to a LDC1101 struct
 * @param	reg The address of the reggister that is to be read
 */
void ldc1101_print_reg(UART_HandleTypeDef *huart, struct LDC1101 *ldc1101,
		uint8_t reg) {

	uint8_t returnData = ldc1101_read_reg(ldc1101, reg);

	char msg[43];

	uint8_t bin[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
	uint8_t i = 0;
	while (returnData > 0) {
		bin[i] = returnData % 2;
		returnData = returnData / 2;
		i++;
	}

	sprintf(msg, "REG: %8u | VALUE: %u%u%u%u%u%u%u%u - 0x%4x\n\r", reg, bin[7],
			bin[6], bin[5], bin[4], bin[3], bin[2], bin[1], bin[0], returnData);

	HAL_UART_Transmit(huart, msg, sizeof(msg), 100);

}

/**
 * @brief	Prints all registers from a ldc1101 over uart
 * @note	na
 * @param	huart A pointer to a huart struct
 * @param	ldc1101 A pointer to a LDC1101 struct
 */
void ldc1101_print_all_reg(UART_HandleTypeDef *huart, struct LDC1101 *ldc1101) {

	HAL_UART_Transmit(huart, "===================================\n\r", 37,
			100);

	for (uint8_t i = 0; i < NUMBER_OF_REGS; i++) {
		ldc1101_print_reg(huart, ldc1101, _REG_ADRS[i]);
	}
}

/**
 * @brief	Given a rpl_data struct it prints it in a human readable form
 * 			over uart
 * @note	na
 * @param	huart A pointer to a huart struct
 * @param	rplData A RPL_DATA struct
 */
void ldc1101_print_rpl_data(UART_HandleTypeDef *huart, struct RPL_DATA rplData) {

	char msg[85];

	uint8_t status = rplData.status;
	uint8_t bin[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
	uint8_t i = 0;
	while (status > 0) {
		bin[i] = status % 2;
		status /= 2;
		i++;
	}
	sprintf(msg,
			"Status: %u%u%u%u%u%u%u%u | RP: %8u, L: %8u |  RP_AVG: %8u, L_AVG: %8u \n\r",
			bin[7], bin[6], bin[5], bin[4], bin[3], bin[2], bin[1], bin[0],
			rplData.rp, rplData.l, rplData.rp_avg, rplData.l_avg);

	HAL_UART_Transmit(huart, msg, sizeof(msg), 100);

}

/**
 * @brief	Given a lhr_data struct it prints it in a human readable form
 * 			over uart
 * @note	na
 * @param	huart A pointer to a huart struct
 * @param	lhrData A lhr_data struct
 */
void ldc1101_print_lhr_data(UART_HandleTypeDef *huart, struct LHR_DATA lhrData) {

	char msg[59];

	uint8_t status = lhrData.status;
	uint8_t bin[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
	uint8_t i = 0;
	while (status > 0) {
		bin[i] = status % 2;
		status /= 2;
		i++;
	}

	sprintf(msg, "Status: %u%u%u%u%u%u%u%u | LHR: %10lu | LHR_AVG: %10lu\n\r",
			bin[7], bin[6], bin[5], bin[4], bin[3], bin[2], bin[1], bin[0],
			lhrData.l, lhrData.l_avg);

	HAL_UART_Transmit(huart, msg, sizeof(msg), 100);

}

/**
 * @brief	Given a rpl_data struct it prints the raw data without the status
 * 			or running average
 * @note	na
 * @param	huart A pointer to a huart struct
 * @param	rplData A RPL_DATA struct
 */
void ldc1101_quick_print_rpl(UART_HandleTypeDef *huart, struct RPL_DATA rplData) {

	char msg[28];

	sprintf(msg, "RP: %8u | L: %8u\r", rplData.rp, rplData.l);

	HAL_UART_Transmit(huart, msg, sizeof(msg), 100);

}

/**
 * @brief
 * @note
 * @param
 * @param
 */
void ldc1101_quick_print_lhr(UART_HandleTypeDef *huart, struct LHR_DATA lhrData) {

	char msg[17];

	sprintf(msg, "LHR: %10lu\r", lhrData.l);

	HAL_UART_Transmit(huart, msg, sizeof(msg), 100);

}

/*
 * UTILS
 */

/**
 * @brief
 * @note
 * @param
 * @param
 */
uint8_t ldc1101_running_avg_8bit(uint8_t avg, uint8_t newValue, uint8_t delta) {
	return (avg * ((delta - 1) / delta)) + (newValue / delta);
}

/**
 * @brief
 * @note
 * @param
 * @param
 */
uint16_t ldc1101_running_avg_16bit(uint16_t avg, uint16_t newValue,
		uint16_t delta) {
	return (avg * ((delta - 1) / delta)) + (newValue / delta);
}

/**
 * @brief
 * @note
 * @param
 * @param
 */
uint32_t ldc1101_running_avg_32bit(uint32_t avg, uint32_t newValue,
		uint32_t delta) {
	return (avg * ((delta - 1) / delta)) + (newValue / delta);
}

/**
 * @brief
 * @note
 * @param
 * @param
 */
void ldc1101_status_clear_bit(struct LDC1101 *ldc1101, uint8_t bit) {
	uint8_t notBit = ~bit;
	ldc1101->status &= notBit;
}

/**
 * @brief
 * @note
 * @param
 * @param
 */
void ldc1101_status_set_bit(struct LDC1101 *ldc1101, uint8_t bit) {
	ldc1101->status |= bit;
}

/**
 * @brief
 * @note
 * @param
 * @param
 * @return	uint8_t boolean true of false
 */
uint8_t ldc1101_status_check_bit(struct LDC1101 *ldc1101, uint8_t bit) {

	if ((ldc1101->status & bit) == bit) {
		return 1;
	}

	return 0;
}

/**
 * @brief	A function to edit a sub set of a byte
 * @note	Example:	oldData: 	0b00110011
 * 						data:		0b00011110
 * 						mask:		0b00111000
 * 						newData:	0b00011011
 * 			The set procedure enforces 1's and 0's i.e. it performs
 * 			both a bitwise & and bitwise |.
 * @param	oldData The original state of the data
 * @param	data The new data to be written
 * @param	mask The mask that determines which bits will be updated,
 * 			all bits from data that align with 1's in mask will be written
 * 			to oldData. All bits outside of the mask will stay the same.
 * @return	uint8_t Byte that has been edited
 */
uint8_t ldc1101_mask_edit_byte(uint8_t oldData, uint8_t data, uint8_t mask) {

	/**
	 * Step by step method
	 * uint8_t maskedData = data & mask;
	 * uint8_t notMask = ~mask;
	 * uint8_t newData = oldData | maskedData;
	 * uint8_t notMaskedData = maskedData | notMask;
	 * newData &= notMaskedData;
	 * return newData;
	 */

	// one liner
	return (oldData | (data & mask)) & ((data & mask) | ~mask);
}

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
