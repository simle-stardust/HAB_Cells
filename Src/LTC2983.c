/*
 * LTC2983.c
 *
 *  Created on: 24.11.2019
 *      Author: Szymon
 */
#include "LTC2983_Configs.h"
#include "stm32l1xx_hal.h"
#include "main.h"
#include "cmsis_os.h"
#include "LTC2983.h"
#include <string.h>

extern SPI_HandleTypeDef hspi1;


#define UPPER_SAMPLE_CHANNEL 10
#define	LOWER_SAMPLE_CHANNEL  7
#define	UPPER_HEATER_CHANNEL 13
#define	LOWER_HEATER_CHANNEL  4
#define	AMBIENT_CHANNEL      16


GPIO_TypeDef * LTC_CS_Ports[6] =
{ LTC1_CS_GPIO_Port, LTC2_CS_GPIO_Port,
LTC3_CS_GPIO_Port, LTC4_CS_GPIO_Port,
LTC5_CS_GPIO_Port, LTC6_CS_GPIO_Port };

GPIO_TypeDef * LTC_RST_Ports[6] =
{ LTC1_RST_GPIO_Port, LTC2_RST_GPIO_Port,
LTC3_RST_GPIO_Port, LTC4_RST_GPIO_Port,
LTC5_RST_GPIO_Port, LTC6_RST_GPIO_Port };

uint16_t LTC_CS_Pins[6] =
{ LTC1_CS_Pin, LTC2_CS_Pin, LTC3_CS_Pin, LTC4_CS_Pin, LTC5_CS_Pin, LTC6_CS_Pin };

uint16_t LTC_RST_Pins[6] =
{ LTC1_RST_Pin, LTC2_RST_Pin, LTC3_RST_Pin, LTC4_RST_Pin, LTC5_RST_Pin,
LTC6_RST_Pin };

// *********************
// SPI RAM data transfer
// *********************
// To write to the RAM, set ram_read_or_write = WRITE_TO_RAM.
// To read from the RAM, set ram_read_or_write = READ_FROM_RAM.
// input_data is the data to send into the RAM. If you are reading from the part, set input_data = 0.

static uint32_t transfer_four_bytes(uint8_t chip, uint8_t ram_read_or_write,
		uint16_t start_address, uint32_t input_data)
{
	uint32_t output_data;
	uint8_t tx[7], rx[7];

	memset(&tx, 0x00, 7);
	memset(&rx, 0x00, 7);

	tx[0] = ram_read_or_write;
	tx[1] = (uint8_t) (start_address >> 8);
	tx[2] = (uint8_t) (start_address & 0xFF);
	tx[3] = (uint8_t) (input_data >> 24);
	tx[4] = (uint8_t) (input_data >> 16);
	tx[5] = (uint8_t) (input_data >> 8);
	tx[6] = (uint8_t) input_data;

	HAL_GPIO_WritePin(LTC_CS_Ports[chip], LTC_CS_Pins[chip], GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, tx, rx, 7, 100);
	HAL_GPIO_WritePin(LTC_CS_Ports[chip], LTC_CS_Pins[chip], GPIO_PIN_SET);

	output_data = (uint32_t) rx[3] << 24 | (uint32_t) rx[4] << 16
			| (uint32_t) rx[5] << 8 | (uint32_t) rx[6];

	return output_data;
}

static uint8_t transfer_byte(uint8_t chip, uint8_t ram_read_or_write,
		uint16_t start_address, uint8_t input_data)
{
	uint8_t tx[4], rx[4];

	tx[0] = ram_read_or_write;
	tx[1] = (uint8_t) (start_address >> 8);
	tx[2] = (uint8_t) start_address;
	tx[3] = input_data;
	HAL_GPIO_WritePin(LTC_CS_Ports[chip], LTC_CS_Pins[chip], GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, tx, rx, 4, 100);
	HAL_GPIO_WritePin(LTC_CS_Ports[chip], LTC_CS_Pins[chip], GPIO_PIN_SET);
	return rx[3];
}

// ******************************
// Misc support functions
// ******************************
static uint16_t get_start_address(uint16_t base_address, uint8_t channel_number)
{
	return base_address + 4 * (channel_number - 1);
}

/*
 static uint8_t is_number_in_array(uint8_t number, uint8_t *array,
 uint8_t array_length)
 // Find out if a number is an element in an array
 {
 uint8_t found = 0;
 for (uint8_t i = 0; i < array_length; i++)
 {
 if (number == array[i])
 {
 found = 1;
 }
 }
 return found;
 }
 */

static void wait_for_process_to_finish(uint8_t chip_select)
{
	uint8_t process_finished = 0;
	uint8_t data;
	while (process_finished == 0)
	{
		osDelay(10);
		data = transfer_byte(chip_select, READ_FROM_RAM,
				COMMAND_STATUS_REGISTER, 0);
		process_finished = data & 0x40;
	}
}

static void assign_channel(uint8_t chip, uint8_t channel_number,
		uint32_t channel_assignment_data)
{
	uint16_t start_address = get_start_address(CH_ADDRESS_BASE, channel_number);
	transfer_four_bytes(chip, WRITE_TO_RAM, start_address,
			channel_assignment_data);
}

static void convert_channel(uint8_t chip_select, uint8_t channel_number)
{
	// Start conversion
	transfer_byte(chip_select, WRITE_TO_RAM, COMMAND_STATUS_REGISTER,
			CONVERSION_CONTROL_BYTE | channel_number);

	wait_for_process_to_finish(chip_select);
}

// *********************************
// Get results
// *********************************
uint32_t get_result(uint8_t chip_select, uint8_t channel_number,
		uint8_t channel_output)
{
	uint32_t raw_data;
	uint16_t start_address = get_start_address(CONVERSION_RESULT_MEMORY_BASE,
			channel_number);

	raw_data = transfer_four_bytes(chip_select, READ_FROM_RAM, start_address,
			0);

	// 24 LSB's are conversion result
	return raw_data;
}

static uint32_t measure_channel(uint8_t chip_select, uint8_t channel_number,
		uint8_t channel_output)
{
	convert_channel(chip_select, channel_number);
	return get_result(chip_select, channel_number, channel_output);
}

static void set_sleep(uint8_t LTCNum)
{
	transfer_byte(LTCNum, WRITE_TO_RAM, COMMAND_STATUS_REGISTER, 0x97);
}

void ConfigureLTCs(void)
{
	for (uint8_t i = 0; i < 6; i++)
	{
		HAL_GPIO_WritePin(LTC_CS_Ports[i], LTC_CS_Pins[i], GPIO_PIN_SET);
	}

	for (uint8_t i = 0; i < 6; i++)
	{
		HAL_GPIO_WritePin(LTC_RST_Ports[i], LTC_RST_Pins[i], GPIO_PIN_RESET);
	}
	osDelay(100);
	for (uint8_t i = 0; i < 6; i++)
	{
		HAL_GPIO_WritePin(LTC_RST_Ports[i], LTC_RST_Pins[i], GPIO_PIN_SET);
	}
	osDelay(300);

	// ----- Check if status register is in correct state (for debug purposes)---------------
/*
	result = transfer_byte(0, READ_FROM_RAM, COMMAND_STATUS_REGISTER, 0);
	result = transfer_byte(1, READ_FROM_RAM, COMMAND_STATUS_REGISTER, 0);
	result = transfer_byte(2, READ_FROM_RAM, COMMAND_STATUS_REGISTER, 0);
	result = transfer_byte(3, READ_FROM_RAM, COMMAND_STATUS_REGISTER, 0);
	result = transfer_byte(4, READ_FROM_RAM, COMMAND_STATUS_REGISTER, 0);
	result = transfer_byte(5, READ_FROM_RAM, COMMAND_STATUS_REGISTER, 0);
*/
	set_sleep(0);
	set_sleep(1);
	set_sleep(2);
	set_sleep(3);
	set_sleep(4);
	set_sleep(5);
}

uint8_t ReadLTCs(float* out, uint8_t LTCNum)
{
	uint32_t measurements[5];
	uint32_t channel_assignment_data = 0;
	uint8_t retval = 0;
	for (uint8_t i = 0; i < 6; i++)
	{
		HAL_GPIO_WritePin(LTC_CS_Ports[i], LTC_CS_Pins[i], GPIO_PIN_RESET);
	}
	osDelay(10);
	for (uint8_t i = 0; i < 6; i++)
	{
		HAL_GPIO_WritePin(LTC_CS_Ports[i], LTC_CS_Pins[i], GPIO_PIN_SET);
	}
	osDelay(200);


	// ----- All Channels: Assign RTD PT-100 -----
	channel_assignment_data = SENSOR_TYPE__RTD_PT_100 | RTD_RSENSE_CHANNEL__2
			| RTD_NUM_WIRES__4_WIRE | RTD_EXCITATION_MODE__ROTATION_SHARING
			| RTD_EXCITATION_CURRENT__100UA | RTD_STANDARD__AMERICAN;
	assign_channel(LTCNum, 4, channel_assignment_data);
	assign_channel(LTCNum, 7, channel_assignment_data);
	assign_channel(LTCNum, 10, channel_assignment_data);
	assign_channel(LTCNum, 13, channel_assignment_data);
	assign_channel(LTCNum, 16, channel_assignment_data);

	// ----- Channel 2: Assign Sense Resistor -----
	channel_assignment_data = SENSOR_TYPE__SENSE_RESISTOR
			| (uint32_t) 0x219800 << SENSE_RESISTOR_VALUE_LSB; // sense resistor - value: 2015.

	assign_channel(LTCNum, 2, channel_assignment_data);

	measurements[0] = measure_channel(LTCNum, UPPER_SAMPLE_CHANNEL, TEMPERATURE); // Ch 4: RTD PT-1000
	measurements[1] = measure_channel(LTCNum, LOWER_SAMPLE_CHANNEL, TEMPERATURE); // Ch 4: RTD PT-1000
	measurements[2] = measure_channel(LTCNum, UPPER_HEATER_CHANNEL, TEMPERATURE); // Ch 4: RTD PT-1000
	measurements[3] = measure_channel(LTCNum, LOWER_HEATER_CHANNEL, TEMPERATURE); // Ch 4: RTD PT-1000
	measurements[4] = measure_channel(LTCNum, AMBIENT_CHANNEL, TEMPERATURE); // Ch 4: RTD PT-1000

	// set LTC to sleep
	set_sleep(LTCNum);

	// if conversion result has any error bits set, set bit in returned value
	retval |= ((measurements[0] & 0xFF000000) != 0x01000000) ? 0x01 : 0x00;
	retval |= ((measurements[1] & 0xFF000000) != 0x01000000) ? 0x02 : 0x00;
	retval |= ((measurements[2] & 0xFF000000) != 0x01000000) ? 0x04 : 0x00;
	retval |= ((measurements[3] & 0xFF000000) != 0x01000000) ? 0x08 : 0x00;
	retval |= ((measurements[4] & 0xFF000000) != 0x01000000) ? 0x10 : 0x00;

	// clear error flags before conversion
	measurements[0] &= 0x00FFFFFF;
	measurements[1] &= 0x00FFFFFF;
	measurements[2] &= 0x00FFFFFF;
	measurements[3] &= 0x00FFFFFF;
	measurements[4] &= 0x00FFFFFF;

	*(out) = ((float) measurements[0]) / 1024.0f; // Ch 4: RTD PT-1000
	*(out + 1) = ((float) measurements[1]) / 1024.0f; // Ch 4: RTD PT-1000
	*(out + 2) = ((float) measurements[2]) / 1024.0f; // Ch 4: RTD PT-1000
	*(out + 3) = ((float) measurements[3]) / 1024.0f; // Ch 4: RTD PT-1000
	*(out + 4) = ((float) measurements[4]) / 1024.0f; // Ch 4: RTD PT-1000

	return retval;
}
