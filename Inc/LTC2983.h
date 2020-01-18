/*
 * LTC2983.h
 *
 *  Created on: 24.11.2019
 *      Author: Szymon
 */

#ifndef LTC2983_H_
#define LTC2983_H_

typedef enum LtcReadMode_t_def
{
	UPPER_SAMPLE_MODE,
	LOWER_SAMPLE_MODE,
	UPPER_HEATER_MODE,
	LOWER_HEATER_MODE,
	AMBIENT_MODE
} LtcReadMode_t;

// Performs channel configuration
void ConfigureLTCs(void);

// Reads array of U12 temperatures from all six LTCs
void ReadLTCs(float* out, LtcReadMode_t mode);

#endif /* LTC2983_H_ */
