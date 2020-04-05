/*
 * LTC2983.h
 *
 *  Created on: 24.11.2019
 *      Author: Szymon
 */

#ifndef LTC2983_H_
#define LTC2983_H_

// Performs channel configuration
void ConfigureLTCs(void);

// Reads array of U12 temperatures from all six LTCs
uint8_t ReadLTCs(float* out, uint8_t LTCNum);

#endif /* LTC2983_H_ */
