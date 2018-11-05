/*
 * he_pid.h
 *
 *  Created on: Nov 3, 2018
 *      Author: chaise
 */

#ifndef HE_PID_H_
#define HE_PID_H_

#include "stdint.h"

/* Defines used in PID function */
#define Kp 	0.7
#define Ki 	0.03
#define Kd  2


/**
 * @brief PID Function for HE (Heating Element)
 * @param[in] i16_bTemp 	Bean Temperature
 * @param[in] i16_heTemp 	Heating Element Temperature
 * @param[in] i16_tTemp		Target Temperature
 * @param[in] reset			Resets all the internal PID loop values to zero (effectively restarting PID)
 * @retval		0			Cut the Heating element off
 * @retval		non-zero	Cut the Heating element On
 */
uint8_t HE_PID(int16_t i16_bTemp, int16_t i16_tTemp, uint8_t reset);


#endif /* HE_PID_H_ */
