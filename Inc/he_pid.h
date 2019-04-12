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
#define Kp 	30
#define Ki 	3
#define Kd  3
#define Kimin -300
#define Kimax 300


/**
 * @brief PID Function for HE (Heating Element)
 * @param[in] i_bTemp 	Bean Temperature
 * @param[in] i_heTemp 	Heating Element Temperature
 * @param[in] i_tTemp		Target Temperature
 * @param[in] reset			Resets all the internal PID loop values to zero (effectively restarting PID)
 * @retval		0			Cut the Heating element off
 * @retval		non-zero	Cut the Heating element On
 */
int HE_PID(int i_bTemp, int i_tTemp, int reset);


#endif /* HE_PID_H_ */
