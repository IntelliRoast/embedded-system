/*
 * he_pid.c
 *
 *  Created on: Nov 3, 2018
 *      Author: chaise
 */

#include "he_pid.h"

/**
 * @brief PID Function for HE (Heating Element)
 * @param[in] i_bTemp 	Bean Temperature
 * @param[in] i_heTemp 	Heating Element Temperature
 * @param[in] i_tTemp		Target Temperature
 * @param[in] reset			Resets all the internal PID loop values to zero (effectively restarting PID)
 * @retval		0			Cut the Heating element off
 * @retval		non-zero	Cut the Heating element On
 */
int HE_PID(int i_bTemp, int i_tTemp, int reset) {
	static int i_Error;
	static int i_lastError;
	static int i_Integral;
	static int i_Derivative;
	int i_PWM;

	if (reset) {
		i_Error = 0;
		i_Integral = 0;
		i_Derivative = 0;
	}
	/* Calculate Proportional component */
	i_Error = i_tTemp - i_bTemp;
	/* Calculate Integral component */
	i_Integral += i_Error;
	/* Calculate Derivative component */
	i_Derivative = i_Error - i_lastError;

	if(i_Integral > Kimax) i_Integral = Kimax;
	if(i_Integral > Kimin) i_Integral = Kimin;

	i_PWM = (Kp * i_Error) + (Ki * i_Integral) + (Kd * i_Derivative);

	if (i_PWM > 1000) i_PWM = 1000;
	if (i_PWM < 0) i_PWM = 0;
	i_lastError = i_Error;

	return i_PWM;
}

