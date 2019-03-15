/*
 * DefaultRoasts.h
 *
 *  Created on: Nov 16, 2018
 *      Author: chaise
 */

#ifndef DEFAULTROASTS_H_
#define DEFAULTROASTS_H_

#include "data_types.h"

#define ROASTLEN 6

extern struct profile LightRoast[ROASTLEN];

extern struct profile MediumRoast[ROASTLEN];

extern struct profile DarkRoast[ROASTLEN];


extern struct profile TestRoast[ROASTLEN];

extern struct profile CustomRoast[ROASTLEN];

// Refer to https://stackoverflow.com/a/28505272 for details on why this is.
//__attribute__((__section__(".roast_data"))) extern const profile_t customRoast[6];



#endif /* DEFAULTROASTS_H_ */
