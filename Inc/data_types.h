/*
 * data_types.h
 *
 *  Created on: Nov 12, 2018
 *      Author: chaise
 */

#ifndef DATA_TYPES_H_
#define DATA_TYPES_H_


#include "stdint.h"

/**
 * if start, machine will begin roasting using currently loaded profile.
 * if load, then the temp and time will be stored in the profile according to the index.
 * if stop, pause roast.
 * if eject, end roast and blast out beans.
 */
enum cmd_t {
	start = 0x00,
	load,
	stop,
	eject,
};
/**
 * @brief Temp Time Pair.
 * An array of such will be sent over Bluetooth from phone to MCU to define profile.
 * total size should be 6 bytes
 */
typedef struct profile_t {
	int time;
	int temp;
} profile_t;



typedef struct commands_t {
	enum cmd_t cmd;
	uint8_t index;
	int temp; /**< profile_t#temp is a temperature in Celcius that the beans should hit at the time indicated in profile_t#time. */
	int time; /**< profile_t#time is the time in seconds at which the beans should hit the temp indicated in profile_t#temp */
} commands_t;

/**
 * @brief An array of such will be sent over from the MCU to the phone during the the roast
 */
typedef struct progress_t {
	uint16_t b_temp; /**< progress_t#b_temp is the current temperature of the beans */
	uint16_t he_temp; /**< progress_t#he_temp is the current temperature of the heating element */
	uint16_t time; /**< progress_t#time is the current time of the roast in seconds */
} progress_t;


#endif /* DATA_TYPES_H_ */
