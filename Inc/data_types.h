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
enum cmd {
	start = 0x00,
	load,
	stop,
	eject,
};

enum state {
	idle,
	roasting,
	cooling,
	ejecting,
	manual
};
/**
 * @brief Temp Time Pair.
 * An array of such will be sent over Bluetooth from phone to MCU to define profile.
 * total size should be 6 bytes
 */
typedef struct profile {
	int time;
	int temp;
}profile_t;


typedef struct progress_t {
	enum state State;
	int time;
	int bt; // bean temp
	int st; // set temp
	int et; // element temp
	int dc; // duty cycle percentage
	int fs; // fan duty cycle
	int send_update; //control signal between tasks for sending an update to the app
}progress_t;


#endif /* DATA_TYPES_H_ */
