/*
 * =====================================================================================
 *
 *       Filename:  max31855.c
 *
 *    Description:  Simple library for handling the data from the max31855 chip.
 *                  Refer to https://github.com/afterthought325/max31855 for the
 *                  latest code revision.
 *
 *        Version:  1.0
 *        Created:  10/28/2018 03:09:28 PM
 *       Compiler:  gcc
 *
 *         Author:  Chaise Farrar (chaise.farrar@gmail.com), 
 *
 * =====================================================================================
 */
#include "max31855.h"


int16_t max31855toCelcius(uint8_t *pu8Data) {
    uint16_t u16RawTemp;
    int16_t i16Celcius;

	u16RawTemp = pu8Data[0]; //The Thermocouple temperature is stored in the upper 14 bits, bit 16 & 17 are not needed.
    u16RawTemp <<= 8;
    u16RawTemp |= pu8Data[1];
    
    if (u16RawTemp & 0x8000) { //temperature is negative
        u16RawTemp = 0xF000 | (u16RawTemp >> 4); // drop lower 4 bits and keep 2's complement

    } else { // temp is positive
        u16RawTemp >>= 4; //drop lower 4 bits.
    }
    i16Celcius = u16RawTemp & 0xFFFF;

    return i16Celcius;
}


int16_t max31855toCelcius_InternalRef(uint8_t *pu8Data) {
    uint16_t *pu16Data = (uint16_t *) pu8Data;
    uint16_t u16RawTemp = pu16Data[0]; //The Reference temp is stored in bits [15:4], bits [3:0] are not needed.
    int16_t i16Celcius;
    
    if (u16RawTemp && 0x8000) { //temp is negative
        u16RawTemp = (u16RawTemp >> 4) | 0xF800; // drop lower four bits and keep 2's complemnt

    } else { // temp is positive
        u16RawTemp >>= 4; //drop lower two bits.
    }

    i16Celcius = u16RawTemp;

    return i16Celcius;
}


/**
 * @brief Returns true if the thermocouple is disconnected.
 * @param pu8Data: array of bytes read from max31855 over SPI 
 * @retval int
 */

int max31855_Disconnected(uint8_t * pu8Data) {
    // If the thermocouple is disconnected, then the first bit is set. 
    // So if you & the first byte with 0x01, then if its disconnected,
    // it will return true 
    return (pu8Data[3] & 0x01);
}

/**
 * \brief Return true if the leads are shorted to ground.
 * @param pu8Data: array of bytes read from max31855 over SPI 
 * @retval int
 */
int max31855_ShortGND(uint8_t * pu8Data) {
    return (pu8Data[3] & 0x02);
}

/**
 * \brief Return true if the leads are shorted to VCC.
 * @param pu8Data: array of bytes read from max31855 over SPI 
 * @retval int
 */
int max31855_ShortVCC(uint8_t * pu8Data) {
    return (pu8Data[3] & 0x04);
}

/**
 * \brief Return true if the thermocouple is disconnnected, shorted to ground, or shorted to VCC.
 * @param pu8Data: array of bytes read from max31855 over SPI 
 * @retval int
 */
int max31855_Error(uint8_t * pu8Data) {
    // bit 16 of the 32 bits is set if bit [0:2] are set,
    // indicating some kind of error.
    return (pu8Data[1] & 0x01); 
}

