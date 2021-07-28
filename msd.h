/*
 * File    : main.cpp
 * Author  : Chris Abajian (cxa6282@rit.edu)
 * Created : July 26, 2021
 *
 * Description:
 *   Initial feasibility program for testing the K64F MCU and sensors
 *   established in the Preliminary Detailed Design phase of RIT MSD I.
 */

#ifndef MSD_INCLUDE_H
#define MSD_INCLUDE_H

/* I2C read/write functions */
void readReg(int address, int subaddress, const char *data, int length);
void writeReg(int address, int subaddress, const char *data, int length);

/* LSM6DSOX macros */
#define LSM6DSOX_ADDR           (0x6A<<1)
#define LSM6DSOX_WHO_AM_I_ADDR  (0x0F)
#define LSM6DSOX_CTRL1_XL_ADDR  (0x10)
#define LSM6DSOX_CTRL8_XL_ADDR  (0x17)
#define LSM6DSOX_OUTX_L_A_ADDR  (0x28)

#endif