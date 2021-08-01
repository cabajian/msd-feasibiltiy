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
void readReg(int address, uint8_t subaddress, char *data, int length);
void writeReg(int address, uint8_t subaddress, uint8_t command);
void setBits(int address, uint8_t subaddress, uint8_t mask);
void clearBits(int address, uint8_t subaddress, uint8_t mask);

/* LSM6DSOX macros */
#define LSM6DSOX_ADDR           (0x6A<<1)
#define LSM6DSOX_WHO_AM_I_ADDR  (0x0F)
#define LSM6DSOX_CTRL1_XL_ADDR  (0x10)
#define LSM6DSOX_CTRL2_G_ADDR   (0x11)
#define LSM6DSOX_CTRL3_C_ADDR   (0x12)
#define LSM6DSOX_CTRL8_XL_ADDR  (0x17)
#define LSM6DSOX_CTRL9_XL_ADDR  (0x18)
#define LSM6DSOX_OUT_TEMP_L_ADDR (0x20)
#define LSM6DSOX_OUTX_L_G_ADDR  (0x22)
#define LSM6DSOX_OUTX_L_A_ADDR  (0x28)

#endif