/*
 * File    : main.cpp
 * Author  : Chris Abajian (cxa6282@rit.edu)
 * Created : July 26, 2021
 *
 * Description:
 *   Initial feasibility program for testing the K64F MCU and sensors
 *   established in the Preliminary Detailed Design phase of RIT MSD I.
 */
 
 #include "mbed.h"

I2C i2c(I2C_SDA, I2C_SCL); //PTE25,PTE24

#define LSM6DSOX_ADDR  (0x6A << 1)
#define LSM6DSOX_WHO_AM_I_ADDR  (0x0F)

/*
 * Main function.   
 */
int main() {
    char data[1];
    char cmd[1];
    i2c.frequency(100000);
    while (true) {
        cmd[0] = {LSM6DSOX_WHO_AM_I_ADDR};
        i2c.write(LSM6DSOX_ADDR, cmd, 1, true);
        i2c.read(LSM6DSOX_ADDR, data, 1);
        printf("WHO_AM_I: %x\n", data[0]);
        ThisThread::sleep_for(5s);
    }
}
