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
// #include <cstdint>
#include "msd.h"
#include <algorithm>

I2C i2c(PTE25, PTE24); //SDA,SCL

/*
 * Main function.   
 */
int main() {
    char data[6];
    char cmd[2];
    i2c.frequency(100000);

    /* WHO_AM_I printout */
    cmd[0] = {LSM6DSOX_WHO_AM_I_ADDR};
    i2c.write(LSM6DSOX_ADDR, &cmd[0], 1);
    i2c.read(LSM6DSOX_ADDR, &data[0], 1);
    printf("WHO_AM_I: %x\n", data[0]);

    /* Accelerometer config */
    // CTRL1_XL (Accelerometer control register 1)
    cmd[0] = LSM6DSOX_CTRL1_XL_ADDR;
    cmd[1] = 0xA6; //6.66kHz, +-16g, hi-res
    i2c.write(LSM6DSOX_ADDR, cmd, 2);
    // CTRL8_XL (Control register 8)
    cmd[0] = LSM6DSOX_CTRL8_XL_ADDR;
    i2c.write(LSM6DSOX_ADDR, &cmd[0], 1);
    i2c.read(LSM6DSOX_ADDR, &data[0], 1);
    cmd[1] = data[0] & ~0x02; //clear XL_FS_MODE to retain full +-16g scale
    i2c.write(LSM6DSOX_ADDR, cmd, 2);

    while (true) {
        /* Read accelerometer */
        // CTRL8_XL (Control register 8)
        cmd[0] = LSM6DSOX_OUTX_L_A_ADDR;
        i2c.write(LSM6DSOX_ADDR, &cmd[0], 1);
        i2c.read(LSM6DSOX_ADDR, data, 6); //read all 6 bytes (XYZ regs, 16b each)
        int16_t acc_x = (data[0] << 8 | data[1]);
        int16_t acc_y = (data[2] << 8 | data[3]);
        int16_t acc_z = (data[4] << 8 | data[5]);
        printf("X: %f\n",   (acc_x / 2048.0f));
        printf("Y: %f\n",   (acc_y / 2048.0f));
        printf("Z: %f\n\n", (acc_z / 2048.0f));
        ThisThread::sleep_for(5s);
    }
}


/*
 * I2C Read function.   
 */
void readReg(int address, int subaddress, const char *data, int length = 1) {
    // Set register to read
    const char subaddr[1] = {(char)subaddress};
    i2c.write(address, subaddr, 1);
    // Read register
    char rdata[length+1];
    rdata[0] = (char)subaddress;
    std::copy(&data[0], &data[length-1], &rdata[1]);
    i2c.read(address, rdata, length+1);
}

/*
 * I2C Write function.    
 */
void writeReg(int address, int subaddress, const char *data, int length = 1) {
    // Write register
    char wdata[length+1];
    wdata[0] = (char)subaddress;
    std::copy(&data[0], &data[length-1], &wdata[1]);
    int res1 = i2c.write(address, wdata, length+1);
}