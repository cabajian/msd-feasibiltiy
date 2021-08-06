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
#include "msd.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <imumaths.h>
#include <jet.h>

I2C i2c(PTE25, PTE24); //SDA,SCL
Adafruit_BNO055 bno = Adafruit_BNO055(-1, BNO055_ADDRESS_A, &i2c);

/*
 * Main function.   
 */
int main() {
    uint8_t bno_sys_stat, bno_self_test, bno_sys_err;
    char data[14];
    i2c.frequency(100000);

    /* Initialize the Orientation Board */
    if(!bno.begin()) {
        printf("BNO055 not detected!\n");
        while(1);
    } else {
        printf("BNO055 was detected!\r\n");
    }
    ThisThread::sleep_for(1s);
    writeReg(BNO055_ADDRESS_A<<1, bno.BNO055_OPR_MODE_ADDR, bno.OPERATION_MODE_NDOF);
    bno.setExtCrystalUse(true);
    bno.getSystemStatus(&bno_sys_stat, &bno_self_test, &bno_sys_err);
    printf("BNO055 system status: %d\t%d\t%d\n", bno_sys_stat, bno_self_test, bno_sys_err);

    /* Initialize the LSM6DSOX */
    // CTRL3_C (Control register 3)
    //   set self-clearing reset bit
    setBits(LSM6DSOX_ADDR, LSM6DSOX_CTRL3_C_ADDR, 0x01);
    readReg(LSM6DSOX_ADDR, LSM6DSOX_CTRL3_C_ADDR, data, 1);
    while(data[0] & 0x01) {
        wait_us(100);
        // Read until reset bit clears
        readReg(LSM6DSOX_ADDR, LSM6DSOX_CTRL3_C_ADDR, data, 1);
    }
    // CTRL3_C (Control register 3)
    //   set BDU to only update output regs after reading
    setBits(LSM6DSOX_ADDR, LSM6DSOX_CTRL3_C_ADDR, 0x40);
    /* Accelerometer config */
    // CTRL1_XL (Accelerometer control register 1)
    //   416Hz, +-8g
    writeReg(LSM6DSOX_ADDR, LSM6DSOX_CTRL1_XL_ADDR, 0x6A);
    double acc_sens = 0.244; // 0.244 milli-g per bit in +-8g scale
    // CTRL8_XL (Control register 8)
    //   clear XL_FS_MODE to retain full +-16g scale
    clearBits(LSM6DSOX_ADDR, LSM6DSOX_CTRL8_XL_ADDR, 0x02);
    // CTRL9_XL (Control register 9)
    //   set "I3C disable" bit since we aren't using it
    setBits(LSM6DSOX_ADDR, LSM6DSOX_CTRL9_XL_ADDR, 0x01);
    /* Gyroscope config */
    // CTRL2_G (Gyroscope control register 2)
    //   416Hz, 2000dps
    writeReg(LSM6DSOX_ADDR, LSM6DSOX_CTRL2_G_ADDR, 0x6C);
    double gyr_sens = 70.0; // 70.0 milli-dps per bit in 2000dps scale

    // jet_setup(bno);
    // while (true) {
    //   jet_loop(bno);
    // }

    while (true) {
        /* Orientation Board Reading */
        printf("----- ORIENTATION BOARD -----\n");
        int8_t bno_temp_i = bno.getTemp();
        printf("Temp: %d\n", bno_temp_i);
        // Possible vector values can be:
        // - VECTOR_ACCELEROMETER - m/s^2
        // - VECTOR_MAGNETOMETER  - uT
        // - VECTOR_GYROSCOPE     - rad/s
        // - VECTOR_EULER         - degrees
        // - VECTOR_LINEARACCEL   - m/s^2
        // - VECTOR_GRAVITY       - m/s^2
        imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
        /* Display the floating point data */
        printf("X: %f Y: %f Z: %f\n", euler.x(), euler.y(), euler.z());
        /* Display calibration status for each sensor on the Orientation Board */
        uint8_t system, gyro, accel, mag = 0;
        bno.getCalibration(&system, &gyro, &accel, &mag);
        printf("CALIBRATION: Sys=%d, Gyro=%d, Accel=%d, Mag=%d\n", (int)(system), (int)(gyro), (int)(accel), (int)(mag));


        /* LSM6DSOX Reading */
        printf("----- LSM6DSOX IMU -----\n");
        readReg(LSM6DSOX_ADDR, LSM6DSOX_OUT_TEMP_L_ADDR, data, 14);
        int16_t temp_i = (data[1] << 8 | data[0]);
        double  temp_d = (temp_i/256.0) + 25.0;
        int16_t gyr_ix = (data[3] << 8 | data[2]);
        int16_t gyr_iy = (data[5] << 8 | data[4]);
        int16_t gyr_iz = (data[7] << 8 | data[6]);
        double  gyr_dx = gyr_ix * gyr_sens * SENSORS_DPS_TO_RADS / 1000;
        double  gyr_dy = gyr_iy * gyr_sens * SENSORS_DPS_TO_RADS / 1000;
        double  gyr_dz = gyr_iz * gyr_sens * SENSORS_DPS_TO_RADS / 1000;
        int16_t acc_ix = (data[9]  << 8 | data[8]);
        int16_t acc_iy = (data[11] << 8 | data[10]);
        int16_t acc_iz = (data[13] << 8 | data[12]);
        double  acc_dx = acc_ix * acc_sens * SENSORS_GRAVITY_STANDARD / 1000;
        double  acc_dy = acc_iy * acc_sens * SENSORS_GRAVITY_STANDARD / 1000;
        double  acc_dz = acc_iz * acc_sens * SENSORS_GRAVITY_STANDARD / 1000;
        printf("Temp: %f\n",   temp_d);
        printf("GyrX: %f\n",   gyr_dx);
        printf("GyrY: %f\n",   gyr_dy);
        printf("GyrZ: %f\n",   gyr_dz);
        printf("AccX: %f\n",   acc_dx);
        printf("AccY: %f\n",   acc_dy);
        printf("AccZ: %f\n\n", acc_dz);
        ThisThread::sleep_for(1s);
    }
}


/*
 * I2C Read helper function.   
 */
void readReg(int address, uint8_t subaddress, char *data, int length = 1) {
    // Set register to read
    char subaddr[1] = {subaddress};
    i2c.write(address, subaddr, 1);
    // Read register
    i2c.read(address, data, length);
}

/*
 * I2C Write helper function.    
 */
void writeReg(int address, uint8_t subaddress, uint8_t command) {
    // Write register
    char wdata[2] = {subaddress, command};
    i2c.write(address, wdata, 2);
}

/*
 * I2C Set bits function.    
 */
void setBits(int address, uint8_t subaddress, uint8_t mask) {
    char data[1];
    readReg(address, subaddress, data, 1);
    data[0] |= mask;
    writeReg(address, subaddress, data[0]);
}

/*
 * I2C Clear bits function.    
 */
void clearBits(int address, uint8_t subaddress, uint8_t mask) {
    char data[1];
    readReg(address, subaddress, data, 1);
    data[0] &= ~mask;
    writeReg(address, subaddress, data[0]);
}