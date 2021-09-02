#include "mbed.h"
#include <jet.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <imumaths.h>

void jet_setup(Adafruit_BNO055 bno) {
  // #if 1    // my BNO is mounted with its dot rearward/right/up so choose P3 (see BNO055 datasheet)
  //   bno.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P3);
  //   bno.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P3);
  // #endif
  bno.setMode(bno.OPERATION_MODE_NDOF);
  ThisThread::sleep_for(1ms);
}

void jet_loop(Adafruit_BNO055 bno) {
  imu::Quaternion q = bno.getQuat();
  // flip BNO/Adafruit quaternion axes to aerospace: x forward, y right, z down
  float temp = q.x();  q.x() = q.y();  q.y() = temp;  q.z() = -q.z();

  static imu::Quaternion tare = {1,0,0,0};
//   if (!digitalRead(BUTTON_TARE))
//     tare = q.conjugate();
//   #if 0   // North-referenced: attach sensor to aircraft any which way, fly level towards magnetic north, click button
//     q = q * tare;
//   #else   // Screen-referenced: aim sensor towards your screen, click button
//     q = tare * q;
//   #endif
  q.normalize();

  // convert aerospace quaternion to aerospace Euler, because BNO055 Euler data is broken
  float heading = 180/M_PI * atan2(q.x()*q.y() + q.w()*q.z(), 0.5 - q.y()*q.y() - q.z()*q.z());
  float pitch   = 180/M_PI * asin(-2.0 * (q.x()*q.z() - q.w()*q.y()));
  float roll    = 180/M_PI * atan2(q.w()*q.x() + q.y()*q.z(), 0.5 - q.x()*q.x() - q.y()*q.y());
  heading = heading < 0 ? heading+360 : heading;  // wrap heading to 0..360 convention

  #if 0    // send quaternion
    Serial.print(F("Quaternion: "));
    Serial.print(q.w(), 4);
    Serial.print(F(" "));
    Serial.print(q.x(), 4);
    Serial.print(F(" "));
    Serial.print(q.y(), 4);
    Serial.print(F(" "));
    Serial.print(q.z(), 4);
    Serial.println(F(""));
  #endif
  #if 0    // send Euler angles, my preferred format
    printf("HeadingPitchRoll: %f %f %f\n", heading, pitch, roll);
    // printf("%f", heading);  // heading, nose-right is positive
    // printf(" ");
    // printf("%f", pitch);    // pitch, nose-up is positive
    // printf(" ");
    // printf("%f", roll);     // roll, leftwing-up is positive
    // printf("");
  #endif
  #if 1    // send Euler angles, alternate (Adafruit?) format
    sensors_event_t evnt;
    bno.getEvent(&evnt);
    printf("Orientation: %f %f %f\n", evnt.orientation.x, evnt.orientation.y, evnt.orientation.z);
  #endif
  #if 0    // send calibration status
    uint8_t sys, gyro, accel, mag = 0;
    bno.getCalibration(&sys, &gyro, &accel, &mag);
    Serial.print(F("Calibration: "));
    Serial.print(sys, DEC);
    Serial.print(F(" "));
    Serial.print(gyro, DEC);
    Serial.print(F(" "));
    Serial.print(accel, DEC);
    Serial.print(F(" "));
    Serial.println(mag, DEC);
  #endif

  ThisThread::sleep_for(10ms);
}

