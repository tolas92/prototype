#include "Arduino.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

/* Variable initialization */
Adafruit_MPU6050 mpu;

void read_imu(sensors_event_t a) {
  Serial.print(a.acceleration.x);
  Serial.print(" ");
  Serial.print(a.acceleration.y);
  Serial.print(" ");
  Serial.println(a.acceleration.z);
}

void setup() {
  // Open the serial port at 115200 bps
  Serial.begin(115200);

  // Setup the MPU6050.
  mpu.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void loop() {
  sensors_event_t a, g, temp;

  mpu.getEvent(&a, &g, &temp);

  while (Serial.available() > 0) {
    // Read the next character
    char chr = Serial.read();

    // Check for 'e' command to print accelerometer values
    if (chr == 'e') {
      read_imu(a);
    }
  }

}
