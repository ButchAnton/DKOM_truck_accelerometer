// Combined Accelerometer and Wireless Xbee shield code.
// Added a bunch of extra stuff, like auto-calibration of the
// accelerometer.
// Butch Anton, 9-Jan-2015.
//
// Updated to support accessing the accelerometer using an OSEPP Shield
// 2012-11-22 by William Day <code@william-day.com>

// I2C device class (I2Cdev) demonstration Arduino sketch for ADXL345 class
// 10/7/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     2011-10-07 - initial release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"

// I2Cdev and ADXL345 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "ADXL345.h"
#include "OSEPPShield.h"
#include <SoftwareSerial.h>
#include <stdio.h>

// Reporting interval for data in milliseconds.

#define REPORTING_INTERVAL (100)

// Seperator between reporting elements for remote data.

#define DATA_SEPARATOR ","

// One gravity expressed in feet per second

#define ONE_GEE (32.0)

// Set up the XBee pins for the wireless shield.
// When you use the Rx/Tx pins on the Arduino,
// you just use the Serial interface, not the
// Xbee interface, to write to the Xbee.

#define Rx    1
#define Tx    0
SoftwareSerial Xbee (Rx, Tx);


// class default I2C address is 0x53
// specific I2C addresses may be passed as a parameter here
// ALT low = 0x53 (default for SparkFun 6DOF board)
// ALT high = 0x1D (OSEPP board!)
ADXL345 accel(0x1D);

// create the interface to the OSEPP Shield using the default
// address (dip switches 0,0,0)
OSEPPShield shield(0,0,0);

int16_t ax, ay, az;

float    max_g;
uint8_t  range; // 0 = 2, 1 = 4, 2 = 8, 3 = 16
uint16_t max_range;

#define LED_PIN 13 // (Arduino is 13, Teensy is 6)
bool blinkState = false;

float distance = 0.0;
float velocity = 0.0;
unsigned long timeInMilliseconds = 0;
float previousVelocity = 0.0;
unsigned long previousTimeInMilliseconds = 0;
float deltaTimeInSeconds = 0.0;

float offset_ax = 0.0;
float offset_ay = 0.0;
float offset_az = 0.0;

float corrected_ax = 0.0;
float corrected_ay = 0.0;
float corrected_az = 0.0;

void setup() {

    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(9600);

    // Start the XBee

    Xbee.begin(9600);		    //   type a char, then hit enter
    delay(500);


    // Tell the OSEPP Shield that we want to communicate with devices plugged into
    // port 0
    shield.select_port(0);

    // initialize device
    /**** Serial.println("Initializing I2C devices..."); */
    accel.initialize();

    // verify connection
    /**** Serial.println("Testing device connections..."); */
    /**** Serial.println(accel.testConnection() ? "ADXL345 connection successful" : "ADXL345 connection failed"); */

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);

    // set to +- 2G range
    set_range(0x0);

    // Calibrate the sensor
    calculateOffset();

}

// Sample the accelerometer ten times, and then take
// the average the five readings.  Invert the sign
// and call this the steady state of the sensor.
// Add it to raw readings to get "true"
// readings from the sensor.

void calculateOffset() {

  float axSum = 0.0;
  float aySum = 0.0;
  float azSum = 0.0;

  int i = 0;

  for (i = 0; i < 10; i++) {

    accel.getAcceleration(&ax, &ay, &az);

    axSum += scale(ax);
    aySum += scale(ay);
    azSum += scale(az);

    // Send some marker output to the Xbee radio
    // so that we know we just calibrated.

    Serial.print(0); Serial.print(DATA_SEPARATOR);
    Serial.print(0.0); Serial.print(DATA_SEPARATOR);
    Serial.print(0.0); Serial.print(DATA_SEPARATOR);
    Serial.print(0.0); Serial.print(DATA_SEPARATOR);
    Serial.print(0.0);
    Serial.println();
  }

  offset_ax = (axSum / 10.0) * -1.0;
  offset_ay = (aySum / 10.0) * -1.0;
  offset_az = (azSum / 10.0) * -1.0;

}


// see http://www.analog.com/static/imported-files/data_sheets/ADXL345.pdf
// for notes on the following functions
void set_range(uint8_t range) {
  if (accel.getFullResolution()) {
    max_g     = (float)(2 << range); // max gravity: 2,  4,  8,  16
    max_range = ((1 << 10) << range) >> 1;  // bit depths:  10, 11, 12, 13; >> 1 since centered around 0
    accel.setRange(range);
  } else {
    max_g     = (float)(2 << range); // max gravity: 2,  4,  8,  16
    max_range = (1 << 10) >> 1;  // bit depths:  10, 11, 12, 13; >> 1 since centered around 0
    accel.setRange(range);
  }
}

float scale(int16_t accel) {
  return ((float)accel) / max_range * max_g;
}

void loop() {

  timeInMilliseconds = millis();
  deltaTimeInSeconds = (((float)timeInMilliseconds - (float)previousTimeInMilliseconds) / 1000.0);
  previousTimeInMilliseconds = timeInMilliseconds;

  // Read raw accel measurements from device

  accel.getAcceleration(&ax, &ay, &az);

  // Apply calibration offsets

  corrected_ax = scale(ax) + offset_ax;
  corrected_ay = scale(ay) + offset_ay;
  corrected_az = scale(az) + offset_az;

  // Calculate speed

  velocity = previousVelocity + (corrected_ax * ONE_GEE * deltaTimeInSeconds);
  previousVelocity = velocity;

  // Calculate distance

  distance += (velocity * deltaTimeInSeconds);

  // Display and send the following data:
  //
  // Timestamp in milliseconds since program started
  // x-axis acceleration
  // y-axis acceleration
  // speed
  // distance

  Serial.print(timeInMilliseconds); Serial.print(DATA_SEPARATOR);
  Serial.print(corrected_ax); Serial.print(DATA_SEPARATOR);
  Serial.print(corrected_ay); Serial.print(DATA_SEPARATOR);
  Serial.print(velocity); Serial.print(DATA_SEPARATOR);
  Serial.print(distance);
  Serial.println();

  delay(REPORTING_INTERVAL);
}
