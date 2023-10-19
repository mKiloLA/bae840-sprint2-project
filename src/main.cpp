#include "Arduino.h"
// #include <QMC5883LCompass.h>

// QMC5883LCompass compass;


// void setup() {
//   Serial.begin(9600);
//   compass.init();
//   compass.setCalibrationOffsets(77.00, -61.00, 66.00);
//   compass.setCalibrationScales(0.80, 1.17, 1.12);
  
// }

// void loop() {
//   int x, y, z;
  
//   // Read compass values
//   compass.read();

//   // Return XYZ readings
//   x = compass.getX();
//   y = compass.getY();
//   z = compass.getZ();
  
//   Serial.print("X: ");
//   Serial.print(x);
//   Serial.print(" Y: ");
//   Serial.print(y);
//   Serial.print(" Z: ");
//   Serial.print(z);
//   Serial.println();
  
//   delay(250);
// }


#include <U8g2lib.h>
#include <Wire.h>
#include "SparkFun_VL53L1X.h"
#include <NewPing.h>

#define SONAR_NUM 2      // Number of sensors.
#define MAX_DISTANCE 400 // Maximum distance (in cm) to ping.

void readSensors();

// Delcare sensor objects
U8G2_SSD1327_EA_W128128_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
SFEVL53L1X distanceSensor;

NewPing sonar[SONAR_NUM] = {   // Sensor object array.
  NewPing(6, 7, MAX_DISTANCE),
  NewPing(4, 5, MAX_DISTANCE)
};

// Sensor variables to hold most up-to-date reading
int distanceTOF;
int distanceRight;
int distanceLeft;

// Constants for moving average filter for ultrasonic
int INDEX = 0;
const int SAMPLE_SIZE = 5;
int valueRight = 0;
int valueLeft = 0;
int sumRight = 0;
int sumLeft = 0;
int distanceRightReadings[SAMPLE_SIZE];
int distanceLeftReadings[SAMPLE_SIZE];

// Setup routine
void setup(void)
{
  Serial.begin(9600);
  u8g2.setBusClock(400000);
  u8g2.begin();

  Wire.begin();

  if (distanceSensor.begin() != 0)
  {
    Serial.println("Sensor failed to begin. Please check wiring. Freezing...");
    while (1);
  }
  Serial.println("Sensor online!");
}

void loop(void)
{
  readSensors();
  u8g2.firstPage();
  do{
    // Indicator bar for the front facing time of flight
    if (distanceTOF < 1000) {
      u8g2.setDrawColor(1);
      u8g2.drawBox(0, 0, 128, 19);
    } else {
      u8g2.setDrawColor(0);
      u8g2.drawBox(0, 0, 128, 19);
    }
    u8g2.setDrawColor(1);
    u8g2.drawBox(21, 80, 86, 19);

    // Indicator bar for the front facing time of flight
    if (distanceTOF < 1300) {
      u8g2.setDrawColor(1);
      u8g2.drawBox(21, 80, 86, 19);
    } else {
      u8g2.setDrawColor(0);
      u8g2.drawBox(21, 80, 86, 19);
    }

    // Indicator bar for the right facing ultrasonic sensor
    if (distanceRight < 100) {
      u8g2.setDrawColor(1);
      u8g2.drawBox(108, 20, 20, 108);
    } else {
      u8g2.setDrawColor(0);
      u8g2.drawBox(108, 20, 20, 108);
    }

    // Indicator bar for the left facing ultrasonic sensor
    if (distanceLeft < 100) {
      u8g2.setDrawColor(1);
      u8g2.drawBox(0, 20, 20, 108);
    } else {
      u8g2.setDrawColor(0);
      u8g2.drawBox(0, 20, 20, 108);
    }

    // Write the distances to the OLED display
    u8g2.setFont(u8g2_font_amstrad_cpc_extended_8f);
    u8g2.setDrawColor(2);
    u8g2.drawStr(48, 32, String(distanceTOF).c_str());
    u8g2.drawStr(72, 64, String(distanceRight).c_str());
    u8g2.drawStr(32, 64, String(distanceLeft).c_str());
  }
  while(u8g2.nextPage());
}

void readSensors(void) {
  // Read time of flight sensor
  distanceSensor.startRanging(); //Write configuration bytes to initiate measurement
  while (!distanceSensor.checkForDataReady()) {
    delay(1);
  }
  distanceTOF = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
  distanceSensor.clearInterrupt();
  distanceSensor.stopRanging();

  // Ping ultrasonic sensors
  sumRight = sumRight - distanceRightReadings[INDEX];
  valueRight = sonar[0].ping_cm();
  distanceRightReadings[INDEX] = valueRight;
  sumRight = sumRight + valueRight;
  distanceRight = sumRight / SAMPLE_SIZE;

  delay(29);

  sumLeft = sumLeft - distanceLeftReadings[INDEX];
  valueLeft = sonar[1].ping_cm();
  distanceLeftReadings[INDEX] = valueLeft;
  sumLeft = sumLeft + valueLeft;
  distanceLeft = sumLeft / SAMPLE_SIZE;
  INDEX = (INDEX + 1) % SAMPLE_SIZE;
}
