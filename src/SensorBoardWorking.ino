#include <Arduino.h>
#include "MPU9250.h"
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_MS8607.h>
#include <math.h>
#define SEALEVELPRESSURE_HPA (1013.25)
#define LSM_CS 5
#define LSM_SCK 18
#define LSM_MISO 19
#define LSM_MOSI 23
Adafruit_LSM6DSOX sox;
Adafruit_MS8607 ms8607;
uint16_t measurement_delay_us = 65535; // Delay between measurements for testing
MPU9250 mpu;
bool IGNITABLE = false;
int count = 0;
TwoWire I2C_one(0);
TwoWire I2C_two(1);

//recovery code w maddy
//serial plotter/ filtering
//send code to ed
//

void setup() {
  Serial.begin(115200);
  while (!Serial){
    Serial.print("Serial Failed to start");
    delay(10);
  }

  //6 DOF
  Serial.println("Adafruit LSM6DSOX test!");
      if (!sox.begin_SPI(LSM_CS, LSM_SCK, LSM_MISO, LSM_MOSI)) {
        Serial.println("Failed to find LSM6DSOX chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("LSM6DSOX Found!");

// Altimeter
  I2C_one.begin(13,14);
  I2C_one.setClock(400000);
  if (!ms8607.begin(&I2C_one)) {
    Serial.println("Failed to find MS8607 chip");
    while (1) {
      delay(10); }
  }
  Serial.println("MS8607 Found, SETUP");
// 9 DOF
  I2C_two.begin(21, 22);
  I2C_two.setClock(400000);
  delay(2000);
  if (!mpu.setup(0x68, MPU9250Setting(), I2C_two)) {  
      while (1) {
        Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
        delay(5000);
      }
  }

}

void loop() {
  sensors_event_t temp, pressure, humidity;
  ms8607.getEvent(&pressure, &temp, &humidity);
  double pressure_alt = pressure.pressure;
  sensors_event_t accel1;
  sensors_event_t gyro1;
  sensors_event_t temp1;
  sox.getEvent(&accel1, &gyro1, &temp1);


  double Ax = accel1.acceleration.x;
  double Ay = accel1.acceleration.y;
  double Az = accel1.acceleration.z;
  double Net_Accel = sqrt((pow(Ax, 2) + pow(Ay, 2) + pow(Az, 2)));
// PHT
Serial.print("Temp: ");
Serial.print(temp.temperature); 
Serial.print(" degrees C");
Serial.print(" Pressure: ");
Serial.print(pressure.pressure); 
Serial.print(" hPa");
Serial.print(" Humid: ");
Serial.print(humidity.relative_humidity); 
Serial.println(" %rH");
double altitude = 44330.0 * (1.0 - pow(pressure_alt / SEALEVELPRESSURE_HPA, 0.1903));
//IGNITION
// double AltArray[READINGS_LENGTH];
// switch (true) {
//   case (count < READINGS_LENGTH) {
//     AltArray[count] = altitude
//   }
//   break;
//   case (count >= READINGS_LENGTH) {
//     for (int i = 0; i < READINGS_LENGTH - 1; i++){
//     Alt[i] = Alt[i+1];
//     }
//     Alt[READINGS_LENGTH - 1] = altitude;
//   }
// }
// double rate_of_change;
//   if(Net_Accel > 20) {
//     IGNITABLE = true;
//   }
//   if (count >= READINGS_LENGTH)
//   {
//     rate_of_change = return_average(Alt, READINGS_LENGTH);
//   }
//   delay(2000);
// switch (true) {
//   case (IGNITABLE && rate_of_change <=0 && altitude >= 900) {
//     digitalWrite(1, HIGH);
//     Serial.print("Ignition 1 occured");
//     delay(100);
//   }
//   break;
//   case (IGNITABLE && rate_of_change <=0 && altitude <= 1000 && ) { // should another condition be that ignition 1 occured, or that a time has passed? if rate of change doesnt go below 0 on the way up it will be fine.
//     digitalWrite(2, HIGH);
//     Serial.print("Ignition 2 occured");
//   }
//   break;
// }

// 6 DEGREES OF FREEDOM
Serial.println("(6DOF)");
  Serial.print("Ax: " + String(Ax) + " ");
  Serial.print("Ay: " + String(Ay) + " ");
  Serial.print("Az: " + String(Az) + " ");
  Serial.print("Gx: " + String(gyro1.gyro.x) + " ");              //  rads/sec
  Serial.print("Gy: " + String(gyro1.gyro.y) + " ");
  Serial.println("Gz: " + String(gyro1.gyro.z) + " ");
  Serial.println("(9DOF)");
if (mpu.update()) {
  static uint32_t prev_ms = millis();
    if (millis() > prev_ms + 25) {
      Serial.print("Yaw: ");      
      Serial.print(mpu.getYaw(), 2);
      Serial.print(" Pitch: ");   
      Serial.print(mpu.getPitch(), 2);
      Serial.print(" Roll:");   
      Serial.println(mpu.getRoll(), 2);
      Serial.print("Ax: ");   
      Serial.print(mpu.getAccX(), 2);
      Serial.print(" Ay: ");   
      Serial.print(mpu.getAccY(), 2);
      Serial.print(" Az: ");   
      Serial.println(mpu.getAccZ(), 2);
            prev_ms = millis();
        }
    }
  Serial.print("ALTITUDE: " +String(altitude));
  Serial.println(" ACCELERATION: " +String(Net_Accel));
  // 9DOF

// Serial.println(String(IGNITABLE));   1 if ignitable condition
Serial.println();
  delay(1000);
}






          


