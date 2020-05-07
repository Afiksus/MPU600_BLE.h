#include"MPU6050_BLE_CONTROL.h"
#include <Wire.h>


MPU_6050 mpu(Wire, 0x68, true, 0.98);

void setup() {
  mpu.DefaultSettings();
  Serial.begin(9600);
  Serial.println("Acceleration:\t\t\tAngle:");
  Serial.println("\tAx|\tAy|\tAz|\tangleX|\tangleY|\tangleZ|");
   delay(1000);
}

void loop() {
  mpu.GetAllData(0x3B);
  
  Serial.print("\t");
  Serial.print(mpu.GetAccX());
  Serial.print("|\t");
  Serial.print(mpu.GetAccY());
  Serial.print("|\t");
  Serial.print(mpu.GetAccZ());
  Serial.print("|\t");
  Serial.print(mpu.GetFilterAngleX());
  Serial.print("|\t");
  Serial.print(mpu.GetFilterAngleY());
  Serial.print("|\t");
  Serial.print(mpu.GetFilterAngleZ());
  Serial.print("|\t");
  Serial.println();
  delay(500);

}
