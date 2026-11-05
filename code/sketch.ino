#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <MPU6050_light.h>

//pins
const int servoPins[] = {12 , 14}; //the order matters

//definitions
Servo servo[2];
MPU6050 mpu = MPU6050(Wire);

//mpu variables
struct acceleration{
  float x;
  float y;
  float z;
} acc;

struct gyroscope{
  float x;
  float y;
  float z;
} gyro;


//-------------------functions---------------------------


void mpu_readings(){
  // Update, this keeps the library going
  mpu.update();
  
  acc.x = mpu.getAccX();
  acc.y = mpu.getAccY();
  acc.z = mpu.getAccZ();

  gyro.x = mpu.getGyroX();
  gyro.y = mpu.getGyroY();
  gyro.z = mpu.getGyroZ();
}


// ---------------------setup & loop ---------------------------
void setup() {
  
  //serial monitor
  Serial.begin(115200);
  Serial.println();

  // attaching servos
  servo[0].attach(servoPins[0], 500, 2400);
  servo[1].attach(servoPins[1], 500, 2400);

  //mpu
  Serial.println(F("Initializing mpu"));
  Wire.begin();  // default pins: SDA=21, SCL=22.
  byte status = mpu.begin();
  if(status==0){
    Serial.println(F("Initializing mpu"));
  }
  else{
    Serial.print(F("error = "));
    Serial.print(status);
  }
  mpu.calcOffsets(true,true); // gyro and accelometer
}


void loop() {
  mpu_readings();
  Serial.println(acc.x);
  delay(15);
}
