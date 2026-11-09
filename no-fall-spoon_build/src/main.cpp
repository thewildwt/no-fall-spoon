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

void serial_monitor(){
  
  Serial.println("Acceleration");
  Serial.print("X:  ");
  Serial.print(acc.x);
  Serial.print("      ");
  Serial.print("Y:  ");
  Serial.print(acc.y);
  Serial.print("      ");
  Serial.print("Z:  ");
  Serial.print(acc.z);
  Serial.println("\n");

  Serial.println("Gyroscope");
  Serial.print("X:  ");
  Serial.print(gyro.x);
  Serial.print("      ");
  Serial.print("Y:  ");
  Serial.print(gyro.y);
  Serial.print("      ");
  Serial.print("Z:  ");
  Serial.print(gyro.z);
  Serial.println("");
  for(int i=0;i<60;i++){
    Serial.print("-");
  }
  Serial.println("");
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
  serial_monitor();

  delay(1000);

}
