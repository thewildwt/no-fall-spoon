#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

//pins
const int servoPins[] = {12 , 14}; //the order matters
const int button_calibrate = {16};

//addresses
const int mpuAddress = 0x68;

//definitions
Servo servo[2];
Adafruit_MPU6050 mpu;

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

//buttons
int buttonState;             // the current reading from the input pin
int lastButtonState = HIGH;  // the previous reading from the input pin

long lastDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 50;    // the debounce time; increase if the output flickers


//offset
struct acceleration acc_offset;
struct gyroscope gyro_offset;
bool offset_state;


//-------------------functions---------------------------


void mpu_readings(){
  
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  acc.x = a.acceleration.x;
  acc.y = a.acceleration.y;
  acc.z = a.acceleration.z;

  gyro.x = g.gyro.x;
  gyro.y = g.gyro.y;
  gyro.z = g.gyro.z;
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

void calc_offsets() {
 
  acc.x -= acc_offset.x;
  acc.y -= acc_offset.y;
  acc.z -= acc_offset.z-1;

  gyro.x -= gyro_offset.x;
  gyro.y -= gyro_offset.y;
  gyro.z -= gyro_offset.z;
}


void calibrate(){
  int reading = digitalRead(button_calibrate);

  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {

    // if the button state has changed:
    if (reading != buttonState) {
      buttonState = reading;

      // only toggle the LED if the new button state is HIGH
      if (buttonState == LOW) {
        acc_offset = acc;
        gyro_offset = gyro;
        offset_state = true;
      }
    }
  }

  lastButtonState = reading;
  
  if (offset_state == true) {
    calc_offsets();
  }

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
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  /*mpu.calcOffsets(true,true); // gyro and accelometer

  Wire.beginTransmission(mpuAddress);*/

  //button 
  pinMode(button_calibrate, INPUT_PULLUP);

}


void loop() {
  mpu_readings();
  calibrate();
  serial_monitor();
  
}
