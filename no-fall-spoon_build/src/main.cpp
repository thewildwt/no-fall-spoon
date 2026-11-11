#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ArduinoEigenDense.h>
#include <Arduino.h>

using namespace Eigen; //for matrix 

//pins
const int servoPins[] = {12 , 14}; //the order matters
const int button_calibrate = {16};

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

//kalman 


struct kalman { //struct for holding kalman variables

  //----time----
  unsigned long timestart;
  unsigned long timestop;
  double seconds; //millis time converted back in seconds
  
  //-----matrices-----
  
  //state variables X_nm at system time n and no. of observations m
  MatrixXf X_bb = MatrixXf::Zero(2,1);
  MatrixXf X_ba = MatrixXf::Zero(2,1);
  MatrixXf X_aa = MatrixXf::Zero(2,1);
  //state covariance 
  MatrixXf P_bb = MatrixXf::Zero(2,2);
  MatrixXf P_ba = MatrixXf::Zero(2,2);
  MatrixXf P_aa = MatrixXf::Zero(2,2);

  MatrixXf F_b = MatrixXf::Zero(2,2); //state transition matrix
  MatrixXf Q_b = MatrixXf::Zero(2,2);  //the covariance of the process noise 
  MatrixXf y_b = MatrixXf::Zero(2,1); // Innovation  residual
  MatrixXf z_b = MatrixXf::Zero(2,1);  // measured data
  MatrixXf S_b = MatrixXf::Zero(2,2); //Innovation covariance
  MatrixXf K_b = MatrixXf::Zero(2,2); //kalman gain
  MatrixXf R_b = MatrixXf::Zero(2,2); //the covariance of the observation noise
  MatrixXf I = MatrixXf::Zero(2,2); //identity matrix

} kalp , kalr;  // kalman pitch and kalman roll

struct angles_tilt  { //storing final angle values
  float pitch;
  float roll;
} ang;

//---------------------------------------------------------
//-------------------functions-----------------------------
//---------------------------------------------------------


//to get initial readings from the mpu
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

//serial monitor for logging and tracking
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


//------offset calculation and recalibration------
void calc_offsets() {
 
  acc.x -= acc_offset.x;
  acc.y -= acc_offset.y;
  acc.z -= acc_offset.z-9.81;

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

//------------kalman filter----------------------

void kalman(){

  //we are doing both roll and pitch calculations in the same place

  //time
  kalp.timestop = millis();
  kalp.seconds = (kalp.timestop - kalp.timestart)/1000 ;
  
  kalr.timestop = millis();
  kalr.seconds = (kalr.timestop - kalr.timestart)/1000 ;

  //predict
  kalp.X_ba = kalp.F_b * kalp.X_aa; //state variable prediction
  kalp.P_ba = kalp.F_b * (kalp.P_aa * kalp.F_b.transpose()) + kalp.R_b; //state covariance prediction

  kalr.X_ba = kalr.F_b * kalr.X_aa; //state variable prediction
  kalr.P_ba = kalr.F_b * (kalr.P_aa * kalr.F_b.transpose()) + kalr.R_b; //state covariance prediction

  //measured values
  kalp.z_b << atan2(acc.z , acc.x) , gyro.y;
  
  kalr.z_b << atan2(acc.z , acc.y) , gyro.x;

  
  //update
  kalp.y_b = kalp.z_b - kalp.X_ba;
  kalp.S_b = kalp.P_ba + kalp.R_b;
  kalp.K_b = kalp.P_ba * (kalp.S_b.inverse());
  kalp.X_bb = kalp.X_ba + kalp.K_b * kalp.y_b;
  kalp.P_bb = (kalp.I - kalp.K_b) * kalp.P_ba;

  kalr.y_b = kalr.z_b - kalr.X_ba;
  kalr.S_b = kalr.P_ba + kalr.R_b;
  kalr.K_b = kalr.P_ba * (kalr.S_b.inverse());
  kalr.X_bb = kalr.X_ba + kalr.K_b * kalr.y_b;
  kalr.P_bb = (kalr.I - kalr.K_b) * kalr.P_ba;

  //storing results 
  ang.pitch = kalp.X_bb(1);
  
  ang.roll = kalr.X_bb(1);

  //resetting for next loop
  kalp.X_aa = kalp.X_bb; 
  kalp.P_aa = kalp.P_bb;
  kalp.timestart = millis();

  kalr.X_aa = kalr.X_bb; 
  kalr.P_aa = kalr.P_bb;
  kalr.timestart = millis();

}

void pid(){
  
}

//--------------------------------------------------------------
// ---------------------setup & loop ---------------------------
//--------------------------------------------------------------



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

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G); // acc sensitivity
  mpu.setGyroRange(MPU6050_RANGE_250_DEG); // gyro sensitivity
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); //dlpf

  //button 
  pinMode(button_calibrate, INPUT_PULLUP);

  //kalman
  kalp.F_b << 1 , kalp.seconds , 0 , 1;
  kalp.I << 1,0,0,1;
  kalp.P_aa << 0,0,0,0;
  kalp.Q_b << 0,0,0,0;
  kalp.R_b << 0,0,0,0;

  kalr.F_b << 1 , kalr.seconds , 0 , 1;
  kalr.I << 1,0,0,1;
  kalr.P_aa << 0,0,0,0;
  kalr.Q_b << 0,0,0,0;
  kalr.R_b << 0,0,0,0;

  kalp.timestart = millis();
  kalr.timestart = millis();


}


void loop() {
  mpu_readings();
  calibrate();
  serial_monitor();
  kalman();
  pid();
}
