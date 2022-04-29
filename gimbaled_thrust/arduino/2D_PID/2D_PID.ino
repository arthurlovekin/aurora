#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <PID_v1.h>
#include <Servo.h>
#include <utility/imumaths.h>
#include <Wire.h>

//#define DEBUG
#define LOGGING

#define SERVOPIN1 8
#define SERVOPIN2 9

// Servo Control ==============================================================================

Servo servoX, servoY;

void servoInit(){
  servoX.attach(SERVOPIN1);
  servoY.attach(SERVOPIN2);

  #ifdef DEBUG
  Serial.print("servoInit: [Pins ");
  Serial.print(SERVOPIN1);
  Serial.print(" and ");
  Serial.print(SERVOPIN2);
  Serial.println(" initialized as servos]");
  #endif
}

double zeroPointX = 93, zeroPointY = 97; // TODO: figure out parameters

// Set servo to specific angle
void setServo(Servo &servo, double angle, double zeroPoint){
  servo.write(angle + zeroPoint);
}
void setServo(Servo &servo, double angle){
  setServo(servo, angle, 90);
}

// Orientation ===============================================================================

Adafruit_BNO055 bno = Adafruit_BNO055(55);
double bnoOffsetX = 166.3125, bnoOffsetY = -1.0000;

void bnoInit(){
  if(!bno.begin()){
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(1000);
    
  bno.setExtCrystalUse(true);

  #ifdef DEBUG
  Serial.println("bnoInit: [BNO initialized]");
  #endif

  delay(5000);

  #ifdef DEBUG
  Serial.println("bnoInit: [Starting zero-point initialization]");
  #endif

  double sumX = 0, sumY = 0;
  sensors_event_t event;
  for(int i = 0; i < 500; i++){
    bno.getEvent(&event);
    sumX += event.orientation.z;
    sumY += event.orientation.y;
    delay(10);
  }
  bnoOffsetX = sumX / 500;
  bnoOffsetY = sumY / 500;

  #ifdef DEBUG
  Serial.println("bnoInit: [Finished zero-point initialization]");
  Serial.print("bnoInit: [");
  Serial.print(bnoOffsetX, 4);
  Serial.print(", ");
  Serial.print(bnoOffsetY, 4);
  Serial.println("]");
  #endif
}

double sX = 30.3, tX = -31.8, cX = 15.2;
double sY = 31.7, tY = -29.3, cY = 15.2;
// Returns angle servo must turn given a target angle
// target and return value both in DEGREES
// Source: https://drive.google.com/file/d/1zFCwedLtzWXtgwTkhYWoFgtl3uXhsXgb/view?usp=sharing
double servoAngle(double u, double s, double t, double c){
  double b = s; // connector arm length
  double a = -t + c; // TVC mount "arm" length

  // coordinates at the end of the TVC gimbal arm
  double m = s - a * sin(u * M_PI / 180);
  double n = t + a * cos(u * M_PI / 180);
  
  // quick commonly used vars
  double m2 = m*m;
  double n2 = n*n;
  double b2 = b*b;
  double c2 = c*c;
  double m4 = m2*m2;
  double n4 = n2*n2;
  double b4 = b2*b2;
  double c4 = c2*c2;
  
  double p = (-b2*m-sqrt(-b4*n2 + 2*b2*c2*n2 + 2*b2*m2*n2 + 2*b2*n4 - c4*n2 + 2*c2*m2*n2 + 2*c2*n4 - m4*n2 - 2*m2*n4 - n2*n4) + c2*m + m*m2 + m*n2)/(2*(m2+n2));
  double q = sqrt(c2-p*p);
  
  return -atan2(p,q)*180/M_PI;

  // simple approximation
  /*double a = -t + c;
  return u * a / c;*/
}

// Current orientation
double angleX, angleY;

// Update current orientation
void updateOrientation(){
  sensors_event_t event; 
  bno.getEvent(&event);
  
  angleX = event.orientation.z;
  angleY = event.orientation.y;

  /*#ifdef DEBUG
  Serial.print("updateOrientation: [");
  Serial.print("X: ");
  Serial.print(angleX, 4);
  Serial.print("\tY: ");
  Serial.print(angleY, 4);
  Serial.println("]");
  #endif*/
}

double inputAngleX, inputAngleY; // Saves current angle offset, used for PID control
// Given current angle, calculate relative the 2 relative angles to the initial orientation
void calculateAngleOffset(){
  if(angleX < 0)
    inputAngleX = angleX + 360 - bnoOffsetX;
  else
    inputAngleX = angleX - bnoOffsetX;
  inputAngleY = angleY - bnoOffsetY;
  // CALIBRATE: bno zero point
}

// PID ============================================================================================

// Specify the links and initial tuning parameters
double Kp = 5, Ki = 0, Kd = 2;
double outputAngleX, outputAngleY, setPointX, setPointY;
PID PIDX(&inputAngleX, &outputAngleX, &setPointX, Kp, Ki, Kd, DIRECT);
PID PIDY(&inputAngleY, &outputAngleY, &setPointY, Kp, Ki, Kd, DIRECT);

void PIDInit(){
  angleX = angleY = 0;
  setPointX = setPointY = 0; // TODO: this should be adjustable during initialization state
  
  PIDX.SetMode(AUTOMATIC);
  PIDY.SetMode(AUTOMATIC);
  PIDX.SetOutputLimits(-5, 5);
  PIDY.SetOutputLimits(-5, 5);

  #ifdef DEBUG
  Serial.println("PIDInit: [PID initialized]");
  #endif
}

// State Control ==================================================================================

// Program states
// TODO: add when necessary
// STATUS: DEFUNCT
/*
enum {initialization, PIDrunning};
int programState = initialization;
*/

void setup() {
  Serial.begin(9600);
  
  servoInit();
  bnoInit();
  PIDInit();
}

void loop() {
  updateOrientation();
  calculateAngleOffset();
  
  PIDX.Compute();
  PIDY.Compute();

  #ifdef LOGGING
  Serial.print(millis());
  Serial.print(", ");
  Serial.print(inputAngleX, 4);
  Serial.print(", ");
  Serial.print(inputAngleY, 4);
  Serial.print(", ");
  Serial.print(outputAngleX, 4);
  Serial.print(", ");
  Serial.print(outputAngleY, 4);
  Serial.print(", ");
  Serial.print(servoAngle(outputAngleX, sX, tX, cX), 4);
  Serial.print(", ");
  Serial.print(servoAngle(outputAngleY, sY, tY, cY), 4);
  Serial.println("");
  #endif

  setServo(servoX, servoAngle(outputAngleX, sX, tX, cX), zeroPointX);
  setServo(servoY, servoAngle(-outputAngleY, sY, tY, cY), zeroPointY);

  delay(5); // TODO: remove
}
