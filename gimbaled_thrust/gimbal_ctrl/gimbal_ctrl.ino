#include <Servo.h>

Servo xServo;
Servo yServo;
const int xServoPin=1;
const int yServoPin=2;

//PID Constants
double kp=0.4;
double ki=0.1;
double kd=0.1;

//angle limits for the physical TVC
double maxAngle = 5;
double minAngle = -5;

//control loop rate (hz)
double frequency = 100;

//initialze for later
double iX=0;
double iY=0;
double prevErrorX=0;
double prevErrorY=0;
double timePrev;

void setup() {
  xServo.attach(xServoPin);
  yServo.attach(yServoPin);
  timePrev=millis();
}

void loop() {
  //////////////////////////////////////////////////////////
  //INCLUDE CODE TO GRAB ANGLE DATA FROM IMU AND STORE
  double angleX = 1; //deg
  double angleY = 1;
  //////////////////////////////////////////////////////////
  
  double errorX = angleX; //assuming the desired angle is 0
  double errorY = angleY;

  double pX = kp*errorX;
  double pY = kp*errorY;

  iX = iX + ki*errorX;
  iY = iY = ki*errorY;

  //find change in time
  double timeCurrent = millis();
  double deltaT = (timeCurrent-timePrev)/1000.0;
  timePrev=timeCurrent;
  
  double dX = kd*(errorX-prevErrorX)/deltaT;
  double dY = kd*(errorY-prevErrorY)/deltaT;
  prevErrorX = errorX;
  prevErrorY = errorY;

  double pidX = pX+iX+dX;
  double pidY = pY+iY+dY;

  //enforce physical limits
  if(pidX>maxAngle)
  {
    pidX=maxAngle;
  }
  else if(pidX<minAngle)
  {
    pidX=minAngle;
  }
  if(pidY>maxAngle)
  {
    pidY=maxAngle;
  }
  else if(pidY<minAngle)
  {
    pidY=minAngle;
  }

  xServo.write(pidX);
  yServo.write(pidY);

  delay(round(1.0/frequency));
  
}
