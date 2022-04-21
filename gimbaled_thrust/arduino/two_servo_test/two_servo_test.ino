#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>

#define SERVOPIN1 8
#define SERVOPIN2 9

Servo xServo, yServo;  // create servo object to control a servo

Adafruit_BNO055 bno = Adafruit_BNO055(55);

void setup(void) 
{
  Serial.begin(9600);
  Serial.println("Servo and BNO055 Test"); Serial.println("");

  xServo.attach(SERVOPIN1);
  yServo.attach(SERVOPIN2);
  
  /* Initialise the sensor */
  if(!bno.begin()){
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(1000);
    
  bno.setExtCrystalUse(true);
}

int xPos = 90, yPos = 90;
void loop(void) 
{
  /* Get a new sensor event */ 
  sensors_event_t event; 
  bno.getEvent(&event);
  
  /* Display the floating point data */
  xPos = 90 + event.orientation.z / 6;
  yPos = 90 + event.orientation.y / 6;
  Serial.print("X: ");
  Serial.print(xPos);
  Serial.print("\tIMUX: ");
  Serial.print(event.orientation.z);
  Serial.print("\tY: ");
  Serial.print(yPos);
  Serial.print("\tIMUY: ");
  Serial.print(event.orientation.y);
  Serial.println("");
  
  delay(15);
  
  xServo.write(xPos);
  yServo.write(yPos);
}
